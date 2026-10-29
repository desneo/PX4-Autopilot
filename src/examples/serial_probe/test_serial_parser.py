#!/usr/bin/env python3
"""直接编译生产串口解析方法，验证帧恢复及连续输入的返回性，不访问串口硬件。"""

import importlib.util
import json
from pathlib import Path
import re
import struct
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parent
REPOSITORY = ROOT.parents[2]

SIMULATOR_SPEC = importlib.util.spec_from_file_location(
    "aisheng_serial_sim", REPOSITORY / "agent_zsh/aisheng_serial_sim.py")
SIMULATOR = importlib.util.module_from_spec(SIMULATOR_SPEC)
SIMULATOR_SPEC.loader.exec_module(SIMULATOR)


def aisheng_frame():
    frame = bytearray(104)
    frame[:5] = bytes((0x55, 0xAA, 0x68, 1, 0xA5))
    for offset, value in ((5, -12345), (7, 2345), (9, -321), (11, 654),
                          (13, -123), (15, 456), (17, -789), (19, 1234),
                          (22, -1500), (24, 1750)):
        struct.pack_into("<h", frame, offset, value)
    frame[21] = 199
    frame[26:28] = bytes((0x5A, 1))
    for target in range(4):
        struct.pack_into("<BBiiH", frame, 28 + target * 12, target + 1, 40 + target,
                         -123456789 + target, 234567890 + target, 1234 + target)
    frame[76] = 0x9D
    struct.pack_into("<HHhhhBH", frame, 77, 640, 480, -1025, 2050, -3075, 4, 1234)
    frame[90] = 0x0D
    struct.pack_into("<hhHBBI", frame, 91, -4500, 9000, 35999, 1, 56, 0x12345678)
    frame[103] = sum(frame[3:103]) & 0xFF
    return bytes(frame)


def zhongke_frame():
    frame = bytearray(32)
    frame[:4] = bytes((0xEE, 0x16, 0x12, 0xA0))
    frame[4:6] = bytes((0x34, 0x51))
    struct.pack_into("<hhhhh", frame, 6, -100, 200, -300, 400, -500)
    struct.pack_into("<hhhH", frame, 20, -600, 700, -800, 1234)
    frame[28] = 0x84
    frame[31] = sum(frame[:31]) & 0xFF
    return bytes(frame)


def message_stub(filename, name):
    """按当前 .msg 声明生成DTO，协议偏移和缩放仍由生产方法决定。"""
    types = {"float32": "float", "float64": "double", "bool": "bool"}
    types.update({f"{sign}{bits}": f"{sign}{bits}_t"
                  for sign in ("int", "uint") for bits in (8, 16, 32, 64)})
    declarations = []
    output = [f"void dump_message(const {name} &message) {{", 'std::cout << "{";']
    first = True
    for line in (REPOSITORY / "msg" / filename).read_text().splitlines():
        line = line.partition("#")[0].strip()
        if not line or "=" in line:
            continue
        type_name, field = line.split()
        match = re.fullmatch(r"(\w+)(?:\[(\d+)\])?", type_name)
        scalar, length = match.groups()
        declarations.append(f"{types[scalar]} {field}{'[' + length + ']' if length else ''}{{}};")
        prefix = ("" if first else ",") + json.dumps(field) + ":"
        output.append(f"std::cout << {json.dumps(prefix)};")
        if length:
            output += ['std::cout << "[";',
                       f"for (unsigned i = 0; i < {length}; ++i) {{",
                       f"std::cout << (i ? \",\" : \"\") << +message.{field}[i];", "}",
                       'std::cout << "]";']
        else:
            output.append(f"std::cout << +message.{field};")
        first = False
    output += ['std::cout << "}";', "}"]
    return f"struct {name} {{\n" + "\n".join(declarations) + "\n};\n", "\n".join(output)


class ProductionSerialParserTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(prefix="px4-serial-parser-")
        cls.addClassCleanup(cls.temporary.cleanup)
        directory = Path(cls.temporary.name)
        source = (ROOT / "serial_probe.cpp").read_text()
        helpers = source[source.index("namespace\n{"):source.index("\nclass SerialProbe")]
        methods = source[source.index("\tvoid receive(uint8_t byte)"):source.index('\n};\n\nextern "C"')]
        structures, dumps = zip(
            message_stub("DytAishengTelemetry.msg", "dyt_aisheng_telemetry_s"),
            message_stub("DytZhongkeTelemetry.msg", "dyt_zhongke_telemetry_s"))
        harness = r'''
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include <iomanip>

STRUCTURES
DUMPS
static uint64_t clock_us = 1000000;
uint64_t hrt_absolute_time() { return ++clock_us; }
#define PX4_ERR(...) do { fprintf(stderr, __VA_ARGS__); fputc('\n', stderr); } while (false)
#define ORB_ID(name) 0
namespace uORB {
template<typename T> class Publication {
public:
    explicit Publication(int) {}
    bool publish(const T &message) { ++count; last = message; return true; }
    uint64_t count{0};
    T last{};
};
}
HELPERS
class SerialParserHarness {
public:
METHODS
    void dump() const {
        std::cout << "{\"aisheng_frames\":" << _aisheng_frames
            << ",\"zhongke_frames\":" << _zhongke_frames
            << ",\"aisheng_checksum_errors\":" << _aisheng_checksum_errors
            << ",\"aisheng_length_errors\":" << _aisheng_length_errors
            << ",\"zhongke_checksum_errors\":" << _zhongke_checksum_errors
            << ",\"noise_bytes\":" << _noise_bytes
            << ",\"total_bytes\":" << _total_bytes
            << ",\"buffered\":" << _length
            << ",\"aisheng_publications\":" << _aisheng_pub.count
            << ",\"zhongke_publications\":" << _zhongke_pub.count
            << ",\"aisheng\":";
        dump_message(_aisheng_pub.last);
        std::cout << ",\"zhongke\":";
        dump_message(_zhongke_pub.last);
        std::cout << "}\n";
    }
};
int main() {
    SerialParserHarness parser;
    std::cout << std::setprecision(9);
    std::string command;
    while (std::cin >> command) {
        unsigned repetitions = 1;
        if (command == "repeat") { std::cin >> repetitions; }
        std::string hex;
        std::cin >> hex;
        std::vector<uint8_t> bytes;
        for (size_t i = 0; i < hex.size(); i += 2) {
            bytes.push_back(static_cast<uint8_t>(std::stoul(hex.substr(i, 2), nullptr, 16)));
        }
        for (unsigned n = 0; n < repetitions; ++n) {
            for (uint8_t byte : bytes) { parser.receive(byte); }
        }
        parser.dump();
    }
}
'''.replace("STRUCTURES", "\n".join(structures)).replace("DUMPS", "\n".join(dumps))
        harness = harness.replace("HELPERS", helpers).replace("METHODS", methods)
        cpp = directory / "serial_parser_test.cpp"
        cpp.write_text(harness)
        cls.executable = directory / "serial_parser_test"
        subprocess.run(["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror",
                        str(cpp), "-o", str(cls.executable)], check=True,
                       capture_output=True, text=True)

    def parse(self, chunks, repetitions=None):
        commands = [f"{'repeat ' + str(repetitions) if repetitions else 'feed'} {chunk.hex()}"
                    for chunk in chunks]
        result = subprocess.run([str(self.executable)], input="\n".join(commands) + "\n",
                                capture_output=True, text=True, check=True, timeout=10)
        self.assertEqual(result.stderr, "")
        states = [json.loads(line) for line in result.stdout.splitlines()]
        self.assertEqual(len(states), len(commands))
        for state in states:
            self.assertLess(state["buffered"], 104, state)
            self.assertEqual(state["aisheng_frames"], state["aisheng_publications"], state)
            self.assertEqual(state["zhongke_frames"], state["zhongke_publications"], state)
        return states

    def test_serial_port_parameter_integration(self):
        source = (ROOT / "serial_probe.cpp").read_text()
        cmake = (ROOT / "CMakeLists.txt").read_text()
        module_config = (ROOT / "module.yaml").read_text()
        micoair_board = (REPOSITORY / "boards/micoair/h743-v2/default.px4board").read_text()
        micoair_defaults = (REPOSITORY / "boards/micoair/h743-v2/init/rc.board_defaults").read_text()
        micoair_extras = (REPOSITORY / "boards/micoair/h743-v2/init/rc.board_extras").read_text()
        cuav_board = (REPOSITORY / "boards/cuav/7-nano/default.px4board").read_text()
        cuav_defaults = (REPOSITORY / "boards/cuav/7-nano/init/rc.board_defaults").read_text()

        self.assertNotIn("/dev/ttyS3", source)
        self.assertIn("const char *device = nullptr;", source)
        self.assertIn('print_usage("no device specified")', source)
        self.assertIn("MODULE_CONFIG", cmake)
        self.assertIn("module.yaml", cmake)
        self.assertIn("name: DYT_PORT_CONFIG", module_config)
        self.assertIn("serial_probe start -d ${SERIAL_DEV}", module_config)
        self.assertIn('CONFIG_BOARD_SERIAL_TEL3="/dev/ttyS4"', micoair_board)
        self.assertIn("CONFIG_EXAMPLES_SERIAL_PROBE=y", micoair_board)
        self.assertIn("param set-default DYT_BRAND 1", micoair_defaults)
        self.assertIn("param set-default DYT_PORT_CONFIG 103", micoair_defaults)
        self.assertNotIn("serial_probe start", micoair_extras)
        self.assertIn("CONFIG_EXAMPLES_SERIAL_PROBE=y", cuav_board)
        self.assertIn("param set-default DYT_PORT_CONFIG 102", cuav_defaults)
        self.assertFalse((REPOSITORY / "boards/cuav/7-nano/init/rc.board_extras").exists())

    def test_valid_aisheng_104_byte_frame_decodes_fields_and_publishes_once(self):
        state = self.parse([aisheng_frame()])[-1]
        self.assertEqual(state["aisheng_publications"], 1)
        self.assertEqual(state["zhongke_publications"], 0)
        self.assertEqual(state["total_bytes"], 104)
        message = state["aisheng"]
        for field, expected in {"self_check": 1, "work_status": 0xA5, "track_status": 1,
                                "gate_x": -321, "gate_y": 654, "status": 0x5A,
                                "autonomous": 1, "status2": 0x9D, "tracking_gate_w": 640,
                                "tracking_gate_h": 480, "tracking_target_type": 4,
                                "laser_update": 0x0D, "ip_segment": 1, "ip_node": 56,
                                "device_timestamp": 0x12345678}.items():
            self.assertEqual(message[field], expected, field)
        for field, expected in {"pitch_angle": -123.45, "yaw_angle": 23.45,
                                "gyro_pitch_rate": -1.23, "gyro_yaw_rate": 4.56,
                                "pitch_misalignment": -7.89, "yaw_misalignment": 12.34,
                                "tracking_confidence": 19.9, "pitch_los_rate": -15,
                                "yaw_los_rate": 17.5, "pitch_attitude": -10.25,
                                "yaw_attitude": 20.5, "roll_attitude": -30.75,
                                "laser_range": 123.4, "aircraft_pitch": -45,
                                "aircraft_roll": 90, "aircraft_yaw": 359.99}.items():
            self.assertAlmostEqual(message[field], expected, delta=.0001, msg=field)
        self.assertEqual(message["target_type"], [1, 2, 3, 4])
        self.assertEqual(message["target_lon"], [-123456789 + i for i in range(4)])
        self.assertEqual(message["target_lat"], [234567890 + i for i in range(4)])
        for i in range(4):
            self.assertAlmostEqual(message["target_confidence"][i], (40 + i) * .01, delta=.0001)
            self.assertAlmostEqual(message["target_alt"][i], (1234 + i) * .1, delta=.0001)
        self.assertGreater(message["timestamp"], 0)
        self.assertNotEqual(message["timestamp"], message["device_timestamp"])

    def test_pc_serial_simulator_frame_is_accepted(self):
        frame = SIMULATOR.build_frame(123)
        state = self.parse([frame])[-1]
        self.assertEqual(len(frame), 104)
        self.assertEqual(frame[:3], bytes((0x55, 0xAA, 0x68)))
        self.assertEqual(sum(frame[3:103]) & 0xFF, frame[103])
        self.assertEqual(state["aisheng_publications"], 1)
        self.assertEqual(state["aisheng"]["gate_x"], 922)
        self.assertEqual(state["aisheng"]["gate_y"], 501)
        self.assertEqual(state["aisheng"]["device_timestamp"], 123)

    def test_every_fragment_boundary_waits_until_frame_complete(self):
        frame = aisheng_frame()
        for split in range(1, len(frame)):
            with self.subTest(split=split):
                states = self.parse([frame[:split], frame[split:]])
                self.assertEqual(states[0]["aisheng_publications"], 0)
                self.assertEqual(states[1]["aisheng_publications"], 1)

    def test_bytewise_delivery_and_concatenated_frames(self):
        frame = aisheng_frame()
        states = self.parse([frame[i:i + 1] for i in range(len(frame))] + [frame * 7])
        self.assertTrue(all(state["aisheng_publications"] == 0 for state in states[:103]))
        self.assertEqual(states[103]["aisheng_publications"], 1)
        self.assertEqual(states[-1]["aisheng_publications"], 8)

    def test_noise_bad_length_and_bad_checksum_each_recover(self):
        frame = aisheng_frame()
        bad_length = bytearray(frame)
        bad_length[2] = 0x67
        bad_checksum = bytearray(frame)
        bad_checksum[-1] ^= 0x40
        cases = ((b"\x00\xff\x55\x00\xaa\x11" * 30, "noise_bytes"),
                 (bytes(bad_length), "aisheng_length_errors"),
                 (bytes(bad_checksum), "aisheng_checksum_errors"))
        for damaged, counter in cases:
            with self.subTest(counter=counter):
                states = self.parse([damaged, frame])
                self.assertEqual(states[0]["aisheng_publications"], 0)
                self.assertEqual(states[-1]["aisheng_publications"], 1)
                self.assertGreater(states[-1][counter], 0)

    def test_valid_frame_embedded_after_false_header_is_retained(self):
        prefix = bytearray(b"\x55\xaa\x68" + b"\x00" * 20)
        frame = aisheng_frame()
        candidate = prefix + frame
        if (sum(candidate[3:103]) & 0xFF) == candidate[103]:
            prefix[3] ^= 1
        state = self.parse([bytes(prefix) + frame])[-1]
        self.assertEqual(state["aisheng_publications"], 1)
        self.assertGreater(state["aisheng_checksum_errors"], 0)

    def test_both_vendor_headers_dispatch_without_a_brand_gate(self):
        state = self.parse([zhongke_frame() + aisheng_frame() + zhongke_frame()])[-1]
        self.assertEqual(state["aisheng_publications"], 1)
        self.assertEqual(state["zhongke_publications"], 2)
        self.assertEqual(state["zhongke"]["status1"], 0x12)
        self.assertAlmostEqual(state["zhongke"]["offset_x"], -5, delta=.0001)
        self.assertAlmostEqual(state["zhongke"]["laser_range"], 123.4, delta=.0001)

    def test_tracking_status_uses_work_status_low_bit(self):
        frame = bytearray(aisheng_frame())
        frame[4] &= 0xFE
        frame[-1] = sum(frame[3:103]) & 0xFF
        state = self.parse([bytes(frame)])[-1]
        self.assertEqual(state["aisheng"]["track_status"], 0)
        self.assertEqual(state["aisheng_publications"], 1)

    def test_continuous_burst_and_noise_return_with_exact_publication_counts(self):
        count = 20000
        states = self.parse([aisheng_frame(), b"\x55\x00\xaa\xff"], repetitions=count)
        self.assertEqual(states[0]["aisheng_publications"], count)
        self.assertEqual(states[1]["aisheng_publications"], count)
        self.assertEqual(states[1]["total_bytes"], count * (104 + 4))
        self.assertGreater(states[1]["noise_bytes"], count)


if __name__ == "__main__":
    unittest.main()
