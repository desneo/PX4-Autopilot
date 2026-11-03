#!/usr/bin/env python3
"""编译实际解析器和发布函数，验证 ASD 全字段映射及与两轴报文的隔离。"""
import json
import math
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parent
MSG = ROOT.parents[3] / 'msg' / 'DytAishengTelemetry.msg'
OMEGA = ('los_omega_n', 'los_omega_e', 'los_omega_d')
ARRAYS = ('target_type', 'target_confidence', 'target_lon', 'target_lat', 'target_alt')


def schema():
    return [line.split('#', 1)[0].split() for line in MSG.read_text().splitlines()
            if line.split('#', 1)[0].strip()]


def full_packet():
    packet = {}
    for index, (kind, name) in enumerate(schema()):
        if name in OMEGA:
            continue
        if '[' in kind:
            packet[name] = [index + value * 0.25 for value in range(4)] if kind.startswith('float') else [index + value for value in range(4)]
        elif kind.startswith('float'):
            packet[name] = -(index + 0.125)
        else:
            packet[name] = index
    packet.update(timestamp=1793606400123456, device_timestamp=1234567890,
                  track_status=1, autonomous=1, target_lon=[1161234567, -1222345678, 0, 2147483647],
                  target_lat=[401234567, -355555555, -2147483648, 0])
    return packet


class CompleteTelemetryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix='px4-aisheng-json-')
        cls.addClassCleanup(cls.temp.cleanup)
        directory = Path(cls.temp.name)
        header = directory / 'uORB' / 'topics' / 'dyt_aisheng_telemetry.h'
        header.parent.mkdir(parents=True)
        types = dict(uint8='uint8_t', uint16='uint16_t', uint32='uint32_t', uint64='uint64_t',
                     int16='int16_t', int32='int32_t', float32='float')
        members = []
        outputs = []
        cls.names = []
        for kind, name in schema():
            if '[' in kind:
                kind, count = kind.rstrip(']').split('[')
                members.append(f'{types[kind]} {name}[{count}] {{}};')
                for index in range(int(count)):
                    outputs.append(f" << ' ' << +msg.{name}[{index}]")
                    cls.names.append(f'{name}[{index}]')
            else:
                members.append(f'{types[kind]} {name} {{}};')
                outputs.append(f" << ' ' << +msg.{name}")
                cls.names.append(name)
        header.write_text('#pragma once\n#include <cstdint>\nstruct dyt_aisheng_telemetry_s {\n'
                          + '\n'.join(members) + '\n};\n')
        source = (ROOT / 'AishengUdpBridge.cpp').read_text()
        start = source.index('\tvoid publish(dyt_aisheng_telemetry_s &message)')
        publish = source[start:source.index('\n\tuint16_t _port', start)]
        harness = r'''
#include "AishengJsonParser.hpp"
#include <cstdio>
#include <iostream>
#include <iomanip>
#include <string>
#define PX4_ERR(...) fprintf(stderr, __VA_ARGS__)
uint64_t hrt_absolute_time() { return 424242; }
struct CapturePublisher {
    dyt_aisheng_telemetry_s message{};
    bool publish(const dyt_aisheng_telemetry_s &value) { message=value; return true; }
};
class ProductionPublisher {
public:
PUBLISH_SOURCE
    CapturePublisher _publisher{};
    uint32_t _publications{0};
    uint64_t _last_rx{0};
};
int main() {
    std::string input;
    while (std::getline(std::cin, input)) {
        dyt_aisheng_telemetry_s parsed{};
        aisheng_udp::JsonParser parser(input.data(), input.size());
        const bool ok = input.size() < aisheng_udp::kUdpBufferSize && parser.parse(parsed);
        ProductionPublisher bridge;
        if (ok) { bridge.publish(parsed); }
        const auto &msg = bridge._publisher.message;
        std::cout << ok << ' ' << bridge._publications << std::setprecision(17)
OUTPUT_SOURCE << '\n';
    }
}
'''.replace('PUBLISH_SOURCE', publish).replace('OUTPUT_SOURCE', '\n'.join(outputs))
        cpp = directory / 'parser_test.cpp'
        cpp.write_text(harness)
        cls.executable = directory / 'parser_test'
        subprocess.run(['g++', '-std=c++14', '-O2', '-Wall', '-Wextra', '-Werror',
                        '-I', str(directory), '-I', str(ROOT), str(cpp), '-o', str(cls.executable)],
                       check=True, capture_output=True, text=True)

    def parse(self, payload):
        if isinstance(payload, dict):
            payload = json.dumps(payload, separators=(',', ':'))
        output = subprocess.run([str(self.executable)], input=payload + '\n', check=True,
                                capture_output=True, text=True).stdout.strip().split()
        self.assertEqual(len(output), len(self.names) + 2)
        self.publications = int(output[1])
        self.message = dict(zip(self.names, map(float, output[2:])))
        return bool(int(output[0]))

    def assertRejected(self, payload):
        self.assertFalse(self.parse(payload), payload)
        self.assertEqual(self.publications, 0)

    def test_all_37_fields_and_all_four_targets_map_to_uorb(self):
        packet = full_packet()
        self.assertEqual(len(packet), 37)
        self.assertTrue(self.parse(packet))
        self.assertEqual(self.publications, 1)
        for name, value in packet.items():
            if name == 'timestamp':
                continue
            values = enumerate(value) if isinstance(value, list) else [(None, value)]
            for index, expected in values:
                key = f'{name}[{index}]' if index is not None else name
                self.assertEqual(self.message[key], expected, key)
        self.assertEqual(self.message['timestamp'], 424242)
        self.assertTrue(all(math.isnan(self.message[name]) for name in OMEGA))

    def test_integer_limits_and_raw_signed_coordinates_keep_precision(self):
        packet = full_packet()
        for kind, name in schema():
            if name not in packet:
                continue
            base = kind.split('[')[0]
            if base.startswith('uint'):
                maximum = 2 ** int(base[4:]) - 1
                if name in ('track_status', 'autonomous'):
                    maximum = 1
                packet[name] = [maximum] * 4 if '[' in kind else maximum
        packet.update(gate_x=32767, gate_y=0, target_lon=[-2147483648, 2147483647, -1, 0])
        self.assertTrue(self.parse(packet))
        self.assertEqual(self.message['device_timestamp'], 4294967295)
        self.assertEqual(self.message['target_lon[0]'], -2147483648)
        self.assertEqual(self.message['target_lon[1]'], 2147483647)
        self.assertEqual(self.message['timestamp'], 424242)

    def test_each_complete_field_is_required(self):
        for name in full_packet():
            with self.subTest(field=name):
                packet = full_packet()
                del packet[name]
                self.assertRejected(packet)

    def test_target_arrays_require_four_valid_elements(self):
        for name in ARRAYS:
            for bad in (0.8, None, [], [1], [1, 2, 3], [1, 2, 3, 4, 5], [1, 2, 3, None],
                        [1, 2, 3, True], [1, 2, 3, []]):
                with self.subTest(field=name, bad=bad):
                    packet = full_packet()
                    packet[name] = bad
                    self.assertRejected(packet)

    def test_scalar_types_and_ranges_are_strict(self):
        for name, invalid in (
                ('self_check', [-1, 256, 1.0, True, None, '1']),
                ('track_status', [2, -1]), ('autonomous', [2]),
                ('gate_x', [-1, 32768, 1.2]), ('gate_y', [-1, 32768]),
                ('tracking_gate_w', [-1, 65536, 100.5]),
                ('device_timestamp', [-1, 4294967296, 1.25]),
                ('timestamp', [-1, 18446744073709551616, 1.5]),
                ('pitch_angle', [None, True, '10', float('nan'), float('inf'), 1e39])):
            for bad in invalid:
                with self.subTest(field=name, bad=bad):
                    packet = full_packet()
                    packet[name] = bad
                    self.assertRejected(packet)

    def test_raw_coordinate_and_target_type_ranges(self):
        for name, bad in (('target_lon', -2147483649), ('target_lat', 2147483648),
                          ('target_lon', 116.12), ('target_type', 256), ('target_type', -1),
                          ('target_confidence', 1e39)):
            with self.subTest(field=name, bad=bad):
                packet = full_packet()
                packet[name][3] = bad
                self.assertRejected(packet)

    def test_unknown_structured_fields_are_skipped(self):
        packet = full_packet()
        packet['metadata'] = {'flags': [True, False, None], 'text': '\u6d4b\u8bd5', 'value': 1e-6}
        self.assertTrue(self.parse(packet))

    def test_duplicate_keys_including_escaped_spelling_are_rejected(self):
        payload = json.dumps(full_packet(), separators=(',', ':'))
        for key in ('pitch_angle', 'pitch_\\u0061ngle'):
            self.assertRejected(payload[:-1] + f',"{key}":1' + '}')

    def test_integer_tokens_do_not_round_or_accept_exponents(self):
        payload = json.dumps(full_packet(), separators=(',', ':'))
        for token in ('01', '+1', '1e0', '1.0', '1.000000000000000000000001', '1e-1000'):
            self.assertRejected(payload.replace('"self_check":1', f'"self_check":{token}'))

    def test_float_exponents_are_supported(self):
        packet = full_packet()
        packet.update(pitch_angle=-1.5e-4, laser_range=1.25e3)
        self.assertTrue(self.parse(packet))
        self.assertAlmostEqual(self.message['pitch_angle'], -1.5e-4, places=10)
        self.assertEqual(self.message['laser_range'], 1250)

    def test_malformed_trailing_and_truncated_json_is_rejected(self):
        payload = json.dumps(full_packet(), separators=(',', ':'))
        for bad in ('{}', '', payload[:-1], payload + 'x', payload + '{}', payload[:-1] + ',}',
                    payload.replace('"pitch_angle":-4.125', '"pitch_angle":1e'),
                    payload.replace('"pitch_angle":-4.125', '"pitch_angle":1.')):
            with self.subTest(payload=bad[-50:]):
                self.assertRejected(bad)

    def test_deep_unknown_values_and_oversize_packets_are_rejected(self):
        packet = full_packet()
        nested = None
        for _ in range(10):
            nested = [nested]
        packet['extra'] = nested
        self.assertRejected(packet)
        packet['extra'] = 'x' * 4096
        self.assertRejected(packet)

    def test_zero_statuses_are_preserved_without_fabricated_defaults(self):
        packet = full_packet()
        for name in ('self_check', 'work_status', 'track_status', 'status', 'status2', 'autonomous'):
            packet[name] = 0
        self.assertTrue(self.parse(packet))
        for name in ('self_check', 'work_status', 'track_status', 'status', 'status2', 'autonomous'):
            self.assertEqual(self.message[name], 0)


if __name__ == '__main__':
    unittest.main()
