#!/usr/bin/env python3
"""编译生产解析器、UDP 接收与发布函数，核对中科测量及协议隔离。"""
import json
from pathlib import Path
import struct
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parent
MSG = ROOT.parents[3] / 'msg' / 'DytZhongkeTelemetry.msg'
BYTE_FIELDS = ('status1', 'status2', 'status3', 'self_check')


def schema():
    return [line.split('#', 1)[0].split() for line in MSG.read_text().splitlines()
            if line.split('#', 1)[0].strip()]


def full_packet():
    return dict(timestamp=1793606400123, status1=132, status2=8, status3=242,
                zoom=51.3, offset_x=-12.35, offset_y=6.45, frame_roll=-3.21,
                frame_pitch=-89.75, frame_yaw=123.45, roll_rate=-0.12,
                pitch_rate=25.34, yaw_rate=-30.27, laser_range=1234.5, self_check=128)


def encoded(packet):
    return json.dumps(packet, separators=(',', ':')) if isinstance(packet, dict) else packet


class ZhongkeTelemetryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix='px4-zhongke-json-')
        cls.addClassCleanup(cls.temp.cleanup)
        directory = Path(cls.temp.name)
        header = directory / 'uORB' / 'topics' / 'dyt_zhongke_telemetry.h'
        header.parent.mkdir(parents=True)
        types = dict(uint8='uint8_t', uint64='uint64_t', float32='float')
        members = [f'{types[kind]} {name} {{}};' for kind, name in schema()]
        cls.kinds = {name: kind for kind, name in schema()}
        header.write_text('#pragma once\n#include <cstdint>\nstruct dyt_zhongke_telemetry_s {\n'
                          + '\n'.join(members) + '\n};\n')
        source = (ROOT / 'ZhongkeUdpBridge.cpp').read_text()
        start = source.index('\tbool open_socket()')
        production = source[start:source.index('\n\tuint16_t _port', start)]
        outputs = '\n'.join(f" << ' ' << +msg.{name}" for _, name in schema())
        harness = r'''
#include "ZhongkeJsonParser.hpp"
#include <arpa/inet.h>
#include <cerrno>
#include <cstdio>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <netinet/in.h>
#include <poll.h>
#include <string>
#include <sys/socket.h>
#include <unistd.h>
#define PX4_ERR(...) do { fprintf(stderr, __VA_ARGS__); fputc('\n', stderr); } while (0)
#define PX4_INFO(...) do { fprintf(stderr, __VA_ARGS__); fputc('\n', stderr); } while (0)
using zhongke_udp::kUdpBufferSize;
constexpr uint16_t kDefaultPort = 9000;
uint64_t hrt_absolute_time() { return 424242; }
struct CapturePublisher {
    dyt_zhongke_telemetry_s message{};
    bool allow{true};
    unsigned attempts{0};
    bool publish(const dyt_zhongke_telemetry_s &value) {
        ++attempts;
        if (!allow) { return false; }
        message = value;
        return true;
    }
};
class ProductionBridge {
public:
    bool should_exit() const { return false; }
PRODUCTION_SOURCE
    uint16_t _port{0};
    int _fd{-1};
    uint64_t _last_rx{0};
    char _last_peer[INET_ADDRSTRLEN] {};
    uint16_t _last_peer_port{0};
    uint32_t _datagrams{0}, _valid{0}, _parse_errors{0}, _oversize{0}, _publications{0};
    CapturePublisher _publisher{};
};
sockaddr_in endpoint(const ProductionBridge &bridge) {
    sockaddr_in address{};
    socklen_t size = sizeof(address);
    if (getsockname(bridge._fd, reinterpret_cast<sockaddr *>(&address), &size) != 0) { std::exit(2); }
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    return address;
}
int main(int argc, char **argv) {
    const std::string mode = argc > 1 ? argv[1] : "parser";
    if (mode == "exclusive") {
        ProductionBridge first, second;
        if (!first.open_socket()) { return 2; }
        second._port = ntohs(endpoint(first).sin_port);
        const bool duplicate = second.open_socket();
        std::cout << duplicate << '\n';
        first.close_socket();
        const bool released = second.open_socket();
        std::cout << released << '\n';
        second.close_socket();
        return 0;
    }
    std::string input;
    while (std::getline(std::cin, input)) {
        dyt_zhongke_telemetry_s parsed{};
        ProductionBridge bridge;
        bool ok = false;
        uint64_t input_timestamp = 0;
        if (mode == "udp") {
            if (!bridge.open_socket()) { return 2; }
            const int sender = socket(AF_INET, SOCK_DGRAM, 0);
            if (sender < 0) { return 2; }
            const auto address = endpoint(bridge);
            if (sendto(sender, input.data(), input.size(), 0,
                       reinterpret_cast<const sockaddr *>(&address), sizeof(address)) < 0) { return 2; }
            pollfd fdset{bridge._fd, POLLIN, 0};
            if (poll(&fdset, 1, 1000) != 1) { return 2; }
            bridge.receive();
            ok = bridge._valid > 0;
            close(sender);
            bridge.close_socket();
        } else {
            zhongke_udp::JsonParser parser(input.data(), input.size());
            ok = parser.parse(parsed);
            input_timestamp = parsed.timestamp;
            bridge._publisher.allow = mode != "publish_failure";
            if (ok) { bridge.publish(parsed); }
        }
        const auto &msg = bridge._publisher.message;
        std::cout << ok << ' ' << bridge._publications << ' ' << bridge._publisher.attempts
                  << ' ' << bridge._last_rx << ' ' << bridge._parse_errors << ' ' << bridge._oversize
                  << ' ' << input_timestamp << std::setprecision(17)
OUTPUT_SOURCE << '\n';
    }
}
'''.replace('PRODUCTION_SOURCE', production).replace('OUTPUT_SOURCE', outputs)
        cpp = directory / 'bridge_test.cpp'
        cpp.write_text(harness)
        cls.executable = directory / 'bridge_test'
        subprocess.run(['g++', '-std=c++14', '-O2', '-Wall', '-Wextra', '-Werror',
                        '-I', str(directory), '-I', str(ROOT), str(cpp), '-o', str(cls.executable)],
                       check=True, capture_output=True, text=True)

    def parse(self, payload, mode='parser'):
        output = subprocess.run([str(self.executable), mode], input=encoded(payload) + '\n',
                                check=True, capture_output=True, text=True).stdout.strip().split()
        self.assertEqual(len(output), len(self.kinds) + 7)
        (valid, self.publications, self.attempts, self.last_rx, self.parse_errors,
         self.oversize, self.input_timestamp) = map(int, output[:7])
        self.message = {name: float(value) if kind == 'float32' else int(value)
                        for (name, kind), value in zip(self.kinds.items(), output[7:])}
        return bool(valid)

    def assertRejected(self, payload, mode='parser'):
        self.assertFalse(self.parse(payload, mode), payload)
        self.assertEqual(self.publications, 0)
        self.assertEqual(self.attempts, 0)

    def test_all_measurements_keep_their_physical_units_and_status_bytes(self):
        packet = full_packet()
        self.assertEqual(set(packet), set(self.kinds))
        self.assertTrue(self.parse(packet))
        self.assertEqual(self.publications, 1)
        for name, expected in packet.items():
            if name == 'timestamp':
                continue
            if self.kinds[name] == 'float32':
                expected = struct.unpack('f', struct.pack('f', expected))[0]
            self.assertEqual(self.message[name], expected, name)
        self.assertEqual(self.input_timestamp, packet['timestamp'])
        self.assertEqual(self.message['timestamp'], 424242)
        self.assertEqual(self.last_rx, 424242)

    def test_zero_and_uint64_max_timestamp_are_valid_but_replaced_on_publish(self):
        for timestamp in (0, 2**64 - 1):
            with self.subTest(timestamp=timestamp):
                packet = full_packet()
                packet['timestamp'] = timestamp
                self.assertTrue(self.parse(packet))
                self.assertEqual(self.input_timestamp, timestamp)
                self.assertEqual(self.message['timestamp'], 424242)

    def test_each_of_the_15_fields_is_required(self):
        for name in full_packet():
            with self.subTest(field=name):
                packet = full_packet()
                del packet[name]
                self.assertRejected(packet)

    def test_raw_status_fields_accept_full_byte_range(self):
        for value in (0, 255):
            packet = full_packet()
            packet.update({name: value for name in BYTE_FIELDS})
            self.assertTrue(self.parse(packet))
            for name in BYTE_FIELDS:
                self.assertEqual(self.message[name], value)

    def test_status_fields_reject_noninteger_tokens_and_out_of_range_values(self):
        for name in BYTE_FIELDS:
            for bad in (-1, 256, 1.0, True, None, '1', [], {}):
                with self.subTest(field=name, value=bad):
                    packet = full_packet()
                    packet[name] = bad
                    self.assertRejected(packet)

    def test_timestamp_requires_exact_unsigned_integer_literal(self):
        packet = encoded(full_packet())
        for token in ('-1', '-0', '+1', '01', '1.0', '1e0', '18446744073709551616',
                      'true', 'null', '"123"', '[]', '{}'):
            with self.subTest(token=token):
                self.assertRejected(packet.replace('1793606400123', token))

    def test_integer_statuses_are_not_rounded_or_converted_from_exponents(self):
        packet = encoded(full_packet())
        for token in ('01', '+1', '1e0', '1.0', '1.000000000000000000000001', '1e-1000'):
            with self.subTest(token=token):
                self.assertRejected(packet.replace('"status1":132', f'"status1":{token}'))

    def test_all_float_fields_reject_invalid_and_nonfinite_measurements(self):
        for name, kind in self.kinds.items():
            if kind != 'float32':
                continue
            for bad in (None, True, '10', [], {}, float('nan'), float('inf'), -float('inf'), 1e39):
                with self.subTest(field=name, value=bad):
                    packet = full_packet()
                    packet[name] = bad
                    self.assertRejected(packet)

    def test_float32_boundaries_and_exponents_are_preserved(self):
        maximum = float.fromhex('0x1.fffffep+127')
        packet = full_packet()
        packet.update(frame_roll=-maximum, frame_pitch=maximum, pitch_rate=-1.5e-4,
                      yaw_rate=1.25e3, laser_range=0)
        self.assertTrue(self.parse(packet))
        self.assertEqual(self.message['frame_roll'], -maximum)
        self.assertEqual(self.message['frame_pitch'], maximum)
        self.assertAlmostEqual(self.message['pitch_rate'], -1.5e-4, places=10)
        self.assertEqual(self.message['yaw_rate'], 1250)
        self.assertEqual(self.message['laser_range'], 0)

    def test_duplicate_known_unknown_and_escaped_keys_are_rejected(self):
        packet = encoded(full_packet())
        for key in ('status1', 'sta\\u0074us1'):
            self.assertRejected(packet[:-1] + f',"{key}":1' + '}')
        self.assertRejected(packet[:-1] + ',"extra":1,"ex\\u0074ra":2}')
        self.assertRejected(packet[:-1] + ',"\\u6d4b\\u8bd5":1,"测试":2}')

    def test_unknown_extensions_accept_structures_unicode_and_embedded_nul(self):
        packet = full_packet()
        packet.update(extra={'flags': [True, False, None], 'text': '测试 😀', 'value': 1e-6},
                      **{'status1\0other': '扩展字段'})
        self.assertTrue(self.parse(packet))
        self.assertEqual(self.message['status1'], 132)
        self.assertTrue(self.parse(encoded(packet).replace('"status1":', '"sta\\u0074us1":')))

    def test_unknown_values_have_bounded_nesting(self):
        packet = full_packet()
        nested = 1
        for _ in range(8):
            nested = [nested]
        packet['extra'] = nested
        self.assertTrue(self.parse(packet))
        packet['extra'] = [nested]
        self.assertRejected(packet)
        packet['extra'] = {'value': nested}
        self.assertRejected(packet)

    def test_unterminated_input_and_long_number_tokens_are_bounded(self):
        packet = encoded(full_packet())
        for length in (1, 5, 20, 80, 150, len(packet) - 1):
            with self.subTest(length=length):
                self.assertRejected(packet[:length])
        self.assertRejected(packet.replace('"zoom":51.3', '"zoom":0.' + '1' * 100))

    def test_invalid_raw_utf8_is_rejected_without_dropping_valid_unicode(self):
        packet = encoded(full_packet())[:-1].encode()
        for bad in (b'\xc0\x80', b'\xe0\x80\x80', b'\xed\xa0\x80', b'\xf4\x90\x80\x80', b'\xff'):
            with self.subTest(value=bad):
                result = subprocess.run([str(self.executable)], input=packet + b',"extra":"' + bad + b'"}\n',
                                        check=True, capture_output=True)
                self.assertEqual(result.stdout.split()[:3], [b'0', b'0', b'0'])

    def test_top_level_field_count_and_decoded_key_length_are_bounded(self):
        packet = full_packet()
        packet.update({f'extension_{index}': index for index in range(17)})
        self.assertTrue(self.parse(packet))
        packet['extension_17'] = 17
        self.assertRejected(packet)
        packet = full_packet()
        packet['x' * 63] = None
        self.assertTrue(self.parse(packet))
        del packet['x' * 63]
        packet['x' * 64] = None
        self.assertRejected(packet)

    def test_invalid_json_strings_numbers_and_trailing_bytes_are_rejected(self):
        packet = encoded(full_packet())
        for bad in ('{}', '', packet[:-1], packet + 'x', packet + '{}', packet[:-1] + ',}',
                    packet.replace('"zoom":51.3', '"zoom":1e'),
                    packet.replace('"zoom":51.3', '"zoom":1.'),
                    packet[:-1] + ',"extra":"\\uD800"}',
                    packet[:-1] + ',"extra":"\\uDC00"}',
                    packet[:-1] + ',"extra":"\\q"}',
                    packet + '\0'):
            with self.subTest(payload=bad[-50:]):
                self.assertRejected(bad)

    def test_both_aisheng_json_forms_are_rejected(self):
        two_axis = dict(timestamp=1793606400123, track_status=1, pitch_angle=-2.5,
                        yaw_angle=3.5, pitch_misalignment=0.25, yaw_misalignment=-0.5,
                        tracking_confidence=25.5, target_confidence=0.8,
                        gyro_pitch_rate=1.25, gyro_yaw_rate=-2.5, pitch_los_rate=3.25,
                        yaw_los_rate=-4.5, los_omega_n=1, los_omega_e=2, los_omega_d=3,
                        tracking_gate_w=40, tracking_gate_h=50, gate_x=320, gate_y=240)
        self.assertRejected(two_axis)
        aisheng_msg = MSG.with_name('DytAishengTelemetry.msg')
        aisheng = {}
        for line in aisheng_msg.read_text().splitlines():
            field = line.split('#', 1)[0].split()
            if not field:
                continue
            kind, name = field
            if name.startswith('los_omega_'):
                continue
            aisheng[name] = [0] * 4 if '[' in kind else 0
        self.assertEqual(len(aisheng), 37)
        self.assertRejected(aisheng)

    def test_actual_udp_receive_publishes_complete_measurements(self):
        self.assertTrue(self.parse(full_packet(), 'udp'))
        self.assertEqual(self.publications, 1)
        self.assertEqual(self.last_rx, 424242)
        self.assertEqual(self.message['timestamp'], 424242)
        self.assertEqual(self.message['status3'], 242)
        self.assertEqual(self.parse_errors, 0)
        self.assertEqual(self.oversize, 0)

    def test_actual_udp_receive_rejects_empty_and_truncated_json(self):
        for payload in ('', encoded(full_packet())[:-1]):
            with self.subTest(payload=payload):
                self.assertRejected(payload, 'udp')
                self.assertEqual(self.parse_errors, 1)
                self.assertEqual(self.oversize, 0)

    def test_actual_udp_receive_rejects_complete_prefix_in_oversize_datagram(self):
        payload = encoded(full_packet())
        allowed = payload + ' ' * (4095 - len(payload))
        self.assertTrue(self.parse(allowed, 'udp'))
        for size in (4096, 5000):
            with self.subTest(size=size):
                oversized = payload + ' ' * (size - len(payload))
                self.assertRejected(oversized)
                self.assertRejected(oversized, 'udp')
                self.assertEqual(self.parse_errors, 0)
                self.assertEqual(self.oversize, 1)

    def test_udp_port_is_exclusive_and_is_released_on_close(self):
        result = subprocess.run([str(self.executable), 'exclusive'], check=True,
                                capture_output=True, text=True)
        self.assertEqual(result.stdout.split(), ['0', '1'])

    def test_failed_uorb_publication_does_not_advance_success_state(self):
        self.assertTrue(self.parse(full_packet(), 'publish_failure'))
        self.assertEqual(self.attempts, 1)
        self.assertEqual(self.publications, 0)
        self.assertEqual(self.last_rx, 0)


if __name__ == '__main__':
    unittest.main()
