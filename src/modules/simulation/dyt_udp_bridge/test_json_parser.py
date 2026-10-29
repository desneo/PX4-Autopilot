#!/usr/bin/env python3
"""编译生产JSON解析器与发布函数，核对兼容、波门尺寸量化和实际uORB映射。"""
import itertools
import json
import math
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parent
OMEGA = ('los_omega_n', 'los_omega_e', 'los_omega_d')


def old_fields():
    return dict(track_status=1, pitch_angle=-79.123456, yaw_angle=-59.123456,
                pitch_misalignment=-45.123456, yaw_misalignment=-45.123456,
                tracking_confidence=25.5, target_confidence=.8,
                pitch_los_rate=-20., yaw_los_rate=-20.)


def fields13():
    return dict(old_fields(), gyro_pitch_rate=-3., gyro_yaw_rate=7.,
                tracking_gate_w=1920.123456, tracking_gate_h=1080.123456)


def fields16():
    return dict(fields13(), los_omega_n=40., los_omega_e=-30., los_omega_d=25.)


def fields18():
    return dict(fields16(), gate_x=960, gate_y=540)


class ProductionJsonParserTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix='px4-dyt-json-')
        cls.addClassCleanup(cls.temp.cleanup)
        directory = Path(cls.temp.name)
        source = (ROOT/'DytUdpBridge.cpp').read_text()
        parser = source[source.index('namespace\n{'):source.index('\nclass DytUdpBridge')]
        publish_start = source.index('\tvoid publish(const AishengFields &fields)')
        publish_end = source.index('\n\tuint16_t _port', publish_start)
        publish = source[publish_start:publish_end]
        scalar_types = {'uint8': 'uint8_t', 'uint16': 'uint16_t', 'uint32': 'uint32_t',
                        'uint64': 'uint64_t', 'int16': 'int16_t', 'int32': 'int32_t',
                        'float32': 'float'}
        members = []
        for line in (ROOT.parents[3]/'msg/DytAishengTelemetry.msg').read_text().splitlines():
            fields = line.split('#', 1)[0].split()
            if not fields:
                continue
            kind, name = fields
            if '[' in kind:
                kind, count = kind.rstrip(']').split('[')
                members.append(f'{scalar_types[kind]} {name}[{count}] {{}};')
            else:
                members.append(f'{scalar_types[kind]} {name} {{}};')
        message = 'struct dyt_aisheng_telemetry_s {\n'+'\n'.join(members)+'\n};'
        harness = r"""
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <iomanip>
#include <string>
#define PX4_ERR(...) fprintf(stderr, __VA_ARGS__)
PARSER_SOURCE
MESSAGE_SOURCE
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
    (void)kDefaultPort;
    std::string line;
    while (std::getline(std::cin, line)) {
        AishengFields fields{};
        JsonParser parser(line.data(), line.size());
        const bool ok = line.size() < kUdpBufferSize && parser.parse(fields);
        ProductionPublisher bridge;
        if (ok) { bridge.publish(fields); }
        // 核对实际publish生成的uORB值，不能用解析器中间字段代替。
        const auto &msg = bridge._publisher.message;
        std::cout << ok << ' ' << std::setprecision(9)
                  << msg.los_omega_n << ' ' << msg.los_omega_e << ' '
                  << msg.los_omega_d << ' ' << msg.gyro_pitch_rate << ' '
                  << msg.gyro_yaw_rate << ' ' << msg.pitch_los_rate << ' '
                  << msg.yaw_los_rate << ' ' << msg.gate_x << ' ' << msg.gate_y << ' '
                  << msg.tracking_gate_w << ' ' << msg.tracking_gate_h << ' '
                  << bridge._publications << ' ' << msg.timestamp << ' '
                  << unsigned(msg.self_check) << ' ' << unsigned(msg.work_status) << ' '
                  << unsigned(msg.track_status) << ' ' << msg.pitch_angle << ' '
                  << msg.yaw_angle << ' ' << msg.pitch_misalignment << ' '
                  << msg.yaw_misalignment << ' ' << msg.tracking_confidence << ' '
                  << msg.target_confidence[0] << ' ' << msg.target_confidence[1] << '\n';
    }
}
""".replace('PARSER_SOURCE', parser).replace('MESSAGE_SOURCE', message).replace('PUBLISH_SOURCE', publish)
        cpp = directory/'parser_test.cpp';cpp.write_text(harness)
        cls.executable = directory/'parser_test'
        subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror',str(cpp),'-o',str(cls.executable)],
                       check=True,capture_output=True,text=True)

    def parse(self, payload):
        if isinstance(payload, dict):payload=json.dumps(payload,separators=(',',':'))
        output=subprocess.run([str(self.executable)],input=payload+'\n',check=True,
                              capture_output=True,text=True).stdout.strip().split()
        self.assertEqual(len(output),24,output)
        self.last_gates=tuple(map(int,output[8:10]))
        self.last_dimensions=tuple(map(int,output[10:12]))
        self.publications=int(output[12])
        self.message=dict(zip(('timestamp','self_check','work_status','track_status','pitch_angle',
                               'yaw_angle','pitch_misalignment','yaw_misalignment',
                               'tracking_confidence','target_confidence_0','target_confidence_1'),
                              map(float,output[13:])))
        return bool(int(output[0])),list(map(float,output[1:8]))

    def assertRejected(self, payload):
        self.assertFalse(self.parse(payload)[0],payload)
        self.assertEqual(self.publications,0)

    def test_old_nine_fields_are_accepted_without_fabricating_vector_zero(self):
        ok,values=self.parse(old_fields());self.assertTrue(ok)
        self.assertTrue(all(math.isnan(x) for x in values[:3]))
        self.assertEqual(values[3:5],[0.,0.])

    def test_existing_thirteen_fields_publish_gyro_and_gate_dimensions(self):
        ok,values=self.parse(fields13());self.assertTrue(ok)
        self.assertTrue(all(math.isnan(x) for x in values[:3]))
        self.assertEqual(values[3:],[ -3.,7.,-20.,-20.])
        self.assertEqual(self.last_dimensions,(1920,1080))

    def test_sixteen_fields_publish_raw_vector_without_clipping_or_scaling(self):
        ok,values=self.parse(fields16());self.assertTrue(ok)
        self.assertEqual(values,[40.,-30.,25.,-3.,7.,-20.,-20.])

    def test_undefined_scalar_angles_do_not_discard_valid_vector(self):
        packet=dict(fields16(),pitch_los_rate=0.,yaw_los_rate=0.)
        ok,values=self.parse(packet);self.assertTrue(ok)
        self.assertEqual(values[:3],[40.,-30.,25.]);self.assertEqual(values[-2:],[0.,0.])

    def test_explicit_invalid_vector_zero_is_distinct_from_absent_fields(self):
        packet=fields16();packet.update(dict.fromkeys(OMEGA,0.))
        ok,values=self.parse(packet);self.assertTrue(ok);self.assertEqual(values[:3],[0.,0.,0.])

    def test_incomplete_vector_group_is_rejected(self):
        for size in (1,2):
            for keys in itertools.combinations(OMEGA,size):
                with self.subTest(keys=keys):self.assertRejected(dict(fields13(),**dict.fromkeys(keys,1.)))

    def test_duplicate_vector_components_are_rejected(self):
        raw=json.dumps(fields16(),separators=(',',':'))
        for key in OMEGA:
            with self.subTest(key=key):self.assertRejected(raw[:-1]+','+json.dumps(key)+':2}')

    def test_duplicate_existing_optional_gyro_is_rejected(self):
        raw=json.dumps(fields13(),separators=(',',':'))
        self.assertRejected(raw[:-1]+',"gyro_yaw_rate":2}')

    def test_vector_rejects_null_bool_string_nonfinite_and_overflow(self):
        raw=json.dumps(fields16(),separators=(',',':'))
        for literal in ('null','true','false','"12"','NaN','Infinity','-Infinity','1e500','3.5e38'):
            with self.subTest(value=literal):self.assertRejected(raw.replace('"los_omega_n":40.0','"los_omega_n":'+literal))

    def test_finite_float32_near_limit_is_accepted_without_truncation(self):
        packet=fields16();packet.update(los_omega_n=3.4028234e38,los_omega_e=-3.4028234e38,los_omega_d=0.)
        ok,values=self.parse(packet);self.assertTrue(ok)
        self.assertGreater(values[0],3e38);self.assertLess(values[1],-3e38)

    def test_legal_fixed_decimal_packet_above_512_bytes_is_accepted(self):
        packet=dict(fields16(),gyro_pitch_rate=-20.,gyro_yaw_rate=-20.,
                    los_omega_n=-3.4028234e38,los_omega_e=3.4028234e38,los_omega_d=-3.4028234e38)
        raw='{'+','.join(json.dumps(key)+':'+format(value,'.6f') for key,value in packet.items())+'}'
        self.assertGreater(len(raw),512);self.assertLess(len(raw),2048)
        ok,values=self.parse(raw);self.assertTrue(ok);self.assertGreater(abs(values[0]),3e38)

    def test_oversize_packet_remains_rejected(self):
        raw=json.dumps(dict(fields16(),padding='x'*2200),separators=(',',':'))
        self.assertRejected(raw)

    def test_missing_required_field_and_malformed_json_stay_rejected(self):
        packet=fields16();packet.pop('yaw_los_rate');self.assertRejected(packet)
        for raw in ('{}','[]',json.dumps(fields16())+' trailing','{"track_status":1,}'):
            with self.subTest(raw=raw):self.assertRejected(raw)

    def test_legacy_9_13_16_fields_default_gate_center_to_zero(self):
        for fields in (old_fields,fields13,fields16):
            with self.subTest(fields=fields.__name__):
                self.assertTrue(self.parse(fields())[0]);self.assertEqual(self.last_gates,(0,0))

    def test_eighteen_fields_keep_integer_gate_center_and_high_mask_bit(self):
        self.assertTrue(self.parse(fields18())[0]);self.assertEqual(self.last_gates,(960,540))
        for x,y in ((0,0),(32767,32767),(0,32767),(32767,0)):
            with self.subTest(x=x,y=y):
                self.assertTrue(self.parse(dict(fields18(),gate_x=x,gate_y=y))[0])
                self.assertEqual(self.last_gates,(x,y))
        self.assertTrue(self.parse(dict(old_fields(),gate_x=1,gate_y=2))[0])
        self.assertEqual(self.last_gates,(1,2))

    def test_eighteen_field_inactive_tracking_zero_center_is_accepted(self):
        self.assertTrue(self.parse(dict(fields18(),track_status=0,gate_x=0,gate_y=0))[0])
        self.assertEqual(self.last_gates,(0,0))

    def test_gate_partial_group_is_rejected(self):
        for key in ('gate_x','gate_y'):
            with self.subTest(key=key):self.assertRejected(dict(fields16(),**{key:1}))

    def test_gate_duplicates_at_mask_bits_14_and_15_are_rejected(self):
        raw=json.dumps(fields18(),separators=(',',':'))
        for key in ('gate_x','gate_y'):
            with self.subTest(key=key):self.assertRejected(raw[:-1]+','+json.dumps(key)+':2}')

    def test_gate_rejects_negative_fractional_out_of_range_and_noninteger_tokens(self):
        raw=json.dumps(fields18(),separators=(',',':'))
        for key,old in (('gate_x','960'),('gate_y','540')):
            for value in ('-1','-0','32768','999999999999999999999999999999','1.5','1.000000001',
                          '32767.000001','1.0','1e0','1e50','null','true','false','"2"','[]','NaN','Infinity','01','+1'):
                with self.subTest(key=key,value=value):
                    self.assertRejected(raw.replace('"'+key+'":'+old,'"'+key+'":'+value))

    def test_nineteen_fields_ignore_unix_millisecond_timestamp_without_changing_measurements(self):
        expected_ok,expected_values=self.parse(fields18())
        expected_gates=self.last_gates
        expected_dimensions=self.last_dimensions
        self.assertTrue(expected_ok)
        # Unix毫秒数远超float32精确整数范围；未知扩展字段不能影响测量发布。
        packet=dict(fields18(),timestamp=1793012345678)
        self.assertEqual(len(packet),19)
        ok,values=self.parse(packet)
        self.assertTrue(ok);self.assertEqual(values,expected_values)
        self.assertEqual(self.last_gates,expected_gates)
        self.assertEqual(self.last_dimensions,expected_dimensions)

    def test_unknown_nested_extension_is_still_compatible(self):
        packet=dict(fields16(),extension={'a':[1,True,None,{'b':'x'}]})
        self.assertTrue(self.parse(packet)[0])


    def test_production_publish_maps_existing_measurements_and_hrt_timestamp(self):
        packet=dict(fields18(),timestamp=1793012345678)
        self.assertTrue(self.parse(packet)[0]);self.assertEqual(self.publications,1)
        self.assertEqual(self.message['timestamp'],424242)
        self.assertEqual(self.message['self_check'],1);self.assertEqual(self.message['work_status'],1)
        for key in ('track_status','pitch_angle','yaw_angle','pitch_misalignment','yaw_misalignment',
                    'tracking_confidence'):
            self.assertAlmostEqual(self.message[key],packet[key],places=5)
        self.assertAlmostEqual(self.message['target_confidence_0'],packet['target_confidence'],places=6)
        self.assertEqual(self.message['target_confidence_1'],0)


    def test_dimension_half_up_rounding_is_visible_in_production_publish(self):
        for width,height,expected in ((0,0,(0,0)),(0.49,0.5,(0,1)),(10.5,20.499,(11,20)),
                                      (1920.123456,1080.75,(1920,1081)),
                                      (65534.5,65534.9,(65535,65535)),(65535,65535.,(65535,65535))):
            with self.subTest(width=width,height=height):
                self.assertTrue(self.parse(dict(fields18(),tracking_gate_w=width,tracking_gate_h=height))[0])
                self.assertEqual(self.last_dimensions,expected)
                self.assertEqual(self.publications,1)

    def test_dimension_numeric_exponents_are_accepted_without_relaxing_center_literals(self):
        raw=json.dumps(fields18(),separators=(',',':'))
        raw=raw.replace('"tracking_gate_w":1920.123456','"tracking_gate_w":1.235e2')
        raw=raw.replace('"tracking_gate_h":1080.123456','"tracking_gate_h":5e-1')
        self.assertTrue(self.parse(raw)[0]);self.assertEqual(self.last_dimensions,(124,1))
        self.assertEqual(self.last_gates,(960,540))

    def test_dimensions_keep_decimal_precision_at_rounding_and_range_boundaries(self):
        packet=dict(fields18(),tracking_gate_w=0.499999999,tracking_gate_h=65534.499999)
        self.assertTrue(self.parse(packet)[0]);self.assertEqual(self.last_dimensions,(0,65534))
        for key in ('tracking_gate_w','tracking_gate_h'):
            for value in (65535.000001,-1e-50):
                with self.subTest(key=key,value=value):
                    self.assertRejected(dict(fields18(),**{key:value}))

    def test_dimensions_remain_independently_optional_for_legacy_packets(self):
        for fields in (old_fields,fields13,fields16,fields18):
            base=fields();base.pop('tracking_gate_w',None);base.pop('tracking_gate_h',None)
            for extension,expected in (({},(0,0)),({'tracking_gate_w':12.5},(13,0)),
                                       ({'tracking_gate_h':24.5},(0,25))):
                with self.subTest(fields=fields.__name__,extension=extension):
                    self.assertTrue(self.parse(dict(base,**extension))[0])
                    self.assertEqual(self.last_dimensions,expected)
                    self.assertEqual(self.publications,1)

    def test_duplicate_dimensions_at_mask_bits_sixteen_and_seventeen_are_rejected(self):
        for fields in (fields13,fields18):
            raw=json.dumps(fields(),separators=(',',':'))
            for key in ('tracking_gate_w','tracking_gate_h'):
                with self.subTest(fields=fields.__name__,key=key):
                    self.assertRejected(raw[:-1]+','+json.dumps(key)+':2}')

    def test_dimensions_and_high_gate_center_bits_are_order_independent(self):
        packet=fields18()
        for items in (list(packet.items()),list(reversed(list(packet.items())))):
            self.assertTrue(self.parse(dict(items))[0])
            self.assertEqual(self.last_dimensions,(1920,1080))
            self.assertEqual(self.last_gates,(960,540))

    def test_dimensions_reject_non_numeric_nonfinite_negative_and_out_of_range_values(self):
        raw=json.dumps(fields18(),separators=(',',':'))
        for key,old in (('tracking_gate_w','1920.123456'),('tracking_gate_h','1080.123456')):
            for literal in ('-1','-0.1','65535.1','65536','1e10','1e500','NaN','Infinity','-Infinity',
                            'null','true','false','"12"','[]','{}','01','+1'):
                with self.subTest(key=key,value=literal):
                    self.assertRejected(raw.replace('"'+key+'":'+old,'"'+key+'":'+literal))

    def test_real_nineteen_field_packet_publishes_dimensions_and_center_together(self):
        packet=dict(fields18(),tracking_gate_w=50.5,tracking_gate_h=40.49,timestamp=1793012345678)
        self.assertEqual(len(packet),19)
        self.assertTrue(self.parse(packet)[0]);self.assertEqual(self.publications,1)
        self.assertEqual(self.last_dimensions,(51,40));self.assertEqual(self.last_gates,(960,540))
        self.assertEqual(self.message['timestamp'],424242)


if __name__=='__main__':unittest.main()
