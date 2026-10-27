#!/usr/bin/env python3
"""直接编译生产JSON解析器，核对旧包兼容、三维LOS、整数波门坐标与发送时间戳扩展。"""
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
        harness = r"""
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <string>
PARSER_SOURCE
int main() {
    (void)kDefaultPort;
    std::string line;
    while (std::getline(std::cin, line)) {
        AishengFields fields{};
        JsonParser parser(line.data(), line.size());
        // 使用与生产UDP接收相同的有界报文门禁；不接受被截断的数据。
        const bool ok = line.size() < kUdpBufferSize && parser.parse(fields);
        std::cout << ok << ' ' << std::setprecision(9)
                  << fields.los_omega_n << ' ' << fields.los_omega_e << ' '
                  << fields.los_omega_d << ' ' << fields.gyro_pitch_rate << ' '
                  << fields.gyro_yaw_rate << ' ' << fields.pitch_los_rate << ' '
                  << fields.yaw_los_rate << ' ' << fields.gate_x << ' '
                  << fields.gate_y << '\n';
    }
}
""".replace('PARSER_SOURCE', parser)
        cpp = directory/'parser_test.cpp';cpp.write_text(harness)
        cls.executable = directory/'parser_test'
        subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror',str(cpp),'-o',str(cls.executable)],
                       check=True,capture_output=True,text=True)

    def parse(self, payload):
        if isinstance(payload, dict):payload=json.dumps(payload,separators=(',',':'))
        output=subprocess.run([str(self.executable)],input=payload+'\n',check=True,
                              capture_output=True,text=True).stdout.strip().split()
        self.assertEqual(len(output),10,output)
        self.last_gates=tuple(map(int,output[8:]))
        return bool(int(output[0])),list(map(float,output[1:8]))

    def assertRejected(self, payload):
        self.assertFalse(self.parse(payload)[0],payload)

    def test_old_nine_fields_are_accepted_without_fabricating_vector_zero(self):
        ok,values=self.parse(old_fields());self.assertTrue(ok)
        self.assertTrue(all(math.isnan(x) for x in values[:3]))
        self.assertEqual(values[3:5],[0.,0.])

    def test_existing_thirteen_fields_keep_gyro_and_unknown_gates(self):
        ok,values=self.parse(fields13());self.assertTrue(ok)
        self.assertTrue(all(math.isnan(x) for x in values[:3]))
        self.assertEqual(values[3:],[ -3.,7.,-20.,-20.])

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
        self.assertTrue(expected_ok)
        # Unix毫秒数远超float32精确整数范围；未知扩展字段不能影响测量发布。
        packet=dict(fields18(),timestamp=1793012345678)
        self.assertEqual(len(packet),19)
        ok,values=self.parse(packet)
        self.assertTrue(ok);self.assertEqual(values,expected_values)
        self.assertEqual(self.last_gates,expected_gates)

    def test_unknown_nested_extension_is_still_compatible(self):
        packet=dict(fields16(),extension={'a':[1,True,None,{'b':'x'}]})
        self.assertTrue(self.parse(packet)[0])


if __name__=='__main__':unittest.main()
