#!/usr/bin/env python3
"""离线验证场景02的独立光学/机械轴/球心断言，不连接或控制仿真。"""
import math
from pathlib import Path
import tempfile
import unittest

import test_gimbal_interfaces as check


def pose(position=(0., 0., 0.), quaternion=(1., 0., 0., 0.)):
    return dict(position=position, quaternion=quaternion)


def sample(yaw=0., pitch=0., model=None):
    model = model or pose((5., 8., .21), check.unit((.9, .1, -.2, .3)))
    mount = pose((0., 0., .289))
    camera = pose((.015, 0., .345), check.joint_rotation(yaw, pitch))
    sensor = pose((0., 0., .0237), check.sensor_rotation())
    optical = check.compose(check.compose(model, camera), sensor)
    expected = check.qmul(check.joint_rotation(yaw, pitch), check.sensor_rotation())
    telemetry = dict(protocol_version=6, angle_convention='motion_head_joint',
                     mount=dict(static=False, quat_world=model['quaternion']),
                     joints_deg=dict(yaw=dict(pos=yaw), pitch=dict(pos=pitch)),
                     camera=dict(quat_world=optical['quaternion']),
                     los_mount=check.rotate(expected, (1., 0., 0.)))
    return telemetry, model, mount, camera, sensor


class OpticalValidationTest(unittest.TestCase):
    def assertVector(self, actual, expected):
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, places=8)

    def test_zero_optical_basis(self):
        result = check.optical_check(*sample())
        self.assertVector(result['actual_los_mount'], (0., 0., 1.))
        self.assertVector(result['actual_image_up_mount'], (-1., 0., 0.))
        self.assertVector(result['actual_lens_center_mount_m'], (.015, 0., .0797))

    def test_new_negative_x_y_axes_and_both_aircraft_parents(self):
        for x in (0., 5.):
            model = pose((x, 7., .21), check.unit((.9, .1, -.2, .3)))
            for yaw in (-60., 0., 60.):
                for pitch in (-80., 0., 30.):
                    result = check.optical_check(*sample(yaw, pitch, model))
                    y, p = math.radians(yaw), math.radians(pitch)
                    los = (-math.sin(p), math.sin(y)*math.cos(p), math.cos(y)*math.cos(p))
                    self.assertVector(result['actual_los_mount'], los)
                    self.assertVector(result['actual_ball_center_mount_m'], (.015, 0., .056))
                    self.assertVector(result['actual_lens_center_mount_m'], check.plus(
                        (.015, 0., .056), tuple(.0237*v for v in los)))

    def test_legacy_vertical_outer_axis_is_rejected(self):
        values = list(sample(45., -30.))
        y, p = -math.radians(45.)/2, math.radians(30.)/2
        values[3]['quaternion'] = check.qmul((math.cos(y), 0., 0., math.sin(y)),
                                            (math.cos(p), 0., math.sin(p), 0.))
        with self.assertRaisesRegex(ValueError, '轴/光学/球心'):
            check.optical_check(*values)

    def test_legacy_orbiting_sphere_center_is_rejected(self):
        values = list(sample(45., 0.))
        values[3]['position'] = (.015*math.sqrt(.5), .015*math.sqrt(.5), .345)
        with self.assertRaisesRegex(ValueError, '球心'):
            check.optical_check(*values)

    def test_wrong_lens_offset_or_image_up_are_rejected(self):
        for error in ('offset', 'rotation'):
            values = list(sample())
            if error == 'offset':
                values[4]['position'] = (0., 0., .026)
            else:
                values[4]['quaternion'] = check.qmul(check.sensor_rotation(), (0., 1., 0., 0.))
            with self.assertRaises(ValueError):
                check.optical_check(*values)

    def test_old_protocol_and_static_mount_are_rejected(self):
        for error in ('protocol', 'mount'):
            values = list(sample())
            if error == 'protocol':
                values[0].update(protocol_version=5, angle_convention='upright_joint')
            else:
                values[0]['mount']['static'] = True
            with self.assertRaises(ValueError):
                check.optical_check(*values)

    def test_quaternion_sign_is_not_a_rotation_error(self):
        self.assertAlmostEqual(check.rotation_error_deg((.5, .5, .5, .5), (-.5, -.5, -.5, -.5)), 0.)

    def test_runtime_world_rejects_old_axis_or_limits(self):
        def source(axis='-1 0 0', lower=-60.):
            return ("<sdf><world><model name='rocket_two_axis_pod_1'>"
                    "<joint name='yaw_joint'><axis><xyz>"+axis+"</xyz><limit><lower>"
                    +str(math.radians(lower))+"</lower><upper>"+str(math.radians(60.))+"</upper></limit></axis></joint>"
                    "<joint name='pitch_joint'><axis><xyz>0 -1 0</xyz><limit><lower>"
                    +str(math.radians(-80.))+"</lower><upper>"+str(math.radians(30.))+"</upper></limit></axis></joint>"
                    "<link><visual><geometry><mesh><uri>pod_fixed_cowling.dae</uri></mesh></geometry></visual></link>"
                    "</model></world></sdf>")
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'world.sdf'
            path.write_text(source())
            check.runtime_model_check(path, 1)
            for xml in (source('0 0 1'), source(lower=-90.)):
                path.write_text(xml)
                with self.assertRaisesRegex(ValueError, '轴向/范围'):
                    check.runtime_model_check(path, 1)


if __name__ == '__main__':
    unittest.main()
