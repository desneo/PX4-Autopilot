#!/usr/bin/env python3
"""A2416A场景锁定验收工具的离线坐标与参数契约测试。"""
from __future__ import annotations

import argparse
import importlib.util
import math
from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location(
    "check_jielian_tracking", HERE / "check_jielian_tracking.py")
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class TrackingAcceptanceMathTest(unittest.TestCase):
    def test_identity_pose_uses_plus_z_optical_axis(self):
        source = MODULE.ModelPose((10.0, 20.0, 30.0), (1.0, 0.0, 0.0, 0.0))
        self.assertEqual(
            MODULE.target_position(source, forward_m=500.0),
            (10.0, 20.0, 530.0),
        )

    def test_image_right_and_up_follow_confirmed_sign_convention(self):
        source = MODULE.ModelPose((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
        self.assertEqual(
            MODULE.target_position(source, forward_m=500.0,
                                   right_m=80.0, up_m=20.0),
            (-20.0, -80.0, 500.0),
        )

    def test_vehicle_quaternion_rotates_complete_trajectory(self):
        half = math.sqrt(0.5)
        source = MODULE.ModelPose((1.0, 2.0, 3.0), (half, 0.0, 0.0, half))
        actual = MODULE.target_position(source, forward_m=10.0, right_m=4.0)
        for value, expected in zip(actual, (5.0, 2.0, 13.0)):
            self.assertAlmostEqual(value, expected)

    def test_default_trajectory_has_inside_and_outside_fov_margin(self):
        args = argparse.Namespace(
            udp_port=8001, forward_m=500.0, cross_m=80.0,
            outside_m=220.0, lost_hold_s=0.8)
        MODULE.validate_args(args)
        half = args.forward_m * math.tan(math.radians(37.0 / 2.0))
        self.assertLess(args.cross_m, half)
        self.assertGreater(args.outside_m, half)

    def test_invalid_port_or_reacquire_window_is_rejected(self):
        baseline = dict(forward_m=500.0, cross_m=80.0,
                        outside_m=220.0, lost_hold_s=0.8)
        with self.assertRaisesRegex(ValueError, "UDP端口"):
            MODULE.validate_args(argparse.Namespace(udp_port=0, **baseline))
        with self.assertRaisesRegex(ValueError, "重捕获"):
            MODULE.validate_args(argparse.Namespace(
                udp_port=8001, **{**baseline, "lost_hold_s": 2.1}))

    def test_nonfinite_target_geometry_is_rejected(self):
        source = MODULE.ModelPose((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
        with self.assertRaises(ValueError):
            MODULE.target_position(source, forward_m=float("nan"))
        with self.assertRaises(ValueError):
            MODULE.target_position(source, forward_m=0.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
