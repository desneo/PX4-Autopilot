#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import random
import sys
import unittest


HERE = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("radar_manager", HERE / "radar_manager.py")
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class FakeTerrain:
    origin_elevation = 4.019969
    center_latitude = 32.217857
    center_longitude = 119.105878

    @staticmethod
    def line_of_sight(_observation):
        return True


class RadarManagerTest(unittest.TestCase):
    def make_manager(self):
        manager = object.__new__(MODULE.RadarManager)
        manager.terrain = FakeTerrain()
        manager.random = random.Random(20261017)
        manager.observations = {}
        manager.tracks = {}
        manager.next_track_id = 1
        manager.sequence = 0
        manager.rate_hz = 1.0
        return manager

    def test_profiles_use_three_and_five_kilometre_nominal_ranges(self):
        self.assertEqual(MODULE.profile_for_model("rocket_two_axis_pod_3"), ("small_uav", 3000.0))
        self.assertEqual(MODULE.profile_for_model("xiaomotuo"), ("small_target", 5000.0))
        self.assertEqual(MODULE.profile_for_model("xiaobianfu"), ("small_target", 5000.0))
        self.assertIsNone(MODULE.profile_for_model("terrain"))

    def test_confirmed_scan_is_anonymous_and_contains_measured_fields(self):
        manager = self.make_manager()
        manager.observations = {
            "xiaomotuo": MODULE.Observation(2000.0, 0.0, 100.0, -35.0, 0.0, 0.0, 1.0),
            "rocket_two_axis_pod_1": MODULE.Observation(1000.0, 0.0, 100.0, 0.0, 10.0, 0.0, 1.0),
        }
        first, first_count = manager._build_scan(1.0)
        self.assertEqual(first_count, 0)
        second, second_count = manager._build_scan(2.0)
        self.assertEqual(second_count, 2)
        payload = json.loads(second)
        self.assertEqual(payload["type"], "radar_tracks")
        self.assertEqual(payload["source"], "radar_center")
        self.assertEqual(len(payload["tracks"]), 2)
        self.assertNotIn("xiaomotuo", second.decode())
        self.assertNotIn("rocket_two_axis_pod", second.decode())
        for track in payload["tracks"]:
            self.assertIn("track_id", track)
            self.assertIn("range_m", track)
            self.assertIn("radial_speed_m_s", track)
            self.assertIn("altitude_msl_m", track)
            self.assertEqual(track["state"], "confirmed")

    def test_target_outside_nominal_and_fade_zone_is_not_detected(self):
        manager = self.make_manager()
        target = MODULE.Observation(5600.0, 0.0, 100.0, sample_time=1.0)
        self.assertFalse(manager._detected(target, 5000.0))

    def test_confirmed_track_coasts_three_scans_before_deletion(self):
        manager = self.make_manager()
        manager.observations = {
            "xiaobianfu": MODULE.Observation(1000.0, 0.0, 100.0, sample_time=1.0)
        }
        manager._build_scan(1.0)
        manager._build_scan(2.0)
        manager.observations.clear()
        for sequence in range(3):
            packet, count = manager._build_scan(3.0 + sequence)
            self.assertEqual(count, 1)
            self.assertEqual(json.loads(packet)["tracks"][0]["state"], "coasting")
        packet, count = manager._build_scan(6.0)
        self.assertEqual(count, 0)
        self.assertEqual(json.loads(packet)["tracks"], [])

    def test_station_validation(self):
        self.assertEqual(MODULE.parse_station("127.0.0.1:7790"), ("127.0.0.1", 7790))
        with self.assertRaisesRegex(ValueError, "主机:端口"):
            MODULE.parse_station("127.0.0.1")

    def test_static_target_trajectory_status_updates_observation(self):
        manager = self.make_manager()
        manager.lock = __import__("threading").Lock()
        message = type("Message", (), {"data": "POSE xiaobianfu 2000 0 100 18 0"})()
        manager.on_target_pose(message)
        observation = manager.observations["xiaobianfu"]
        self.assertEqual((observation.x, observation.y, observation.z), (2000.0, 0.0, 100.0))


if __name__ == "__main__":
    unittest.main()
