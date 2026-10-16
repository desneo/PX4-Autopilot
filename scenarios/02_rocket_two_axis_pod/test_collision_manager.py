#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest


HERE = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location(
    "collision_manager", HERE / "collision_manager.py"
)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class FakeController:
    def __init__(self, nav_state=None, failures=None):
        self.nav_state = nav_state
        self.failures = failures or {}
        self.stopped = []

    def navigation_state(self, aircraft_id):
        return self.nav_state

    def stop_all_motors(self, aircraft_id):
        self.stopped.append(aircraft_id)
        return self.failures.get(aircraft_id, True), "ack"


class CollisionManagerTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.log = Path(self.temp.name) / "collision-events.jsonl"

    def tearDown(self):
        self.temp.cleanup()

    def processor(self, controller):
        return MODULE.CollisionProcessor(
            4, controller, self.log, 32.217857, 119.105878, 4.019969
        )

    @staticmethod
    def event(collision_type, ids, speed=20.0):
        return {
            "schema_version": 1,
            "collision_type": collision_type,
            "simulation_time_s": 12.5,
            "aircraft_ids": ids,
            "relative_speed_m_s": speed,
            "position_enu_m": {"x": 100, "y": 200, "z": 50},
        }

    def test_px4_pair_stops_all_motors_on_both_aircraft(self):
        controller = FakeController()
        result = self.processor(controller).process(
            self.event("px4_px4", [2, 1])
        )
        self.assertEqual(controller.stopped, [1, 2])
        self.assertTrue(result["damage_triggered"])
        self.assertEqual(result["damage_action"], "all_motors_off")
        self.assertIn("position_wgs84", result)

    def test_target_collision_uses_inclusive_ten_metres_per_second_threshold(self):
        slow_controller = FakeController()
        slow = self.processor(slow_controller).process(
            self.event("px4_target", [1], 9.999)
        )
        self.assertFalse(slow["damage_triggered"])
        self.assertEqual(slow_controller.stopped, [])

        fast_controller = FakeController()
        fast = MODULE.CollisionProcessor(
            4,
            fast_controller,
            Path(self.temp.name) / "fast.jsonl",
            32.217857,
            119.105878,
            4.019969,
        ).process(self.event("px4_target", [1], 10.0))
        self.assertTrue(fast["damage_triggered"])
        self.assertEqual(fast_controller.stopped, [1])

    def test_surface_contact_exempts_explicit_normal_landing_modes(self):
        for nav_state in MODULE.NORMAL_LANDING_NAV_STATES:
            controller = FakeController(nav_state=nav_state)
            result = MODULE.CollisionProcessor(
                4,
                controller,
                Path(self.temp.name) / f"land-{nav_state}.jsonl",
                32.217857,
                119.105878,
                4.019969,
            ).process(self.event("px4_surface", [1]))
            self.assertFalse(result["damage_triggered"])
            self.assertEqual(result["reason"], "normal_landing_mode")
            self.assertEqual(controller.stopped, [])

    def test_surface_contact_with_unknown_state_fails_safe_to_damage(self):
        controller = FakeController(nav_state=None)
        result = self.processor(controller).process(
            self.event("px4_surface", [3])
        )
        self.assertTrue(result["damage_triggered"])
        self.assertEqual(controller.stopped, [3])

    def test_repeated_collision_does_not_issue_duplicate_failure(self):
        controller = FakeController()
        processor = self.processor(controller)
        event = self.event("px4_target", [1], 15)
        processor.process(event)
        result = processor.process(event)
        self.assertEqual(controller.stopped, [1])
        self.assertTrue(result["damage_results"][0]["already_damaged"])

    def test_parse_navigation_state_and_jsonl_output(self):
        self.assertEqual(MODULE.parse_nav_state(" nav_state: 18\n"), 18)
        self.assertIsNone(MODULE.parse_nav_state("arming_state: 2"))
        processor = self.processor(FakeController())
        processor.process(self.event("px4_target", [1], 5))
        saved = json.loads(self.log.read_text().splitlines()[0])
        self.assertEqual(saved["reason"], "relative_speed_below_10_m_s")

    def test_rejects_invalid_event(self):
        with self.assertRaisesRegex(ValueError, "飞机 ID"):
            MODULE.validate_event(self.event("px4_surface", [5]), 4)
        with self.assertRaisesRegex(ValueError, "2 个"):
            MODULE.validate_event(self.event("px4_px4", [1]), 4)


if __name__ == "__main__":
    unittest.main()
