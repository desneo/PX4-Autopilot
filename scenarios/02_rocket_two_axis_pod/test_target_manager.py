#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest import mock
import xml.etree.ElementTree as ET


HERE = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("target_manager", HERE / "target_manager.py")
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class FakeTerrain:
    origin_elevation = 4.019969

    def maximum_route_height(self, _start, _end):
        return 2.0


class TargetManagerTest(unittest.TestCase):
    def payload(self, **changes):
        value = {
            "model": "xiaomotuo",
            "speed": 35,
            "altitude": 104.019969,
            "distance": 2000,
            "direction": "east_west",
        }
        value.update(changes)
        return value

    def test_model_specific_speed_ranges(self):
        with self.assertRaisesRegex(ValueError, "25～65"):
            MODULE.validate_config(self.payload(speed=10), FakeTerrain())
        bat = MODULE.validate_config(
            self.payload(model="xiaobianfu", speed=18), FakeTerrain()
        )
        self.assertEqual(bat.speed, 18)
        with self.assertRaisesRegex(ValueError, "10～30"):
            MODULE.validate_config(
                self.payload(model="xiaobianfu", speed=35), FakeTerrain()
            )

    def test_direction_geometry_crosses_center_and_ends_500_metres_beyond(self):
        east = MODULE.validate_config(self.payload(), FakeTerrain())
        self.assertEqual(east.source_xy, (2000, 0))
        self.assertEqual(east.endpoint_xy, (-500, 0))
        northeast = MODULE.validate_config(
            self.payload(direction="northeast_45"), FakeTerrain()
        )
        self.assertAlmostEqual(math.hypot(*northeast.source_xy), 2000)
        self.assertAlmostEqual(math.hypot(*northeast.endpoint_xy), 500)
        self.assertGreater(northeast.source_xy[0], 0)
        self.assertGreater(northeast.source_xy[1], 0)
        self.assertLess(northeast.endpoint_xy[0], 0)
        self.assertLess(northeast.endpoint_xy[1], 0)

    def test_rejects_low_altitude_and_distance(self):
        with self.assertRaisesRegex(ValueError, "最高地形30米"):
            MODULE.validate_config(self.payload(altitude=35), FakeTerrain())
        with self.assertRaisesRegex(ValueError, "100～10000"):
            MODULE.validate_config(self.payload(distance=99), FakeTerrain())
        with self.assertRaisesRegex(ValueError, "100～10000"):
            MODULE.validate_config(self.payload(distance=10001), FakeTerrain())
        self.assertEqual(
            MODULE.validate_config(self.payload(distance=100), FakeTerrain()).distance,
            100,
        )
        self.assertEqual(
            MODULE.validate_config(self.payload(distance=10000), FakeTerrain()).distance,
            10000,
        )

    def test_coordinate_offset_uses_east_and_north_axes(self):
        latitude, longitude = MODULE.coordinate_offset(32.0, 119.0, 1000, 0)
        self.assertAlmostEqual(latitude, 32.0)
        self.assertGreater(longitude, 119.0)
        latitude, longitude = MODULE.coordinate_offset(32.0, 119.0, 0, 1000)
        self.assertGreater(latitude, 32.0)
        self.assertAlmostEqual(longitude, 119.0)

    def test_visual_model_strips_runtime_systems_but_keeps_visuals(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source.sdf"
            output = root / "run" / "visual.sdf"
            source.write_text(
                "<sdf version='1.9'><model name='source'><pose>0 0 1 0 0 0</pose>"
                "<link name='base_link'><sensor name='imu' type='imu'/>"
                "<collision name='body'><geometry><box><size>1 1 1</size>"
                "</box></geometry></collision><visual name='body'><geometry><box>"
                "<size>1 1 1</size></box></geometry></visual></link>"
                "<plugin name='flight' filename='libflight.so'/></model></sdf>"
            )
            MODULE.make_visual_model(source, output, "xiaomotuo")
            model = ET.parse(output).getroot().find("model")
            self.assertEqual(model.get("name"), "xiaomotuo")
            self.assertEqual(model.findtext("static"), "true")
            self.assertEqual(model.findtext("pose"), "0 0 0 0 0 0")
            self.assertIsNotNone(model.find(".//visual"))
            self.assertIsNone(model.find(".//sensor"))
            self.assertIsNone(model.find(".//collision"))
            self.assertIsNone(model.find(".//plugin"))

    def test_mavlink2_heartbeat_matches_reference_frame(self):
        payload = struct.pack("<IBBBBB", 0, 1, 0, 129, 4, 3)
        frame = MODULE.mavlink2_frame(0, payload, 50, 0, 200)
        self.assertEqual(
            frame.hex(),
            "fd09000000c801000000000000000100810403b14a",
        )

    def test_mavlink_proxy_is_send_only_and_uses_dedicated_system_id(self):
        class FakeSocket:
            def __init__(self):
                self.sent = []

            def sendto(self, data, address):
                self.sent.append((data, address))

            def close(self):
                pass

        fake = FakeSocket()
        with mock.patch.object(MODULE.socket, "socket", return_value=fake):
            proxy = MODULE.MavlinkTargetProxy(("127.0.0.1", 14550), 200)
            config = MODULE.TargetConfig(
                "xiaomotuo", 35, 104.019969, 2000, "east_west",
            )
            proxy.update(
                config,
                (1000, 0, 100, 35, False),
                32.217857,
                119.105878,
                4.019969,
            )
        self.assertEqual([frame[7] for frame, _ in fake.sent], [0, 33, 74, 24])
        self.assertTrue(all(frame[5] == 200 for frame, _ in fake.sent))
        self.assertTrue(all(address == ("127.0.0.1", 14550) for _, address in fake.sent))

    def test_target_contact_freezes_trajectory_even_at_zero_relative_speed(self):
        class FakeManager:
            def __init__(self):
                self.qgc_address = ("127.0.0.1", 14550)
                self.mavlink_system_id = 200
                self.trajectory = []
                self.status = []

            def publish_trajectory(self, value):
                self.trajectory.append(value)

            def publish_status(self, state, message, model=None):
                self.status.append((state, message, model))

        class FakeSocket:
            def close(self):
                pass

        manager = FakeManager()
        config = MODULE.TargetConfig(
            "xiaomotuo", 35, 104.019969, 2000, "east_west",
        )
        with mock.patch.object(MODULE.socket, "socket", return_value=FakeSocket()):
            session = MODULE.TargetSession(manager, config)
        session.on_collision(0.0)
        self.assertTrue(session.collided_event.is_set())
        self.assertEqual(manager.trajectory, ["STOP"])
        self.assertEqual(manager.status[-1][0], "collided")
        session.on_collision(35.0)
        self.assertEqual(manager.trajectory, ["STOP"])

if __name__ == "__main__":
    unittest.main()
