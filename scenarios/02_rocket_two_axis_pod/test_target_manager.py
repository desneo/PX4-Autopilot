#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
import struct
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
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
    center_latitude = 32.21785737811048
    center_longitude = 119.1058778440883

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

    def continuous_payload(self, **changes):
        value = {
            "mode": "continuous", "model": "xiaomotuo", "speed": 35,
            "altitude": 104.019969, "latitude": FakeTerrain.center_latitude,
            "longitude": FakeTerrain.center_longitude, "heading": 73.5,
        }
        value.update(changes)
        return value

    def test_continuous_coordinates_round_trip_without_terrain_restrictions(self):
        latitude, longitude = MODULE.coordinate_offset(
            FakeTerrain.center_latitude, FakeTerrain.center_longitude, 16000, -12000,
        )
        terrain = FakeTerrain()
        terrain.maximum_route_height = mock.Mock(side_effect=AssertionError("不应查询地形"))
        config = MODULE.validate_config(self.continuous_payload(
            latitude=latitude, longitude=longitude, altitude=-100, heading=-450,
        ), terrain)
        self.assertAlmostEqual(config.source_xy[0], 16000)
        self.assertAlmostEqual(config.source_xy[1], -12000)
        self.assertEqual(config.altitude, -100)
        self.assertEqual(config.heading, 270)
        self.assertAlmostEqual(config.yaw, -math.pi)
        self.assertAlmostEqual(config.velocity_unit[0], -1)
        self.assertAlmostEqual(config.velocity_unit[1], 0)

    def test_continuous_requires_all_numeric_parameters(self):
        for field in ("speed", "altitude", "latitude", "longitude", "heading"):
            with self.subTest(field=field):
                payload = self.continuous_payload()
                del payload[field]
                with self.assertRaises(ValueError):
                    MODULE.validate_config(payload, FakeTerrain())
            for invalid in (None, True, "bad", float("nan"), float("inf")):
                with self.subTest(field=field, invalid=invalid):
                    with self.assertRaises(ValueError):
                        MODULE.validate_config(
                            self.continuous_payload(**{field: invalid}), FakeTerrain(),
                        )

    def test_continuous_validates_geographic_bounds_mode_and_speed(self):
        for changes in (
            {"latitude": 90.1}, {"latitude": -90.1},
            {"longitude": 180.1}, {"longitude": -180.1},
            {"mode": "bad"}, {"mode": {}}, {"model": "bad"},
            {"speed": 24.9}, {"speed": 65.1},
            {"model": "xiaobianfu", "speed": 35},
        ):
            with self.subTest(changes=changes):
                with self.assertRaises(ValueError):
                    MODULE.validate_config(self.continuous_payload(**changes), FakeTerrain())
        for latitude, longitude in ((-90, -180), (90, 180)):
            MODULE.validate_config(self.continuous_payload(
                latitude=latitude, longitude=longitude,
                model="xiaobianfu", speed=18,
            ), FakeTerrain())

    def test_continuous_model_pose_and_trajectory_use_same_heading(self):
        config = MODULE.validate_config(self.continuous_payload(), FakeTerrain())
        manager = SimpleNamespace(
            qgc_address=("127.0.0.1", 14550), mavlink_system_id=200,
            terrain=FakeTerrain(), runtime_dir=Path("/tmp/test-target-manager"),
            assets_root=Path("/tmp/test-assets"), world="test", lock=threading.RLock(),
        )
        with mock.patch.object(MODULE.socket, "socket"):
            session = MODULE.TargetSession(manager, config)
        with mock.patch.object(MODULE, "make_visual_model"), mock.patch.object(
            session, "_run_gz_service", return_value=True,
        ) as service:
            session.create_model()
        request = service.call_args.args[2]
        self.assertIn("z: 100.000000000", request)
        self.assertIn(f"z: {math.sin(config.yaw / 2):.12f}", request)
        command = session.trajectory_command().split()
        self.assertEqual(command[:2], ["START_CONTINUOUS", "xiaomotuo"])
        self.assertEqual(len(command), 7)
        self.assertAlmostEqual(float(command[4]), 100)
        self.assertAlmostEqual(float(command[6]), config.yaw)

    def test_continuous_mavlink_reports_actual_heading_and_velocity(self):
        config = MODULE.validate_config(self.continuous_payload(), FakeTerrain())
        with mock.patch.object(MODULE.socket, "socket") as socket:
            proxy = MODULE.MavlinkTargetProxy(("127.0.0.1", 14550), 200)
            proxy.update(config, (0, 0, 100, 35, False),
                         FakeTerrain.center_latitude, FakeTerrain.center_longitude,
                         FakeTerrain.origin_elevation)
        frames = {call.args[0][7]: call.args[0] for call in socket.return_value.sendto.call_args_list}
        position = struct.unpack("<IiiiihhhH", frames[33][10:-2])
        self.assertEqual(position[-1], 7350)
        self.assertEqual(position[5], round(3500 * math.cos(math.radians(73.5))))
        self.assertEqual(position[6], round(3500 * math.sin(math.radians(73.5))))
        self.assertEqual(position[3], 104020)

    def test_qgc_attitude_decodes_with_reference_mavlink_dialect(self):
        # 使用仓库自带的生成式解析器校验完整帧及 CRC，避免只复查手工打包格式。
        dialect_path = HERE.parents[1] / (
            "src/modules/mavlink/mavlink/pymavlink/dialects/v20/common.py"
        )
        spec = importlib.util.spec_from_file_location("target_reference_mavlink", dialect_path)
        dialect = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(dialect)
        configs = [MODULE.validate_config(self.payload(direction=direction), FakeTerrain())
                   for direction in MODULE.DIRECTIONS]
        configs.extend(MODULE.validate_config(
            self.continuous_payload(heading=heading), FakeTerrain(),
        ) for heading in (0, 90, 180, 270, 73.5, -45, 719.5))
        for config in configs:
            for done in (False, True):
                with self.subTest(mode=config.mode, heading=config.heading, done=done):
                    with mock.patch.object(MODULE.socket, "socket") as socket:
                        proxy = MODULE.MavlinkTargetProxy(("127.0.0.1", 14550), 200)
                        proxy.update(config, (0, 0, 100, config.speed, done),
                                     FakeTerrain.center_latitude, FakeTerrain.center_longitude,
                                     FakeTerrain.origin_elevation)
                    parser = dialect.MAVLink(None)
                    messages = [parser.parse_char(call.args[0])
                                for call in socket.return_value.sendto.call_args_list]
                    by_type = {message.get_type(): message for message in messages}
                    attitude = by_type["ATTITUDE"]
                    yaw_deg = math.degrees(attitude.yaw) % 360.0
                    difference = (yaw_deg - config.heading + 180.0) % 360.0 - 180.0
                    self.assertAlmostEqual(difference, 0, places=4)
                    self.assertEqual(attitude.roll, 0)
                    self.assertEqual(attitude.pitch, 0)
                    self.assertEqual((attitude.rollspeed, attitude.pitchspeed, attitude.yawspeed), (0, 0, 0))
                    self.assertEqual(attitude.get_srcSystem(), 200)
                    position = by_type["GLOBAL_POSITION_INT"]
                    self.assertEqual(attitude.time_boot_ms, position.time_boot_ms)
                    self.assertAlmostEqual(position.hdg / 100, config.heading, places=2)
                    if done:
                        self.assertEqual((position.vx, position.vy), (0, 0))
                    else:
                        course = math.degrees(math.atan2(position.vy, position.vx)) % 360
                        difference = (yaw_deg - course + 180.0) % 360.0 - 180.0
                        self.assertAlmostEqual(difference, 0, delta=0.02)

    def test_qgc_attitude_updates_at_five_hz(self):
        config = MODULE.validate_config(self.payload(), FakeTerrain())
        with mock.patch.object(MODULE.socket, "socket") as socket, mock.patch.object(
            MODULE.time, "monotonic", side_effect=(100.0, 100.0, 100.1, 100.21),
        ):
            proxy = MODULE.MavlinkTargetProxy(("127.0.0.1", 14550), 200)
            for _ in range(3):
                proxy.update(config, (0, 0, 100, 35, False),
                             FakeTerrain.center_latitude, FakeTerrain.center_longitude,
                             FakeTerrain.origin_elevation)
        attitudes = [call.args[0] for call in socket.return_value.sendto.call_args_list
                     if call.args[0][7] == 30]
        self.assertEqual(len(attitudes), 2)
        for frame in attitudes:
            values = struct.unpack("<Iffffff", frame[10:-2])
            self.assertAlmostEqual(values[3], -math.pi / 2, places=6)

    def test_qgc_encoding_error_is_retried_without_stopping_session(self):
        manager = SimpleNamespace(
            qgc_address=("127.0.0.1", 14550), mavlink_system_id=200,
            terrain=FakeTerrain(), lock=threading.RLock(), publish_status=mock.Mock(),
        )
        config = MODULE.validate_config(self.continuous_payload(), FakeTerrain())
        with mock.patch.object(MODULE.socket, "socket"):
            session = MODULE.TargetSession(manager, config)
        session.on_truth(config.model, 0, 0, 100, 35, False)
        session.mavlink_proxy.update = mock.Mock(side_effect=[struct.error("超出编码范围"), None])
        session._send_mavlink()
        self.assertTrue(session.mavlink_failed)
        self.assertFalse(session.stop_event.is_set())
        session._send_mavlink()
        self.assertFalse(session.mavlink_failed)
        self.assertIn("恢复", manager.publish_status.call_args.args[1])

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
        self.assertEqual([frame[7] for frame, _ in fake.sent], [0, 30, 33, 74, 24])
        self.assertTrue(all(frame[5] == 200 for frame, _ in fake.sent))
        self.assertTrue(all(address == ("127.0.0.1", 14550) for _, address in fake.sent))

    def test_target_contact_freezes_trajectory_even_at_zero_relative_speed(self):
        class FakeManager:
            def __init__(self):
                self.lock = threading.RLock()
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


class TargetLifecycleTest(unittest.TestCase):
    payload = TargetManagerTest.payload
    continuous_payload = TargetManagerTest.continuous_payload

    def setUp(self):
        self.manager = MODULE.TargetManager.__new__(MODULE.TargetManager)
        self.manager.lock = threading.RLock()
        self.manager.terrain = FakeTerrain()
        self.manager.session = None
        self.manager.pending_config = None
        self.manager.worker = None
        self.manager.shutdown_event = threading.Event()
        self.manager.publish_status = mock.Mock()
        self.created = []
        self.trace = []
        created, trace = self.created, self.trace

        class Session:
            def __init__(self, manager, config):
                self.manager, self.config = manager, config
                self.state = "starting"
                self.stop_event = threading.Event()
                self.started = threading.Event()
                self.cleaning = threading.Event()
                self.allow_cleanup = threading.Event()
                self.allow_cleanup.set()
                self.fail_cleanup = False
                created.append(self)
                trace.append(("create", config.heading))

            def _status(self, state, message):
                self.state = state
                self.manager.publish_status(state, message, self.config.model)

            def run(self):
                if self.stop_event.is_set():
                    return
                self._status("running", "运行中")
                self.started.set()
                if not self.stop_event.wait(5):
                    raise RuntimeError("测试未及时停止会话")

            def cleanup(self):
                self.cleaning.set()
                if not self.allow_cleanup.wait(5):
                    raise RuntimeError("测试未释放清理屏障")
                if self.fail_cleanup:
                    raise RuntimeError("模型移除失败")
                trace.append(("clean", self.config.heading))

        self.patch = mock.patch.object(MODULE, "TargetSession", Session)
        self.patch.start()
        self.addCleanup(self.patch.stop)
        self.addCleanup(self.finish)

    def finish(self):
        for session in self.created:
            session.fail_cleanup = False
            session.allow_cleanup.set()
        self.manager.shutdown()

    def wait_for(self, predicate):
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.005)
        self.fail("未等到预期会话状态")

    def start_first(self):
        self.manager.start(self.continuous_payload(heading=10))
        self.wait_for(lambda: len(self.created) == 1 and self.created[0].started.is_set())
        return self.created[0]

    def test_replacement_waits_for_cleanup_and_keeps_latest_valid_start(self):
        first = self.start_first()
        first.allow_cleanup.clear()
        self.manager.start(self.continuous_payload(heading=20))
        self.assertTrue(first.cleaning.wait(3))
        self.manager.start(self.continuous_payload(heading=30))
        self.manager.start(self.continuous_payload(latitude=91))
        self.assertEqual(len(self.created), 1)
        first.allow_cleanup.set()
        self.wait_for(lambda: len(self.created) == 2 and self.created[1].started.is_set())
        self.assertEqual(self.trace[:3], [("create", 10), ("clean", 10), ("create", 30)])

    def test_stop_cancels_pending_replacement(self):
        first = self.start_first()
        first.allow_cleanup.clear()
        self.manager.start(self.continuous_payload(heading=20))
        self.assertTrue(first.cleaning.wait(3))
        self.manager.stop()
        first.allow_cleanup.set()
        self.wait_for(lambda: self.manager.worker is None)
        self.assertEqual(len(self.created), 1)
        self.assertIsNone(self.manager.session)
        self.assertEqual(self.manager.publish_status.call_args.args[0], "idle")

    def test_invalid_commands_and_legacy_start_preserve_running_session(self):
        first = self.start_first()
        for message in (
            "broken", "[]", '{"command":"unknown"}',
            json.dumps({"command": "start", **self.continuous_payload(heading=None)}),
            json.dumps({"command": "start", **self.payload()}),
        ):
            self.manager.on_command(SimpleNamespace(data=message))
            self.assertIs(self.manager.session, first)
            self.assertFalse(first.stop_event.is_set())
            self.assertEqual(self.manager.publish_status.call_args.args[0], "running")
            self.assertIn("error", self.manager.publish_status.call_args.kwargs)

    def test_cleanup_failure_blocks_replacement_and_can_be_retried(self):
        first = self.start_first()
        first.fail_cleanup = True
        self.manager.start(self.continuous_payload(heading=20))
        self.wait_for(lambda: self.manager.worker is None)
        self.assertIs(self.manager.session, first)
        self.assertIsNone(self.manager.pending_config)
        self.assertEqual(len(self.created), 1)
        self.assertEqual(self.manager.publish_status.call_args.args[0], "error")
        first.fail_cleanup = False
        self.manager.stop()
        self.wait_for(lambda: self.manager.worker is None)
        self.assertIsNone(self.manager.session)

    def test_continuous_start_replaces_collided_session(self):
        first = self.start_first()
        first.state = "collided"
        self.manager.start(self.continuous_payload(model="xiaobianfu", speed=18))
        self.wait_for(lambda: len(self.created) == 2 and self.created[1].started.is_set())
        self.assertEqual(self.created[1].config.model, "xiaobianfu")

if __name__ == "__main__":
    unittest.main()
