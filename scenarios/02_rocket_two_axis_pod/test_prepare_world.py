#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET


HERE = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("prepare_world", HERE / "prepare_world.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class PrepareWorldTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        assets = self.root / "world" / "assets"
        assets.mkdir(parents=True)
        (assets / "terrain.dae").write_text("mesh")
        (self.root / "world" / "world.sdf").write_text(
            "<sdf version='1.10'><world name='source'>"
            "<gui><plugin filename='MapCenter' name='地图中心'>"
            "<center_pose>1 2 210 0 1.5707963267948966 1.5707963267948966</center_pose>"
            "</plugin></gui>"
            "<model name='terrain'><link name='link'><visual name='v'><geometry><mesh>"
            "<uri>assets/terrain.dae</uri></mesh></geometry></visual></link></model>"
            "</world></sdf>"
        )

    def tearDown(self):
        self.temp.cleanup()

    def make_model(self, aircraft_id: int) -> Path:
        directory = self.root / f"generated_{aircraft_id}" / "rocket_two_axis_pod"
        (directory / "meshes").mkdir(parents=True)
        (directory / "meshes" / "body.stl").write_text("mesh")
        (directory / "parameters.json").write_text(json.dumps({"simulation": {"spawn_pose": "0 0 0.24 0 0 0"}}))
        path = directory / "model.sdf"
        path.write_text(
            "<sdf version='1.9'><model name='rocket_two_axis_pod'><static>false</static>"
            "<link name='base_link'><collision name='body_collision'><geometry><box>"
            "<size>1 1 1</size></box></geometry></collision>"
            "<visual name='body'><geometry><mesh>"
            "<uri>model://rocket_two_axis_pod/meshes/body.stl</uri>"
            "</mesh></geometry></visual></link>"
            "<plugin name='two_axis::GimbalController' filename='lib.so'>"
            f"<command_topic>/uav_{aircraft_id}/pod/command</command_topic>"
            "</plugin></model></sdf>"
        )
        return path

    def add_terrain_tile(self):
        path = self.root / "world" / "assets" / "tile.dae"
        path.write_text("""<COLLADA><asset><unit meter='1'/><up_axis>Z_UP</up_axis></asset>
          <library_geometries><geometry id='terrain'><mesh><source id='positions'>
          <float_array id='positions-array' count='12'>0 0 0 20 0 -2 0 20 -4 20 20 -6</float_array>
          <technique_common><accessor source='#positions-array' count='4' stride='3'/></technique_common></source>
          <vertices id='vertices'><input semantic='POSITION' source='#positions'/></vertices>
          <triangles count='2'><input semantic='VERTEX' source='#vertices' offset='0'/><p>0 1 2 1 3 2</p></triangles>
          </mesh></geometry></library_geometries><library_visual_scenes><visual_scene id='Scene'>
          <node id='terrain-node'><instance_geometry url='#terrain'/></node></visual_scene></library_visual_scenes>
          <scene><instance_visual_scene url='#Scene'/></scene></COLLADA>""")
        source = self.root / "world" / "world.sdf"
        tree = ET.parse(source)
        model = ET.SubElement(tree.getroot().find("world"), "model", name="tile_test")
        ET.SubElement(model, "static").text = "true"
        ET.SubElement(model, "pose").text = "0 0 2.5 0 0 0"
        mesh = ET.SubElement(ET.SubElement(ET.SubElement(ET.SubElement(model, "link", name="terrain"), "collision", name="ground"), "geometry"), "mesh")
        ET.SubElement(mesh, "uri").text = str(path)
        tree.write(source)
        return source, path

    def test_ground_relative_spawn_and_marker_use_interpolated_absolute_height(self):
        source, _ = self.add_terrain_tile()
        output = self.root / "run" / "world.sdf"
        MODULE.prepare(source, output, [(1, self.make_model(1)), (2, self.make_model(2))])
        world = ET.parse(output).getroot().find("world")
        self.assertIsNone(
            world.find(
                "model[@name='tile_test']/link/collision/"
                "surface/contact/collide_bitmask"
            )
        )
        for number, expected_z in ((1, 1.215), (2, .715)):
            model_z = float(world.findtext(f"model[@name='rocket_two_axis_pod_{number}']/pose").split()[2])
            marker_z = float(world.findtext(f"model[@name='POD_LOCATION_MARKER_{number}']/pose").split()[2])
            self.assertAlmostEqual(model_z, expected_z)
            self.assertEqual(marker_z, model_z)

    def test_missing_unsupported_or_uncovered_real_terrain_never_falls_back_to_zero(self):
        source, path = self.add_terrain_tile()
        world = ET.parse(source).getroot().find("world")
        with self.assertRaisesRegex(ValueError, "覆盖范围"):
            MODULE.terrain_heights(world, [(100, 100)])
        original = path.read_text()
        path.write_text(original.replace("Z_UP", "Y_UP"))
        with self.assertRaisesRegex(ValueError, "米制Z_UP"):
            MODULE.terrain_heights(world, [(5, 5)])
        path.unlink()
        with self.assertRaisesRegex(ValueError, "存在的绝对路径DAE"):
            MODULE.terrain_heights(world, [(5, 5)])
        world.find("model[@name='tile_test']").set("name", "unknown_physical_terrain")
        with self.assertRaisesRegex(ValueError, "不能假定地面为0"):
            MODULE.terrain_heights(world, [(5, 5)])

    def test_actual_world_dual_spawn_aligns_normal_and_inherits_five_mm_tail_gap(self):
        assets = HERE.parents[2] / "px4-gazebo-assets"
        source = assets / "worlds/geo_32_217857_119_105878/world.sdf"
        if not source.is_file():
            self.skipTest("实际资产world未配置")
        output = self.root / "run" / "world.sdf"
        models = [(number, self.make_model(number)) for number in (1, 2)]
        for _, path in models:
            (path.parent / "parameters.json").write_text(json.dumps({"simulation": {"spawn_pose": "0 0 0.215 0 0 0"}}))
        MODULE.prepare(source, output, models)
        world = ET.parse(output).getroot().find("world")
        vehicle = ET.parse(assets / "models/vehicles/rocket_racing_quad/model.sdf").getroot().find("model")
        body = vehicle.find("link[@name='base_link']/collision[@name='body_collision']")
        tail_z = float(body.findtext("pose").split()[2])-float(body.findtext("geometry/cylinder/length"))/2
        for number, ground in ((1, -.251526112), (2, -.483718752)):
            pose = [float(v) for v in world.findtext(f"model[@name='rocket_two_axis_pod_{number}']/pose").split()]
            self.assertAlmostEqual(pose[2], ground+.215, places=10)
            _, normal = MODULE.terrain_surfaces(world, [(pose[0], pose[1])])[0]
            roll, pitch, yaw = pose[3:]
            self.assertEqual(yaw, 0.0)
            up = (math.sin(pitch)*math.cos(roll), -math.sin(roll), math.cos(pitch)*math.cos(roll))
            for actual, expected in zip(up, normal):
                self.assertAlmostEqual(actual, expected, places=10)
            # 机体沿法线倾转，尾底圆面平行三角面；竖直原点高度须乘nz。
            normal_gap = .215*normal[2]+tail_z
            self.assertAlmostEqual(normal_gap, .005, delta=.0003)
            marker_z = float(world.findtext(f"model[@name='POD_LOCATION_MARKER_{number}']/pose").split()[2])
            self.assertEqual(marker_z, pose[2])

    def test_spawn_height_inherits_each_model_and_explicit_override_wins(self):
        first, second = self.make_model(1), self.make_model(2)
        (first.parent / "parameters.json").write_text(json.dumps({"simulation": {"spawn_pose": "0 0 .215 0 0 0"}}))
        (second.parent / "parameters.json").write_text(json.dumps({"simulation": {"spawn_pose": "0 0 .230 0 0 0"}}))
        self.assertEqual(MODULE.spawn_altitude(first, None), .215)
        self.assertEqual(MODULE.spawn_altitude(second, None), .230)
        self.assertEqual(MODULE.spawn_altitude(first, .4), .4)
        (first.parent / "parameters.json").unlink()
        with self.assertRaisesRegex(ValueError, "simulation.spawn_pose"):
            MODULE.spawn_altitude(first, None)
        self.assertEqual(MODULE.spawn_altitude(first, .24), .24)
        for bad in (0, -1, float("nan"), float("inf")):
            with self.subTest(height=bad), self.assertRaises(ValueError):
                MODULE.spawn_altitude(first, bad)

    def test_slope_normal_is_upward_and_pose_yaw_remains_zero(self):
        source, _ = self.add_terrain_tile()
        world = ET.parse(source).getroot().find("world")
        height, normal = MODULE.terrain_surfaces(world, [(5, 5)])[0]
        self.assertAlmostEqual(height, 1.0)
        length = math.sqrt(1.05)
        for actual, expected in zip(normal, (.1/length, .2/length, 1/length)):
            self.assertAlmostEqual(actual, expected)
        roll, pitch = MODULE.ground_roll_pitch(normal)
        up = (math.sin(pitch)*math.cos(roll), -math.sin(roll), math.cos(pitch)*math.cos(roll))
        for actual, expected in zip(up, normal):
            self.assertAlmostEqual(actual, expected)

    def test_two_aircraft_get_unique_names_topics_and_grid_poses(self):
        output = self.root / "run" / "world.sdf"
        MODULE.prepare(
            self.root / "world" / "world.sdf",
            output,
            [(1, self.make_model(1)), (2, self.make_model(2))],
            target_control=True,
        )
        world = ET.parse(output).getroot().find("world")
        self.assertEqual(world.get("name"), "geo_32_217857_119_105878")
        first = world.find("model[@name='rocket_two_axis_pod_1']")
        second = world.find("model[@name='rocket_two_axis_pod_2']")
        self.assertEqual(first.findtext("pose"), "5.25 5 0.24 0 0 0")
        self.assertEqual(second.findtext("pose"), "10.25 5 0.24 0 0 0")
        self.assertEqual(
            {
                collision.findtext("surface/contact/category_bitmask")
                for collision in first.findall(".//link/collision")
            },
            {"1"},
        )
        self.assertEqual(
            {
                collision.findtext("surface/contact/collide_bitmask")
                for collision in second.findall(".//link/collision")
            },
            {"2"},
        )
        for number in (1, 2):
            marker = world.find(f"model[@name='POD_LOCATION_MARKER_{number}']")
            self.assertIsNotNone(marker)
            self.assertFalse(marker.findall(".//collision"))
        # 雷达仍位于世界中心，飞机出生点不进入雷达杆/相位中心。
        self.assertEqual(world.findtext("model[@name='scenario02_radar']/pose"), "0 0 0 0 0 0")
        self.assertEqual(
            first.findtext(".//command_topic"), "/uav_1/pod/command"
        )
        expected_targets = [
            "rocket_two_axis_pod_1",
            "rocket_two_axis_pod_2",
            "xiaomotuo",
            "xiaobianfu",
        ]
        for model in (first, second):
            controller = model.find("./plugin[@name='two_axis::GimbalController']")
            self.assertEqual(controller.findtext("tracking_backend"), "sim_truth")
            self.assertEqual(
                [item.text for item in controller.findall("sim_truth_target_name")],
                expected_targets,
            )
        self.assertTrue(Path(first.findtext(".//uri")).is_absolute())
        contact_sensor = first.find(".//sensor[@type='contact']")
        self.assertIsNotNone(contact_sensor)
        self.assertEqual(
            contact_sensor.findtext("contact/collision"), "body_collision"
        )
        self.assertEqual(contact_sensor.findtext("update_rate"), "250")
        self.assertIsNotNone(
            world.find("./plugin[@name='gz::sim::systems::Contact']")
        )
        collision_monitor = world.find(
            "./plugin[@name='scenario02::CollisionMonitor']"
        )
        self.assertIsNotNone(collision_monitor)
        self.assertEqual(
            collision_monitor.findtext("event_topic"),
            "/scenario/02/collision/event",
        )
        self.assertIsNone(collision_monitor.find("target_damage_speed_m_s"))
        target_control = world.find("./gui/plugin[@filename='TargetControl']")
        self.assertIsNotNone(target_control)
        self.assertEqual(
            target_control.findtext("command_topic"),
            "/scenario/02/target/command",
        )
        self.assertEqual(target_control.findtext("default_altitude_m"), "100.000000")
        self.assertEqual(target_control.findtext("aircraft_count"), "2")
        self.assertEqual(
            target_control.findtext("center_pose"),
            "1 2 210 0 1.5707963267948966 1.5707963267948966",
        )
        self.assertEqual(target_control.findtext("follow_service"), "/gui/follow")
        self.assertEqual(target_control.findtext("track_topic"), "/gui/track")
        self.assertEqual(
            target_control.findtext("move_to_pose_service"), "/gui/move_to/pose"
        )
        self.assertIsNone(world.find("./gui/plugin[@filename='MapCenter']"))
        gui_properties = {
            item.get("key"): item.text
            for item in target_control.findall("./gz-gui/property")
        }
        self.assertEqual(gui_properties["showTitleBar"], "false")
        self.assertEqual(gui_properties["showDockButton"], "false")
        self.assertEqual(gui_properties["showCloseButton"], "false")
        self.assertEqual(gui_properties["showCollapseButton"], "false")
        self.assertEqual(gui_properties["height"], "700")
        anchors = target_control.find("./gz-gui/anchors")
        self.assertEqual(anchors.get("target"), "3D View")
        self.assertIsNotNone(anchors.find("./line[@own='right'][@target='right']"))
        self.assertIsNotNone(anchors.find("./line[@own='top'][@target='top']"))
        self.assertEqual(
            target_control.findtext("radar_command_topic"),
            "/scenario/02/radar/command",
        )
        radar = world.find("./model[@name='scenario02_radar']")
        self.assertIsNotNone(radar)
        self.assertEqual(radar.findtext("static"), "true")
        self.assertEqual(radar.findtext("link/visual[@name='antenna']/pose"), "0 0 3.0 0 0 0")
        self.assertIsNone(radar.find(".//collision"))
        trajectory = world.find("./plugin[@name='scenario02::TargetTrajectory']")
        self.assertIsNotNone(trajectory)
        self.assertEqual(
            trajectory.findtext("command_topic"),
            "/scenario/02/target/trajectory/command",
        )

    def test_enable_collision_preserves_unfiltered_aircraft_collisions(self):
        output = self.root / "run" / "world.sdf"
        MODULE.prepare(
            self.root / "world" / "world.sdf",
            output,
            [(1, self.make_model(1)), (2, self.make_model(2))],
            enable_aircraft_collision=True,
        )
        aircraft = ET.parse(output).getroot().findall(
            "world/model[@name='rocket_two_axis_pod_1']/link/collision"
        )
        self.assertTrue(aircraft)
        self.assertTrue(
            all(collision.find("surface/contact/collide_bitmask") is None for collision in aircraft)
        )

    def test_rejects_wrong_topic_prefix(self):
        with self.assertRaisesRegex(ValueError, "吊仓话题应为"):
            MODULE.prepare(
                self.root / "world" / "world.sdf",
                self.root / "run" / "world.sdf",
                [(2, self.make_model(1))],
            )

    def test_explicit_csrt_backend_is_written_and_invalid_backend_is_rejected(self):
        output = self.root / "run" / "world.sdf"
        model = self.make_model(1)
        MODULE.prepare(
            self.root / "world" / "world.sdf",
            output,
            [(1, model)],
            tracking_backend="csrt",
        )
        controller = ET.parse(output).getroot().find(
            "world/model[@name='rocket_two_axis_pod_1']/plugin[@name='two_axis::GimbalController']"
        )
        self.assertEqual(controller.findtext("tracking_backend"), "csrt")
        self.assertEqual(
            [item.text for item in controller.findall("sim_truth_target_name")],
            ["rocket_two_axis_pod_1", "xiaomotuo", "xiaobianfu"],
        )
        with self.assertRaisesRegex(ValueError, "sim_truth 或 csrt"):
            MODULE.prepare(
                self.root / "world" / "world.sdf",
                output,
                [(1, model)],
                tracking_backend="unknown",
            )

    def test_target_default_altitude_uses_world_center_elevation(self):
        source = self.root / "world" / "world.sdf"
        source.write_text(
            "<sdf version='1.10'><world name='source'>"
            "<spherical_coordinates><elevation>4.019969</elevation>"
            "</spherical_coordinates></world></sdf>"
        )
        output = self.root / "run" / "world.sdf"
        MODULE.prepare(source, output, [(1, self.make_model(1))], target_control=True)
        world = ET.parse(output).getroot().find("world")
        self.assertEqual(
            world.findtext("./gui/plugin[@filename='TargetControl']/default_altitude_m"),
            "104.019969",
        )

    def test_headless_world_does_not_load_target_plugins(self):
        output = self.root / "run" / "world.sdf"
        MODULE.prepare(
            self.root / "world" / "world.sdf",
            output,
            [(1, self.make_model(1))],
        )
        world = ET.parse(output).getroot().find("world")
        self.assertIsNone(world.find("./gui/plugin[@filename='TargetControl']"))
        self.assertIsNotNone(world.find("./gui/plugin[@filename='MapCenter']"))
        self.assertIsNone(world.find("./plugin[@name='scenario02::TargetTrajectory']"))
        self.assertIsNotNone(
            world.find("./plugin[@name='scenario02::CollisionMonitor']")
        )
        self.assertIsNotNone(world.find("./model[@name='scenario02_radar']"))

    def test_rejects_aircraft_without_collision_geometry(self):
        model = self.make_model(1)
        tree = ET.parse(model)
        collision = tree.getroot().find(".//collision")
        parent = tree.getroot().find(".//link")
        parent.remove(collision)
        tree.write(model)
        with self.assertRaisesRegex(ValueError, "缺少碰撞体"):
            MODULE.prepare(
                self.root / "world" / "world.sdf",
                self.root / "run" / "world.sdf",
                [(1, model)],
            )

    def test_rejects_unsupported_target_collision_shape_before_start(self):
        model = self.make_model(1)
        tree = ET.parse(model)
        geometry = tree.getroot().find(".//collision/geometry")
        geometry.clear()
        ET.SubElement(geometry, "heightmap")
        tree.write(model)
        with self.assertRaisesRegex(ValueError, "不支持碰撞体形状"):
            MODULE.load_model(1, model)


if __name__ == "__main__":
    unittest.main()
