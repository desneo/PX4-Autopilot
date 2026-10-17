#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
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
        self.assertEqual(first.findtext("pose"), "0 0 0.24 0 0 0")
        self.assertEqual(second.findtext("pose"), "5 0 0.24 0 0 0")
        self.assertEqual(
            first.findtext(".//command_topic"), "/uav_1/pod/command"
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
        self.assertEqual(
            collision_monitor.findtext("target_damage_speed_m_s"), "10.0"
        )
        target_control = world.find("./gui/plugin[@filename='TargetControl']")
        self.assertIsNotNone(target_control)
        self.assertEqual(
            target_control.findtext("command_topic"),
            "/scenario/02/target/command",
        )
        self.assertEqual(target_control.findtext("default_altitude_m"), "100.000000")
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

    def test_rejects_wrong_topic_prefix(self):
        with self.assertRaisesRegex(ValueError, "吊仓话题应为"):
            MODULE.prepare(
                self.root / "world" / "world.sdf",
                self.root / "run" / "world.sdf",
                [(2, self.make_model(1))],
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


if __name__ == "__main__":
    unittest.main()
