#!/usr/bin/env python3
"""独立 Gazebo 世界验证实际碰撞体位姿、三维相交及零速致损事件。"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import tempfile
import threading
import time


WORLD = "scenario02_collision_test"
TOPIC = "/scenario/02/collision/event"


def model(name: str, pose: str, links: str) -> str:
    return f"<model name='{name}'><static>true</static><pose>{pose}</pose>{links}</model>"


def world_sdf() -> str:
    cylinder = (
        "<link name='base_link'><collision name='body'>"
        "<pose>0 0 0.0395 0 0 0</pose><geometry><cylinder>"
        "<radius>0.045</radius><length>0.499</length>"
        "</cylinder></geometry></collision></link>"
    )
    gap = "<link name='base_link'>" + "".join(
        f"<collision name='part_{index}'><pose>{x} 0 0 0 0 0</pose>"
        "<geometry><box><size>0.05 0.05 0.05</size></box></geometry></collision>"
        for index, x in enumerate((-0.4, 0.4))
    ) + "</link>"
    offset = (
        "<link name='base_link'/><link name='offset_link'><pose>0.7 0 0 0 0 0</pose>"
        "<collision name='offset_collision'><pose>0.2 0 0 0 0 0</pose>"
        "<geometry><sphere><radius>0.05</radius></sphere></geometry>"
        "</collision></link><joint name='fixed' type='fixed'>"
        "<parent>base_link</parent><child>offset_link</child></joint>"
    )
    models = (
        model("rocket_two_axis_pod_1", "0 0 10 0 1.5707963267948966 0", cylinder)
        + model("rocket_two_axis_pod_2", "5 0 10 0 0 0", gap)
        + model("rocket_two_axis_pod_3", "10 0 10 1.5707963267948966 0 0", cylinder)
        + model("rocket_two_axis_pod_4", "15 0 10 0 0 0", offset)
        + model("rocket_two_axis_pod_5", "-100 0 10 0 0 0", cylinder)
        + model("xiaobianfu", "-100 0 10 0 0 0", "<link name='base_link'/>")
    )
    return (
        "<sdf version='1.10'><world name='" + WORLD + "'>"
        "<plugin filename='gz-sim-physics-system' name='gz::sim::systems::Physics'/>"
        "<plugin filename='gz-sim-user-commands-system' name='gz::sim::systems::UserCommands'/>"
        "<plugin filename='libCollisionMonitor.so' name='scenario02::CollisionMonitor'/>"
        "<gravity>0 0 0</gravity>" + models + "</world></sdf>"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plugin-dir", type=Path, default=Path(__file__).resolve().parent
                        / "gui/build/target-system-plugins")
    args = parser.parse_args()
    plugin_dir = args.plugin_dir.resolve()
    if not (plugin_dir / "libCollisionMonitor.so").is_file():
        parser.error("请先构建 CollisionMonitor 插件")

    os.environ["GZ_PARTITION"] = f"scenario02_collision_test_{os.getpid()}"
    os.environ["GZ_IP"] = "127.0.0.1"
    os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"] = str(plugin_dir)
    from gz.msgs10.stringmsg_pb2 import StringMsg
    from gz.transport13 import Node

    events = []
    lock = threading.Lock()
    node = Node()

    def on_event(message):
        with lock:
            events.append(json.loads(message.data))

    if not node.subscribe(StringMsg, TOPIC, on_event):
        raise RuntimeError("无法订阅碰撞事件")

    def move(x, y, z):
        result = subprocess.run(
            ["gz", "service", "-s", f"/world/{WORLD}/set_pose",
             "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
             "--timeout", "2000", "--req",
             f'name: "xiaobianfu" position: {{x: {x} y: {y} z: {z}}} '
             'orientation: {w: 1}'],
            capture_output=True, text=True, timeout=5,
        )
        if result.returncode != 0 or "data: true" not in result.stdout:
            raise RuntimeError(f"移动测试靶机失败：{result.stdout} {result.stderr}")

    with tempfile.TemporaryDirectory(prefix="scenario02-collision-test.") as directory:
        world = Path(directory) / "world.sdf"
        world.write_text(world_sdf())
        with (Path(directory) / "gazebo.log").open("w+") as log:
            server = subprocess.Popen(
                ["gz", "sim", "-s", "-v2", str(world)],
                stdout=log, stderr=subprocess.STDOUT,
            )
            try:
                deadline = time.monotonic() + 15
                while f"/world/{WORLD}/set_pose" not in node.service_list():
                    if server.poll() is not None or time.monotonic() > deadline:
                        log.seek(0)
                        raise RuntimeError("测试世界未就绪：" + log.read())
                    time.sleep(0.1)
                time.sleep(0.5)
                result = subprocess.run(
                    ["gz", "service", "-s", f"/world/{WORLD}/control",
                     "--reqtype", "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean",
                     "--timeout", "2000", "--req", "pause: false"],
                    capture_output=True, text=True, timeout=5,
                )
                if result.returncode != 0 or "data: true" not in result.stdout:
                    raise RuntimeError("无法开始碰撞测试")
                time.sleep(0.5)
                with lock:
                    zero_speed = [event for event in events if event["aircraft_ids"] == [5]]
                if len(zero_speed) != 1 or zero_speed[0]["relative_speed_m_s"] != 0:
                    raise RuntimeError(f"零相对速度接触事件缺失或重复：{zero_speed}")
                if not zero_speed[0]["damage_candidate"]:
                    raise RuntimeError("零相对速度接触没有触发致损候选")
                print("PASS: 零相对速度接触与持续接触去重", flush=True)
                cases = (
                    ("俯仰后机头接触", (0.58, 0, 10), 1),
                    ("俯仰后上方不误判", (0, 0, 10.3), None),
                    ("独立碰撞体间隙不误判", (5, 0, 10), None),
                    ("实际偏置部件接触", (5.4, 0, 10), 2),
                    ("横滚后机头接触", (10, -0.8, 10), 3),
                    ("link与collision中心偏置", (16.25, 0, 10), 4),
                )
                for label, position, expected in cases:
                    move(-200, 0, 10)
                    time.sleep(0.2)
                    with lock:
                        events.clear()
                    move(*position)
                    deadline = time.monotonic() + 0.6
                    while time.monotonic() < deadline:
                        time.sleep(0.05)
                    with lock:
                        actual = list(events)
                    if expected is None:
                        if actual:
                            raise RuntimeError(f"{label}：发生误判 {actual}")
                    elif len(actual) != 1 or actual[0]["aircraft_ids"] != [expected]:
                        raise RuntimeError(f"{label}：事件缺失或重复 {actual}")
                    elif not actual[0]["damage_candidate"]:
                        raise RuntimeError(f"{label}：接触没有触发致损候选")
                    print(f"PASS: {label}", flush=True)
            finally:
                server.terminate()
                try:
                    server.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    server.kill()
                    server.wait(timeout=5)
    print("Gazebo 碰撞集成测试通过：实际尺寸、偏置、三维姿态和持续接触去重")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
