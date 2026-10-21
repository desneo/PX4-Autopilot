#!/usr/bin/env python3
"""独立Gazebo分区：实际九个CAD网格及带偏置/缩放的活动mesh交会。"""
from __future__ import annotations
import argparse
import copy
import json
import math
import os
from pathlib import Path
import subprocess
import threading
import time
import xml.etree.ElementTree as ET

import prepare_world as prepare
from test_mesh_collision_bounds import write_box_dae

WORLD = "scenario02_mesh_collision_test"
TOPIC = "/scenario/02/collision/event"


def make_world(output: Path, assets: Path) -> Path:
    output.mkdir(parents=True, exist_ok=True)
    mesh_path = write_box_dae(output / "offset_scale.dae", lower=(-.025, -.05, .45), upper=(.025, .05, .55),
        transform="<translate>0 0 .25</translate><rotate>0 0 1 90</rotate>")
    # DART要求法线数量与顶点数量相同；测试网格与资产导出格式一致。
    tree = ET.parse(mesh_path)
    ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
    dae_mesh = tree.find("c:library_geometries/c:geometry/c:mesh", ns)
    normal_source = ET.SubElement(dae_mesh, "{"+ns["c"]+"}source", id="normals")
    ET.SubElement(normal_source, "{"+ns["c"]+"}float_array", id="normals-array", count="24").text = " ".join(["1 0 0"]*8)
    common = ET.SubElement(normal_source, "{"+ns["c"]+"}technique_common")
    accessor = ET.SubElement(common, "{"+ns["c"]+"}accessor", source="#normals-array", count="8", stride="3")
    for axis in "XYZ":
        ET.SubElement(accessor, "{"+ns["c"]+"}param", name=axis, type="float")
    triangles = dae_mesh.find("c:triangles", ns)
    ET.SubElement(triangles, "{"+ns["c"]+"}input", semantic="NORMAL", source="#normals", offset="1")
    indices = triangles.find("c:p", ns)
    indices.text = " ".join(value for index in indices.text.split() for value in (index, index))
    ET.register_namespace("", ns["c"])
    tree.write(mesh_path, encoding="utf-8", xml_declaration=True)
    root = ET.Element("sdf", version="1.10")
    world = ET.SubElement(root, "world", name=WORLD)
    for filename, name in (("physics", "Physics"), ("user-commands", "UserCommands")):
        ET.SubElement(world, "plugin", filename=f"gz-sim-{filename}-system", name=f"gz::sim::systems::{name}")
    prepare.set_text(world, "gravity", "0 0 0")
    physics = ET.SubElement(world, "physics", name="physics", type="ignored")
    prepare.set_text(physics, "max_step_size", ".004")
    prepare.set_text(physics, "real_time_factor", "1")
    aircraft = ET.fromstring(f'''<model name="rocket_two_axis_pod_1"><static>false</static><pose>0 0 10 0 0 0</pose>
      <link name="mount_link"><inertial><mass>1</mass><inertia><ixx>.1</ixx><iyy>.1</iyy><izz>.1</izz></inertia></inertial></link>
      <link name="moving_link"><pose>.8 0 0 0 0 0</pose><inertial><pose>.6 0 1.5 0 0 0</pose><mass>1</mass><inertia><ixx>.01</ixx><iyy>.01</iyy><izz>.01</izz></inertia></inertial>
        <collision name="offset_mesh"><pose>.6 0 0 0 0 0</pose><geometry><mesh><uri>{mesh_path}</uri><scale>2 1 2</scale></mesh></geometry></collision>
      </link><joint name="installation" type="fixed"><parent>world</parent><child>mount_link</child></joint>
      <joint name="yaw_joint" type="revolute"><parent>mount_link</parent><child>moving_link</child><axis><xyz>-1 0 0</xyz><limit><lower>-1.0471975512</lower><upper>1.0471975512</upper><effort>100</effort><velocity>2</velocity></limit><dynamics><damping>.1</damping></dynamics></axis></joint>
      <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController"><joint_name>yaw_joint</joint_name><topic>/scenario02_mesh/articulation</topic><p_gain>40</p_gain><i_gain>0</i_gain><d_gain>5</d_gain><cmd_max>100</cmd_max><cmd_min>-100</cmd_min></plugin>
      <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher"/>
    </model>''')
    world.append(aircraft)
    pod_path = assets / "models/gimbals/two_axis_pod/model.sdf"
    actual_pod = copy.deepcopy(ET.parse(pod_path).find("model"))
    actual_pod.set("name", "rocket_two_axis_pod_9")
    prepare.set_text(actual_pod, "static", "true")
    prepare.set_text(actual_pod, "pose", "100 0 10 0 0 0")
    for parent in actual_pod.iter():
        for item in list(parent):
            if item.tag in {"sensor", "plugin"}:
                parent.remove(item)
    for uri in actual_pod.findall(".//mesh/uri"):
        uri.text = str(pod_path.parent / uri.text.removeprefix("model://two_axis_pod/"))
    assert len(actual_pod.findall("link/collision/geometry/mesh")) == 9
    world.append(actual_pod)
    world.append(ET.fromstring('<model name="xiaobianfu"><static>true</static><pose>-100 -100 10 0 0 0</pose><link name="target"/></model>'))
    plugin = prepare.add_collision_systems(world)
    assert len(plugin.findall("mesh_collision")) == 10
    ET.indent(root, space="  ")
    path = output / "world.sdf"
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)
    return path


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--plugin-dir", type=Path, default=Path(__file__).resolve().parent / "gui/build/target-system-plugins")
    ap.add_argument("--assets-root", type=Path, default=Path(__file__).resolve().parents[3] / "px4-gazebo-assets")
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    args.output = args.output.resolve()
    world_path = make_world(args.output, args.assets_root.resolve())
    os.environ["GZ_PARTITION"] = f"scenario02_mesh_collision_{os.getpid()}"
    os.environ["GZ_IP"] = "127.0.0.1"
    os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"] = str(args.plugin_dir.resolve())
    from gz.msgs10.stringmsg_pb2 import StringMsg
    from gz.msgs10.double_pb2 import Double
    from gz.msgs10.model_pb2 import Model
    from gz.transport13 import Node
    node = Node()
    events, positions = [], []
    lock = threading.Lock()

    def on_event(message):
        with lock:
            events.append(json.loads(message.data))

    def on_joints(message):
        for joint in message.joint:
            if joint.name == "yaw_joint":
                with lock:
                    positions.append(joint.axis1.position)

    assert node.subscribe(StringMsg, TOPIC, on_event)
    assert node.subscribe(Model, f"/world/{WORLD}/model/rocket_two_axis_pod_1/joint_state", on_joints)
    publisher = node.advertise("/scenario02_mesh/articulation", Double)

    def service(name, reqtype, request):
        result = subprocess.run(["gz", "service", "-s", f"/world/{WORLD}/{name}",
            "--reqtype", reqtype, "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", request],
            text=True, capture_output=True, timeout=5)
        if result.returncode or "data: true" not in result.stdout:
            raise RuntimeError(f"{name}失败：{result.stdout} {result.stderr}")

    def move(position):
        x, y, z = position
        service("set_pose", "gz.msgs.Pose", f'name: "xiaobianfu" position: {{x:{x} y:{y} z:{z}}} orientation: {{w:1}}')

    def clear():
        move((-100., -100., 10.))
        time.sleep(.2)
        with lock:
            events.clear()

    results = []
    def case(label, position, expected):
        clear(); move(position); time.sleep(.5)
        with lock:
            observed = list(events)
        assert (not observed) if expected is None else (len(observed) == 1 and observed[0]["aircraft_ids"] == [expected]), (label, observed)
        if observed:
            assert observed[0]["damage_candidate"] is True
        results.append(dict(case=label, expected_aircraft=expected, events=observed, passed=True))
        print("PASS:", label, flush=True)

    with (args.output / "gazebo.log").open("w+") as log:
        server = subprocess.Popen(["gz", "sim", "-s", "-v2", str(world_path)], stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic()+15
            while f"/world/{WORLD}/set_pose" not in node.service_list():
                if server.poll() is not None or time.monotonic() > deadline:
                    log.seek(0); raise RuntimeError("mesh世界未就绪："+log.read())
                time.sleep(.1)
            service("control", "gz.msgs.WorldControl", "pause:false")
            time.sleep(.8)
            case("实际新CAD九个mesh交会", (100., 0., 10.056), 9)
            case("实际新CAD九个mesh外侧不误判", (100., 2., 10.056), None)
            case("非零mesh局部中心不当作collision原点", (1.4, 0., 10.), None)
            case("mesh节点变换/缩放/link及collision偏置交会", (1.4, 0., 11.5), 1)
            target = (1.4, 1.5*math.sin(math.pi/3), 10+1.5*math.cos(math.pi/3))
            case("关节零位尚未转入目标位置", target, None)
            angle = Double(); angle.data = math.pi/3
            deadline = time.monotonic()+8
            while time.monotonic() < deadline:
                publisher.publish(angle)
                with lock:
                    reached = positions and abs(positions[-1]-math.pi/3) < .04
                    detected = [event for event in events if event["aircraft_ids"] == [1]]
                if reached and detected:
                    break
                time.sleep(.05)
            else:
                raise AssertionError(f"动态mesh未随- X关节进入目标：positions={positions[-5:]}, events={events}")
            time.sleep(.3)
            with lock:
                detected = [event for event in events if event["aircraft_ids"] == [1]]
                actual_position = positions[-1]
            assert len(detected) == 1, detected
            results.append(dict(case="实际活动关节60度带动偏置mesh交会且持续接触去重", passed=True,
                target_joint_rad=angle.data, actual_joint_rad=actual_position, events=detected))
            print("PASS: 实际活动关节60度带动偏置mesh交会且持续接触去重", flush=True)
        finally:
            server.terminate()
            try:
                server.wait(timeout=5)
            except subprocess.TimeoutExpired:
                server.kill(); server.wait(timeout=5)
    (args.output / "summary.json").write_text(json.dumps(dict(result="passed", world=str(world_path), tests=results), indent=2, ensure_ascii=False)+"\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
