#!/usr/bin/env python3
"""在独立 headless Gazebo 中验证靶机 command、仿真时间与模型生命周期。"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import socket
import struct
import subprocess
import tempfile
import threading
import time


WORLD = "scenario02_target_command_test"
COMMAND = "/scenario/02/target/command"
STATUS = "/scenario/02/target/status"
TRUTH = "/scenario/02/target/trajectory/status"
COLLISION = "/scenario/02/collision/event"
STEP_SECONDS = 0.01
ORIGIN = (32.2178573781, 119.1058778441, 100.0)
EARTH_RADIUS_M = 6_371_000.0


def stop_process(process: subprocess.Popen | None) -> None:
    if process is None or process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=12)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5)


def prepare_fixture(directory: Path, plugin: Path) -> tuple[Path, Path]:
    assets = directory / "assets"
    metadata = assets / "worlds" / WORLD / "metadata.json"
    metadata.parent.mkdir(parents=True)
    metadata.write_text(json.dumps({
        "origin_elevation_source_m": ORIGIN[2],
        "center_wgs84": {"latitude_deg": ORIGIN[0], "longitude_deg": ORIGIN[1]},
        "tiles": [{"center_enu_m": [0, 0], "size_m": [2000, 2000],
                   "terrain_z_max_m": 80}],
    }))
    for model in ("xiaomotuo", "xiaobianfu"):
        sdf = assets / "models" / "vehicles" / model / "model.sdf"
        sdf.parent.mkdir(parents=True)
        sdf.write_text(f"""<sdf version='1.10'><model name='{model}'>
  <link name='base_link'><visual name='body'><geometry>
    <box><size>1 1 1</size></box>
  </geometry></visual></link>
</model></sdf>""")
    world = directory / "world.sdf"
    world.write_text(f"""<sdf version='1.10'><world name='{WORLD}'>
  <physics name='test' type='ignored'><max_step_size>{STEP_SECONDS}</max_step_size>
    <real_time_factor>0</real_time_factor></physics>
  <gravity>0 0 0</gravity>
  <plugin filename='gz-sim-physics-system' name='gz::sim::systems::Physics'/>
  <plugin filename='gz-sim-user-commands-system' name='gz::sim::systems::UserCommands'/>
  <plugin filename='gz-sim-scene-broadcaster-system' name='gz::sim::systems::SceneBroadcaster'/>
  <plugin filename='{plugin}' name='scenario02::TargetTrajectory'/>
</world></sdf>""")
    return assets, world


class CommandHarness:
    def __init__(self):
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.empty_pb2 import Empty
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.msgs10.scene_pb2 import Scene
        from gz.msgs10.stringmsg_pb2 import StringMsg
        from gz.msgs10.world_control_pb2 import WorldControl
        from gz.msgs10.world_stats_pb2 import WorldStatistics
        from gz.transport13 import Node

        self.Boolean, self.Empty, self.Scene = Boolean, Empty, Scene
        self.StringMsg, self.WorldControl = StringMsg, WorldControl
        self.node = Node()
        self.lock = threading.Lock()
        self.statuses: list[dict] = []
        self.truths: list[tuple] = []
        self.poses: dict[str, tuple] = {}
        self.iterations = 0
        self.paused = True
        assert self.node.subscribe(StringMsg, STATUS, self.on_status)
        assert self.node.subscribe(StringMsg, TRUTH, self.on_truth)
        assert self.node.subscribe(Pose_V, f"/world/{WORLD}/pose/info", self.on_pose)
        assert self.node.subscribe(WorldStatistics, f"/world/{WORLD}/stats", self.on_stats)
        self.command_publisher = self.node.advertise(COMMAND, StringMsg)
        self.collision_publisher = self.node.advertise(COLLISION, StringMsg)
        assert self.command_publisher.valid() and self.collision_publisher.valid()

    def on_status(self, message) -> None:
        with self.lock:
            self.statuses.append(json.loads(message.data))

    def on_truth(self, message) -> None:
        values = message.data.split()
        if len(values) == 7 and values[0] == "POSE":
            with self.lock:
                self.truths.append((values[1], *map(float, values[2:6]), bool(int(values[6]))))

    def on_pose(self, message) -> None:
        with self.lock:
            for pose in message.pose:
                self.poses[pose.name] = (
                    pose.position.x, pose.position.y, pose.position.z,
                    pose.orientation.z, pose.orientation.w,
                )

    def on_stats(self, message) -> None:
        with self.lock:
            self.iterations = message.iterations
            self.paused = message.paused

    def wait_for(self, predicate, label: str, *, pump: bool = False, timeout: float = 12):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            result = predicate()
            if result:
                return result
            if pump:
                self.step(10)
            time.sleep(0.04)
        with self.lock:
            statuses = self.statuses[-5:]
        raise RuntimeError(f"等待{label}超时；最近状态：{statuses}")

    def step(self, count: int) -> None:
        with self.lock:
            target = self.iterations + count
        request = self.WorldControl(pause=True, multi_step=count)
        ok, response = self.node.request(
            f"/world/{WORLD}/control", request,
            self.WorldControl, self.Boolean, 5000,
        )
        # WorldControl 的应答可能超时，而服务已经执行；以实际迭代数为准，避免重复步进。
        self.wait_for(lambda: self.iterations >= target and self.paused,
                      f"仿真步进结束（目标 {target}，应答 {ok and response.data}）")

    def publish(self, payload: dict, *, collision: bool = False) -> None:
        publisher = self.collision_publisher if collision else self.command_publisher
        assert publisher.publish(self.StringMsg(data=json.dumps(payload))), "发送 command 失败"

    def cursor(self) -> int:
        with self.lock:
            return len(self.statuses)

    def status(self, state: str, cursor: int, *, pump: bool = True) -> dict:
        def match():
            with self.lock:
                return next((item for item in self.statuses[cursor:]
                             if item["state"] == state), None)
        return self.wait_for(match, f"靶机 {state}", pump=pump)

    def truth(self, model: str | None = None) -> tuple:
        with self.lock:
            return next(item for item in reversed(self.truths)
                        if model is None or item[0] == model)

    def model_names(self) -> set[str]:
        ok, scene = self.node.request(
            f"/world/{WORLD}/scene/info", self.Empty(), self.Empty, self.Scene, 5000,
        )
        assert ok, "无法读取 Gazebo scene"
        return {model.name for model in scene.model}

    def start(self, payload: dict) -> None:
        cursor = self.cursor()
        self.publish(payload)
        self.status("running", cursor)
        self.step(20)
        names = self.model_names()
        assert names == {payload["model"]}, f"靶机实体数量或型号错误：{names}"

    def stop(self) -> None:
        cursor = self.cursor()
        self.publish({"command": "stop"})
        self.status("idle", cursor)
        self.wait_for(lambda: not self.model_names(), "靶机实体移除", pump=True)


def continuous(*, model="xiaomotuo", x=0., y=0., heading=73.5, speed=35., altitude=103.) -> dict:
    return {
        "command": "start", "mode": "continuous", "model": model,
        "speed": speed, "altitude": altitude, "heading": heading,
        "latitude": ORIGIN[0] + math.degrees(y / EARTH_RADIUS_M),
        "longitude": ORIGIN[1] + math.degrees(x / (EARTH_RADIUS_M * math.cos(math.radians(ORIGIN[0])))),
    }


def drain_packets(receiver: socket.socket, duration: float = .25) -> list[bytes]:
    packets = []
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        try:
            packets.append(receiver.recvfrom(65535)[0])
        except socket.timeout:
            pass
    return packets


def assert_no_telemetry(receiver: socket.socket) -> None:
    drain_packets(receiver)
    assert not drain_packets(receiver, .4), "停止后仍收到 QGC 状态"


def mavlink_payloads(packets: list[bytes]) -> dict[int, list[bytes]]:
    """解析测试 UDP 中的 MAVLink 2 帧，不依赖 pymavlink。"""
    messages: dict[int, list[bytes]] = {}
    for packet in packets:
        offset = 0
        while offset < len(packet):
            assert packet[offset] == 0xFD, "QGC UDP 数据缺少 MAVLink 2 帧头"
            assert len(packet) - offset >= 12, "QGC UDP 帧头或校验和不完整"
            length = packet[offset + 1]
            signed = bool(packet[offset + 2] & 1)
            frame_size = 12 + length + (13 if signed else 0)
            assert len(packet) - offset >= frame_size, "QGC UDP 帧被截断"
            assert packet[offset + 5] == 200, "QGC UDP 靶机 SYSID 错误"
            message_id = int.from_bytes(packet[offset + 7:offset + 10], "little")
            payload = packet[offset + 10:offset + 10 + length]
            messages.setdefault(message_id, []).append(payload)
            offset += frame_size
    return messages


def assert_telemetry_heading(
    receiver: socket.socket, expected_heading: float, speed: float,
    start: tuple, end: tuple,
) -> None:
    # 清空替换前的积压报文，再读取当前会话实际发出的 UDP 帧。
    drain_packets(receiver)
    messages = mavlink_payloads(drain_packets(receiver, .5))
    assert messages.get(30), "运行中未收到 QGC ATTITUDE"
    assert messages.get(33), "运行中未收到 QGC GLOBAL_POSITION_INT"
    actual_heading = math.degrees(math.atan2(end[1] - start[1], end[2] - start[2])) % 360
    assert abs((actual_heading - expected_heading + 180) % 360 - 180) < 1e-5, "实际轨迹航向错误"
    expected_yaw = math.radians((actual_heading + 180) % 360 - 180)
    for payload in messages[30]:
        _boot, roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate = struct.unpack(
            "<Iffffff", payload.ljust(28, b"\0"),
        )
        assert abs(yaw - expected_yaw) < 1e-6, (
            f"QGC ATTITUDE yaw 与实际轨迹不符：{yaw}，期望 {expected_yaw}"
        )
        assert (roll, pitch, roll_rate, pitch_rate, yaw_rate) == (0.,) * 5
    for payload in messages[33]:
        _boot, _lat, _lon, _alt, _relative_alt, north, east, down, heading = struct.unpack(
            "<IiiiihhhH", payload.ljust(28, b"\0"),
        )
        assert abs((heading / 100 - actual_heading + 180) % 360 - 180) < .011, (
            "QGC GLOBAL_POSITION_INT.hdg 与实际轨迹不符"
        )
        assert abs(north / 100 - speed * math.cos(math.radians(actual_heading))) < .011
        assert abs(east / 100 - speed * math.sin(math.radians(actual_heading))) < .011
        assert down == 0, "QGC 直平飞速度包含垂直分量"


def verify(harness: CommandHarness, receiver: socket.socket) -> None:
    # 起点在地形之外且低于旧模式净空限制，验证 command 没有沿用 GUI 限制。
    harness.start(continuous(x=1500, y=-1300))
    start = harness.truth()
    assert abs(start[1] - 1500) < 30 and abs(start[2] + 1300) < 30
    assert abs(start[3] - 3) < 1e-6 and not start[5]
    harness.step(300)
    end = harness.truth()
    expected = 35 * 3
    assert abs(end[1] - start[1] - expected * math.sin(math.radians(73.5))) < 4
    assert abs(end[2] - start[2] - expected * math.cos(math.radians(73.5))) < 4
    assert end[4] == 35 and not end[5], "连续模式错误报告终点"
    harness.wait_for(lambda: "xiaomotuo" in harness.poses, "实际模型位姿")
    pose = harness.poses["xiaomotuo"]
    assert math.dist(pose[:3], end[1:4]) < 4, "实际模型位姿与航迹状态不符"
    expected_yaw = math.radians(90 - 73.5)
    assert abs(2 * math.atan2(pose[3], pose[4]) - expected_yaw) < 1e-6
    assert_telemetry_heading(receiver, 73.5, 35, start, end)

    before_pause = harness.truth()
    time.sleep(.3)
    assert harness.truth() == before_pause, "暂停时靶机仍在移动"
    harness.step(100)
    assert math.dist(harness.truth()[1:3], before_pause[1:3]) > 30, "恢复后靶机未继续飞行"

    # 同型号替换必须重新生成起点；并穿过旧终点及地形边界持续飞行。
    harness.start(continuous(x=100, heading=270))
    start = harness.truth()
    assert abs(start[1] - 100) < 30 and abs(start[2]) < 1e-6
    harness.step(2500)
    end = harness.truth()
    assert end[1] < -750 and end[4] == 35 and not end[5], "旧终点或地图边界终止了连续飞行"
    cursor = harness.cursor()
    harness.publish(continuous(speed=0))
    rejected = harness.status("running", cursor)
    assert rejected.get("error"), "非法 start 应报告错误并保留运行状态"
    before = harness.truth()
    harness.step(100)
    assert harness.truth()[1] < before[1] - 30, "非法 start 改变了当前会话"

    harness.start(continuous(model="xiaobianfu", x=-3000, y=2000, heading=-45, speed=18))
    start = harness.truth()
    harness.step(100)
    assert_telemetry_heading(receiver, 315, 18, start, harness.truth())
    cursor = harness.cursor()
    harness.publish({
        "collision_type": "px4_target", "entity_1": "rocket_two_axis_pod_1",
        "entity_2": "xiaobianfu", "relative_speed_m_s": 42,
    }, collision=True)
    harness.status("collided", cursor, pump=False)
    harness.step(20)
    frozen = harness.truth()
    frozen_pose = harness.poses["xiaobianfu"]
    harness.step(200)
    assert harness.truth() == frozen, "碰撞后航迹未冻结"
    assert math.dist(harness.poses["xiaobianfu"][:3], frozen_pose[:3]) < 1e-6, "碰撞后实际模型未冻结"
    assert harness.model_names() == {"xiaobianfu"}, "碰撞后模型应留在场景中"
    assert_no_telemetry(receiver)

    harness.start(continuous(x=5000, heading=180))
    harness.stop()
    assert_no_telemetry(receiver)

    # 原 GUI 协议仍保留先停止再启动和终点等待行为。
    legacy = {"command": "start", "model": "xiaomotuo", "speed": 65,
              "altitude": 220, "distance": 100, "direction": "east_west"}
    harness.start(legacy)
    start = harness.truth()
    harness.step(100)
    assert_telemetry_heading(receiver, 270, 65, start, harness.truth())
    cursor = harness.cursor()
    harness.publish(legacy)
    response = harness.status("running", cursor, pump=False)
    assert "先终止" in response["message"], "旧协议重复启动行为改变"
    harness.step(1100)
    endpoint = harness.truth()
    assert abs(endpoint[1] + 500) < 1e-6 and endpoint[4:] == (0., True)
    harness.step(200)
    assert harness.truth() == endpoint, "旧协议未在终点等待"
    harness.stop()
    assert_no_telemetry(receiver)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plugin-dir", type=Path,
                        default=Path(__file__).resolve().parent / "gui/build/target-system-plugins")
    args = parser.parse_args()
    plugin = args.plugin_dir.resolve() / "libTargetTrajectory.so"
    if not plugin.is_file():
        parser.error(f"找不到航迹插件：{plugin}，请先构建 TargetTrajectory")
    os.environ.update({"GZ_PARTITION": f"scenario02_target_command_test_{os.getpid()}", "GZ_IP": "127.0.0.1"})
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("127.0.0.1", 0))
    receiver.settimeout(.05)
    server = manager = None
    with tempfile.TemporaryDirectory(prefix="scenario02_target_command_") as temporary:
        directory = Path(temporary)
        assets, world = prepare_fixture(directory, plugin)
        harness = CommandHarness()
        with (directory / "server.log").open("w+") as server_log, (directory / "manager.log").open("w+") as manager_log:
            try:
                server = subprocess.Popen(["gz", "sim", "-s", "-v2", str(world)],
                                          stdout=server_log, stderr=subprocess.STDOUT)
                harness.wait_for(lambda: f"/world/{WORLD}/control" in harness.node.service_list(), "Gazebo 控制服务")
                harness.step(1)
                manager = subprocess.Popen([
                    "python3", "-u", str(Path(__file__).resolve().parent / "target_manager.py"),
                    "--world", WORLD, "--runtime-dir", str(directory / "runtime"),
                    "--assets-root", str(assets), "--qgc-address", f"127.0.0.1:{receiver.getsockname()[1]}",
                ], stdout=manager_log, stderr=subprocess.STDOUT)
                harness.wait_for(harness.command_publisher.has_connections, "靶机 command 订阅")
                # 首次 idle 可能早于 transport 握手；通过无副作用 stop 确认双向就绪。
                def ready():
                    with harness.lock:
                        if harness.statuses:
                            return True
                    harness.publish({"command": "stop"})
                    return False
                harness.wait_for(ready, "靶机管理服务就绪")
                verify(harness, receiver)
                print("靶机 command headless 集成测试通过：连续飞行、QGC 航向、仿真时间、替换、碰撞、清理及旧航迹")
            except Exception:
                print(f"进程退出状态：Gazebo={server.poll() if server else None}，"
                      f"靶机管理服务={manager.poll() if manager else None}")
                for title, log in (("Gazebo", server_log), ("靶机管理服务", manager_log)):
                    log.flush()
                    log.seek(0)
                    print(f"{title}日志：\n{log.read()}")
                raise
            finally:
                stop_process(manager)
                stop_process(server)
                receiver.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
