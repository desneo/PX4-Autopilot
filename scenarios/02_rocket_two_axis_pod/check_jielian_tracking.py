#!/usr/bin/env python3
"""在已运行的02场景中验收A2416A对真实xiaomotuo实体的捷联跟踪。"""
from __future__ import annotations

import argparse
import copy
from dataclasses import asdict, dataclass
import gzip
import json
import math
import multiprocessing as mp
import os
from pathlib import Path
import socket
import sys
import threading
import time
import uuid


HERE = Path(__file__).resolve().parent
DEFAULT_ASSETS = HERE.parents[2] / "px4-gazebo-assets"
TARGET_NAME = "xiaomotuo"
TRACKING_TIMEOUT_S = 2.5


@dataclass(frozen=True)
class ModelPose:
    position: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]


def rotate(quaternion: tuple[float, float, float, float],
           vector: tuple[float, float, float]) -> tuple[float, float, float]:
    """将局部向量按w/x/y/z四元数旋转到世界坐标。"""
    norm = math.sqrt(sum(value * value for value in quaternion))
    if not math.isfinite(norm) or norm <= 0:
        raise ValueError("四元数必须为非零有限值")
    w, x, y, z = (value / norm for value in quaternion)
    vx, vy, vz = vector
    tx, ty, tz = (2 * (y * vz - z * vy),
                  2 * (z * vx - x * vz),
                  2 * (x * vy - y * vx))
    return (vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx)


def target_position(source: ModelPose, *, forward_m: float,
                    right_m: float = 0.0, up_m: float = 0.0) -> tuple[float, float, float]:
    """按捷联导引头固定光轴计算目标世界坐标。

    装配机光轴为机体系+Z；图像右侧对应机体系-Y，图像上侧对应
    机体系-X。因此局部偏移为(-up, -right, forward)。
    """
    values = (forward_m, right_m, up_m, *source.position, *source.quaternion)
    if not all(math.isfinite(value) for value in values) or forward_m <= 0:
        raise ValueError("位姿和目标偏移必须为有限值，前向距离必须大于0")
    offset = rotate(source.quaternion, (-up_m, -right_m, forward_m))
    return tuple(origin + delta for origin, delta in zip(source.position, offset))


def _pose_dict(message) -> dict:
    return {
        "position": [message.position.x, message.position.y, message.position.z],
        "quaternion": [message.orientation.w, message.orientation.x,
                       message.orientation.y, message.orientation.z],
    }


def service_worker(connection, partition: str, world: str) -> None:
    """把Gazebo服务请求放到独立进程，避免50 Hz订阅回调阻塞请求。"""
    os.environ["GZ_PARTITION"] = partition
    os.environ["GZ_IP"] = "127.0.0.1"
    from gz.transport13 import Node
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.empty_pb2 import Empty
    from gz.msgs10.entity_factory_pb2 import EntityFactory
    from gz.msgs10.entity_pb2 import Entity
    from gz.msgs10.pose_pb2 import Pose
    from gz.msgs10.scene_pb2 import Scene

    node = Node()
    while True:
        request = connection.recv()
        operation = request["operation"]
        if operation == "quit":
            break
        try:
            if operation == "scene":
                success, response = node.request(
                    f"/world/{world}/scene/info", Empty(), Empty, Scene, 4000)
                if not success:
                    raise RuntimeError("读取Gazebo场景实体超时")
                result = {
                    model.name: {
                        **_pose_dict(model.pose),
                        "links": [link.name for link in model.link],
                    }
                    for model in response.model
                }
            elif operation == "set_pose":
                payload = Pose(name=request["name"])
                (payload.position.x, payload.position.y,
                 payload.position.z) = request["position"]
                (payload.orientation.w, payload.orientation.x,
                 payload.orientation.y, payload.orientation.z) = request["quaternion"]
                success, response = node.request(
                    f"/world/{world}/set_pose", payload, Pose, Boolean, 4000)
                if not success or not response.data:
                    raise RuntimeError("Gazebo set_pose未成功")
                result = True
            elif operation == "create":
                payload = EntityFactory(
                    sdf_filename=request["sdf_filename"],
                    name=request["name"], allow_renaming=False)
                success, response = node.request(
                    f"/world/{world}/create", payload, EntityFactory, Boolean, 4000)
                if not success or not response.data:
                    raise RuntimeError("Gazebo未能创建xiaomotuo实体")
                result = True
            elif operation == "remove":
                payload = Entity(name=request["name"], type=Entity.MODEL)
                success, response = node.request(
                    f"/world/{world}/remove", payload, Entity, Boolean, 4000)
                if not success or not response.data:
                    raise RuntimeError("Gazebo未能删除验收创建的xiaomotuo实体")
                result = True
            else:
                raise ValueError(f"未知Gazebo服务操作：{operation}")
            connection.send({"ok": True, "result": result})
        except Exception as exc:  # 服务进程必须把失败传回主进程
            connection.send({"ok": False, "error": str(exc)})
    connection.close()


class BinaryLink:
    """A2416A标准UDP控制/遥测链路，心跳维持同一客户端会话。"""

    def __init__(self, protocol, host: str, port: int, record):
        self.protocol = protocol
        self.destination = (host, port)
        self.record = record
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("0.0.0.0", 0))
        self.socket.settimeout(0.05)
        self.closed = threading.Event()
        self.lock = threading.Lock()
        self.latest = None
        self.latest_at = 0.0
        self.receive_thread = threading.Thread(target=self._receive, daemon=True)
        self.heartbeat_thread = threading.Thread(target=self._heartbeat, daemon=True)
        self.receive_thread.start()
        self.heartbeat_thread.start()

    def _send_one(self, command: int) -> None:
        self.socket.sendto(self.protocol.make_control(command), self.destination)

    def _heartbeat(self) -> None:
        while not self.closed.is_set():
            try:
                self._send_one(self.protocol.CMD_HEARTBEAT)
            except OSError:
                pass
            self.closed.wait(0.1)

    def _receive(self) -> None:
        while not self.closed.is_set():
            try:
                raw, source = self.socket.recvfrom(2048)
                frame = self.protocol.decode_telemetry(raw)
            except socket.timeout:
                continue
            except (OSError, ValueError):
                continue
            now = time.monotonic()
            with self.lock:
                self.latest = frame
                self.latest_at = now
            self.record("binary", {
                "source": list(source),
                "raw_hex": raw.hex(),
                "fields": asdict(frame),
            })

    def send_discrete(self, command: int) -> float:
        """协议离散命令发送3次，网关在200 ms窗口内去重。"""
        stamp = time.monotonic()
        for _ in range(3):
            self._send_one(command)
            time.sleep(0.02)
        self.record("command", {"command": command})
        return stamp

    def snapshot(self):
        with self.lock:
            return copy.deepcopy((self.latest_at, self.latest))

    def close(self) -> None:
        self.closed.set()
        self.receive_thread.join(timeout=1)
        self.heartbeat_thread.join(timeout=1)
        self.socket.close()


class TrackingAcceptance:
    def __init__(self, args):
        self.args = args
        self.output = args.output.resolve()
        self.output.mkdir(parents=True, exist_ok=True)
        self.rows: list[dict] = []
        self.cases: list[dict] = []
        self.phase = "连接"
        self.lock = threading.RLock()
        self.telemetry = None
        self.telemetry_at = 0.0
        self.source_pose = None
        self.source_pose_at = 0.0
        self.created_target = False
        self.marker = "JIELIAN_TRACKING_ACCEPTANCE_" + uuid.uuid4().hex
        self.service_process = self.service_connection = None
        self.binary = self.node = None

        protocol_root = args.assets_root / "models/gimbals/jielian-A2416A"
        if not protocol_root.is_dir():
            raise FileNotFoundError(f"未找到A2416A资产目录：{protocol_root}")
        sys.path.insert(0, str(protocol_root))
        from protocol import codec
        self.protocol = codec

        os.environ["GZ_PARTITION"] = args.partition
        os.environ["GZ_IP"] = "127.0.0.1"
        from gz.transport13 import Node
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.msgs10.stringmsg_pb2 import StringMsg
        self.node = Node()
        if not self.node.subscribe(StringMsg, args.topic_prefix + "/telemetry",
                                   self._on_telemetry):
            raise RuntimeError("订阅A2416A插件遥测失败")
        pose_topic = f"/world/{args.world}/dynamic_pose/info"
        if not self.node.subscribe(Pose_V, pose_topic, self._on_poses):
            raise RuntimeError("订阅Gazebo动态位姿失败")

        context = mp.get_context("spawn")
        self.service_connection, child = context.Pipe()
        self.service_process = context.Process(
            target=service_worker,
            args=(child, args.partition, args.world), daemon=True)
        self.service_process.start()
        child.close()
        self.binary = BinaryLink(codec, args.udp_host, args.udp_port, self.record)

    def record(self, kind: str, value: dict) -> None:
        now = time.monotonic()
        with self.lock:
            self.rows.append({
                "kind": kind, "wall_time": now,
                "phase": self.phase, "value": value,
            })

    def _on_telemetry(self, message) -> None:
        try:
            value = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if value.get("source") != "gazebo_plugin":
            return
        now = time.monotonic()
        with self.lock:
            self.telemetry, self.telemetry_at = value, now
            self.rows.append({
                "kind": "plugin", "wall_time": now,
                "phase": self.phase, "value": value,
            })

    def _on_poses(self, message) -> None:
        for pose in message.pose:
            if pose.name != self.args.aircraft_model:
                continue
            value = ModelPose(
                (pose.position.x, pose.position.y, pose.position.z),
                (pose.orientation.w, pose.orientation.x,
                 pose.orientation.y, pose.orientation.z),
            )
            with self.lock:
                self.source_pose, self.source_pose_at = value, time.monotonic()
            break

    def snapshot(self) -> dict:
        with self.lock:
            telemetry = copy.deepcopy(self.telemetry)
            telemetry_at = self.telemetry_at
            source_pose = copy.deepcopy(self.source_pose)
            source_pose_at = self.source_pose_at
        binary_at, binary = self.binary.snapshot()
        return {
            "plugin": telemetry, "plugin_at": telemetry_at,
            "binary": binary, "binary_at": binary_at,
            "source_pose": source_pose, "source_pose_at": source_pose_at,
        }

    def request(self, operation: str, **values):
        self.service_connection.send({"operation": operation, **values})
        if not self.service_connection.poll(7):
            raise AssertionError(f"Gazebo服务请求超时：{operation}")
        response = self.service_connection.recv()
        if not response["ok"]:
            raise AssertionError(response["error"])
        self.record("service", {"operation": operation, **values})
        return response["result"]

    def wait(self, predicate, label: str, timeout: float = 10.0,
             *, after: float = 0.0) -> dict:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            snapshot = self.snapshot()
            now = time.monotonic()
            fresh = (
                snapshot["plugin"] is not None and snapshot["binary"] is not None
                and snapshot["source_pose"] is not None
                and now - snapshot["plugin_at"] < 0.5
                and now - snapshot["binary_at"] < 0.5
                and now - snapshot["source_pose_at"] < 0.5
                and snapshot["plugin_at"] > after
                and snapshot["binary_at"] > after
            )
            if fresh and predicate(snapshot):
                return snapshot
            time.sleep(0.01)
        snapshot = self.snapshot()
        brief = {
            "plugin": snapshot["plugin"],
            "binary": asdict(snapshot["binary"]) if snapshot["binary"] else None,
            "source_pose": asdict(snapshot["source_pose"])
            if snapshot["source_pose"] else None,
        }
        raise AssertionError(f"{label}超时：{brief}")

    def case(self, label: str, predicate, timeout: float = 10.0,
             *, after: float = 0.0) -> dict:
        snapshot = self.wait(predicate, label, timeout, after=after)
        evidence = {
            "tracking_state": snapshot["plugin"].get("tracking_state"),
            "locked_target_name": snapshot["plugin"].get("locked_target_name"),
            "measurement_seq": snapshot["plugin"].get("measurement_seq"),
            "misalignment_deg": snapshot["plugin"].get("misalignment_deg"),
            "gate_px": snapshot["plugin"].get("gate_px"),
            "binary_work_state": snapshot["binary"].work_state,
            "binary_yaw_misalignment_deg": snapshot["binary"].yaw_misalignment_deg,
        }
        self.cases.append({"name": label, "passed": True, "evidence": evidence})
        print(f"{label}：通过", flush=True)
        return snapshot

    def _write_target_model(self) -> Path:
        from target_manager import make_visual_model
        source = self.args.assets_root / "models/vehicles/xiaomotuo/model.sdf"
        output = self.output / "xiaomotuo_acceptance.sdf"
        make_visual_model(source, output, TARGET_NAME)

        import xml.etree.ElementTree as ET
        tree = ET.parse(output)
        model = tree.getroot().find("model")
        ET.SubElement(model, "link", name=self.marker)
        ET.indent(tree, space="  ")
        tree.write(output, encoding="utf-8", xml_declaration=True)
        return output

    def create_target(self) -> None:
        scene = self.request("scene")
        if TARGET_NAME in scene:
            raise AssertionError(
                "场景已有xiaomotuo；为避免覆盖人工靶机，请先在02界面终止靶机后重试")
        model = self._write_target_model()
        self.created_target = True
        self.request("create", sdf_filename=str(model), name=TARGET_NAME)
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            scene = self.request("scene")
            if TARGET_NAME in scene and self.marker in scene[TARGET_NAME]["links"]:
                return
            time.sleep(0.05)
        raise AssertionError("xiaomotuo创建后未在场景中出现唯一验收标志")

    def place(self, *, right_m: float) -> float:
        snapshot = self.snapshot()
        source = snapshot["source_pose"]
        if source is None:
            raise AssertionError("尚未获得装配飞机真实位姿")
        position = target_position(
            source, forward_m=self.args.forward_m, right_m=right_m)
        stamp = time.monotonic()
        self.request("set_pose", name=TARGET_NAME, position=position,
                     quaternion=(1.0, 0.0, 0.0, 0.0))
        self.record("target_pose", {
            "name": TARGET_NAME, "position": position,
            "right_m": right_m, "forward_m": self.args.forward_m,
            "source_pose": asdict(source),
        })
        return stamp

    @staticmethod
    def locked(snapshot: dict) -> bool:
        plugin = snapshot["plugin"]
        binary = snapshot["binary"]
        return (
            plugin.get("tracking_state") == "TRACKING"
            and plugin.get("locked_target_name") == TARGET_NAME
            and plugin.get("targets")
            and plugin["targets"][0].get("name") == TARGET_NAME
            and plugin["targets"][0].get("type") == 4
            and bool(binary.work_state & 0x01)
            and binary.targets[0].type == 4
            and binary.gate_w > 0 and binary.gate_h > 0
            and abs(binary.yaw_misalignment_deg
                    - plugin["misalignment_deg"]["yaw"]) <= 0.06
        )

    def run(self) -> dict:
        self.phase = "连接真实接口"
        self.case(
            "真实50 Hz插件遥测与100 Hz二进制遥测已连接",
            lambda value: value["plugin"].get("type") == "telemetry"
            and value["plugin"].get("measurement_seq", 0) > 0,
            timeout=30,
        )
        self.create_target()
        self.phase = "光轴中心检测"
        center_stamp = self.place(right_m=0.0)
        recognition_stamp = self.binary.send_discrete(
            self.protocol.CMD_RECOGNITION_ON)
        self.case(
            "识别开启并检测真实xiaomotuo实体",
            lambda value: value["plugin"].get("recognition_enabled") is True
            and any(target.get("name") == TARGET_NAME
                    for target in value["plugin"].get("targets", [])),
            after=max(center_stamp, recognition_stamp),
        )

        self.phase = "自动锁定"
        lock_stamp = self.binary.send_discrete(self.protocol.CMD_AUTO_LOCK_ON)
        locked = self.case(
            "自动锁定xiaomotuo且固定占用槽位1",
            self.locked, after=lock_stamp)
        first_sequence = locked["plugin"]["measurement_seq"]

        self.phase = "横穿视场左侧"
        left_stamp = self.place(right_m=-self.args.cross_m)
        left = self.case(
            "目标移动到图像左侧后保持同一目标",
            lambda value: self.locked(value)
            and value["plugin"]["misalignment_deg"]["yaw"] < -2.0,
            after=left_stamp,
        )

        self.phase = "横穿视场右侧"
        right_stamp = self.place(right_m=self.args.cross_m)
        right = self.case(
            "目标横穿到图像右侧且偏航失调角变为正",
            lambda value: self.locked(value)
            and value["plugin"]["misalignment_deg"]["yaw"] > 2.0,
            after=right_stamp,
        )
        if left["plugin"]["measurement_seq"] <= first_sequence:
            raise AssertionError("目标横穿期间测量序号没有增长")

        self.phase = "短暂丢失"
        lost_stamp = self.place(right_m=self.args.outside_m)
        temporarily_lost = self.case(
            "目标离开视场后进入TEMP_LOST并保留原锁定目标",
            lambda value: value["plugin"].get("tracking_state") == "TEMP_LOST"
            and value["plugin"].get("locked_target_name") == TARGET_NAME
            and bool(value["binary"].work_state & 0x01),
            after=lost_stamp,
        )
        temp_lost_at = temporarily_lost["plugin_at"]
        time.sleep(self.args.lost_hold_s)
        if time.monotonic() - temp_lost_at >= TRACKING_TIMEOUT_S:
            raise AssertionError("测试配置使目标返回时间超过2.5 s重捕获窗口")

        self.phase = "重捕获"
        return_stamp = self.place(right_m=-self.args.cross_m / 2.0)
        recovered = self.case(
            "2.5 s窗口内返回后重捕获同一目标",
            lambda value: self.locked(value)
            and value["plugin"]["measurement_seq"]
            > right["plugin"]["measurement_seq"],
            after=return_stamp,
        )
        reacquire_s = recovered["plugin_at"] - temp_lost_at
        if not 0 < reacquire_s < TRACKING_TIMEOUT_S:
            raise AssertionError(f"重捕获耗时不在(0,2.5) s：{reacquire_s:.3f}")

        return {
            "passed": True,
            "world": self.args.world,
            "partition": self.args.partition,
            "aircraft_model": self.args.aircraft_model,
            "target": TARGET_NAME,
            "topic_prefix": self.args.topic_prefix,
            "udp": f"{self.args.udp_host}:{self.args.udp_port}",
            "forward_m": self.args.forward_m,
            "cross_m": self.args.cross_m,
            "outside_m": self.args.outside_m,
            "temp_lost_hold_s": self.args.lost_hold_s,
            "reacquire_s": reacquire_s,
            "cases": self.cases,
            "evidence": (
                "Gazebo真实xiaomotuo实体位姿、插件50 Hz测量和标准104字节UDP遥测；"
                "没有注入或伪造插件遥测"
            ),
        }

    def close(self) -> list[str]:
        errors = []
        if self.binary is not None:
            try:
                self.binary.send_discrete(self.protocol.CMD_AUTO_LOCK_OFF)
                self.binary.send_discrete(self.protocol.CMD_RECOGNITION_OFF)
                self.binary.close()
            except Exception as exc:
                errors.append("UDP清理：" + str(exc))
        if self.created_target and self.service_connection is not None:
            try:
                scene = self.request("scene")
                if (TARGET_NAME in scene
                        and self.marker in scene[TARGET_NAME].get("links", [])):
                    self.request("remove", name=TARGET_NAME)
                elif TARGET_NAME in scene:
                    raise RuntimeError("同名实体已失去本次唯一标志，拒绝删除")
            except Exception as exc:
                errors.append("目标清理：" + str(exc))
        if self.node is not None:
            try:
                self.node.unsubscribe(self.args.topic_prefix + "/telemetry")
                self.node.unsubscribe(
                    f"/world/{self.args.world}/dynamic_pose/info")
            except Exception as exc:
                errors.append("订阅清理：" + str(exc))
        if self.service_process is not None and self.service_process.pid is not None:
            try:
                self.service_connection.send({"operation": "quit"})
                self.service_process.join(timeout=2)
                if self.service_process.is_alive():
                    self.service_process.terminate()
                    self.service_process.join(timeout=2)
            except (EOFError, OSError) as exc:
                errors.append("服务进程清理：" + str(exc))
        if self.service_connection is not None:
            self.service_connection.close()
        with gzip.open(self.output / "jielian_tracking_samples.jsonl.gz",
                       "wt", encoding="utf-8") as stream:
            for row in self.rows:
                stream.write(json.dumps(row, ensure_ascii=False) + "\n")
        return errors


def positive_float(value: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError("必须为有限正数")
    return result


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--assets-root", type=Path, default=DEFAULT_ASSETS)
    result.add_argument("--partition", default="px4_scenario_02_rocket_two_axis_pod")
    result.add_argument("--world", default="geo_32_217857_119_105878")
    result.add_argument("--aircraft-model", default="rocket_jielian_a2416a_1")
    result.add_argument("--topic-prefix", default="/uav_1/pod")
    result.add_argument("--udp-host", default="127.0.0.1")
    result.add_argument("--udp-port", type=int, default=8001)
    result.add_argument("--forward-m", type=positive_float, default=500.0)
    result.add_argument("--cross-m", type=positive_float, default=80.0)
    result.add_argument("--outside-m", type=positive_float, default=220.0)
    result.add_argument("--lost-hold-s", type=positive_float, default=0.8)
    result.add_argument("--output", type=Path, required=True)
    return result


def validate_args(args) -> None:
    if not 1 <= args.udp_port <= 65535:
        raise ValueError("UDP端口必须在1～65535")
    half_fov_m = args.forward_m * math.tan(math.radians(37.0 / 2.0))
    if args.cross_m >= half_fov_m * 0.8:
        raise ValueError("横穿偏移必须留在37度水平视场内并保留20%裕量")
    if args.outside_m <= half_fov_m * 1.1:
        raise ValueError("丢失偏移必须超出37度水平视场至少10%")
    if args.lost_hold_s >= TRACKING_TIMEOUT_S - 0.5:
        raise ValueError("TEMP_LOST保持时间必须为重捕获留出至少0.5 s")


def main() -> int:
    args = parser().parse_args()
    args.assets_root = args.assets_root.resolve()
    args.output = args.output.resolve()
    probe = TrackingAcceptance.__new__(TrackingAcceptance)
    try:
        validate_args(args)
        probe.__init__(args)
        result = probe.run()
    except Exception as exc:
        result = {
            "passed": False,
            "error": str(exc),
            "cases": getattr(probe, "cases", []),
        }
        print("A2416A场景锁定验收失败：" + str(exc), file=sys.stderr)
    finally:
        errors = probe.close() if hasattr(probe, "output") else []
    if errors:
        result["passed"] = False
        result["cleanup_errors"] = errors
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "jielian_tracking_result.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
