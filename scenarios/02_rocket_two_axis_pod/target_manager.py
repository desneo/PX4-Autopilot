#!/usr/bin/env python3
"""场景 02 靶机管理服务：场景航迹控制与 QGC 状态。"""

from __future__ import annotations

import argparse
import copy
from dataclasses import dataclass
import json
import math
from pathlib import Path
import signal
import socket
import struct
import subprocess
import threading
import time
import xml.etree.ElementTree as ET


COMMAND_TOPIC = "/scenario/02/target/command"
STATUS_TOPIC = "/scenario/02/target/status"
TRAJECTORY_COMMAND_TOPIC = "/scenario/02/target/trajectory/command"
TRAJECTORY_STATUS_TOPIC = "/scenario/02/target/trajectory/status"
COLLISION_EVENT_TOPIC = "/scenario/02/collision/event"
EARTH_RADIUS_M = 6_371_000.0
MODEL_RULES = {
    "xiaomotuo": {"display": "小摩托", "speed": (25.0, 65.0)},
    "xiaobianfu": {"display": "小蝙蝠", "speed": (10.0, 30.0)},
}
DIRECTIONS = {
    "east_west": {"display": "从东向西", "unit": (1.0, 0.0), "heading": 270.0},
    "northeast_45": {
        "display": "从东北45度",
        "unit": (math.sqrt(0.5), math.sqrt(0.5)),
        "heading": 225.0,
    },
}


def coordinate_offset(
    latitude_deg: float, longitude_deg: float, east_m: float, north_m: float
) -> tuple[float, float]:
    latitude = latitude_deg + math.degrees(north_m / EARTH_RADIUS_M)
    longitude = longitude_deg + math.degrees(
        east_m / (EARTH_RADIUS_M * math.cos(math.radians(latitude_deg)))
    )
    return latitude, longitude


def local_position(
    center_latitude: float, center_longitude: float,
    latitude: float, longitude: float,
) -> tuple[float, float]:
    """当前场景经纬度局部近似的逆变换，返回东、北坐标（米）。"""
    east = math.radians(longitude - center_longitude) * EARTH_RADIUS_M * math.cos(
        math.radians(center_latitude)
    )
    north = math.radians(latitude - center_latitude) * EARTH_RADIUS_M
    return east, north


def parse_station(value: str) -> tuple[str, int]:
    host, separator, port_text = value.strip().rpartition(":")
    if not separator or not host:
        raise ValueError("地面站地址格式应为 主机:端口")
    try:
        port = int(port_text)
    except ValueError as error:
        raise ValueError("地面站端口必须是整数") from error
    if not 1 <= port <= 65535:
        raise ValueError("地面站端口必须在 1～65535 范围内")
    try:
        address = socket.getaddrinfo(
            host, port, family=socket.AF_INET, type=socket.SOCK_DGRAM
        )[0][4]
    except socket.gaierror as error:
        raise ValueError(f"无法解析地面站地址 {host}：{error}") from error
    return address[0], port


@dataclass(frozen=True)
class TargetConfig:
    model: str
    speed: float
    altitude: float
    distance: float = 0.0
    direction: str = ""
    mode: str = "center_pass"
    initial_xy: tuple[float, float] = (0.0, 0.0)
    initial_heading: float = 0.0

    @property
    def source_xy(self) -> tuple[float, float]:
        if self.mode == "continuous":
            return self.initial_xy
        unit = DIRECTIONS[self.direction]["unit"]
        return self.distance * unit[0], self.distance * unit[1]

    @property
    def heading(self) -> float:
        if self.mode == "continuous":
            return self.initial_heading
        return DIRECTIONS[self.direction]["heading"]

    @property
    def yaw(self) -> float:
        return math.radians(90.0 - self.heading)

    @property
    def velocity_unit(self) -> tuple[float, float]:
        heading = math.radians(self.heading)
        return math.sin(heading), math.cos(heading)

    @property
    def endpoint_xy(self) -> tuple[float, float]:
        unit = DIRECTIONS[self.direction]["unit"]
        return -500.0 * unit[0], -500.0 * unit[1]


class TerrainMap:
    def __init__(self, metadata_path: Path, assets_dir: Path):
        self.metadata = json.loads(metadata_path.read_text())
        self.assets_dir = assets_dir
        self.origin_elevation = float(self.metadata["origin_elevation_source_m"])
        self.center_latitude = float(self.metadata["center_wgs84"]["latitude_deg"])
        self.center_longitude = float(self.metadata["center_wgs84"]["longitude_deg"])
        self.tiles = self.metadata["tiles"]

    def _tile(self, x: float, y: float) -> dict:
        for tile in self.tiles:
            center_x, center_y = tile["center_enu_m"]
            size_x, size_y = tile["size_m"]
            if (
                center_x - size_x / 2 <= x <= center_x + size_x / 2
                and center_y - size_y / 2 <= y <= center_y + size_y / 2
            ):
                return tile
        raise ValueError(f"坐标 ({x:.1f}, {y:.1f}) 超出地形范围")

    def maximum_route_height(
        self, start: tuple[float, float], end: tuple[float, float]
    ) -> float:
        distance = math.dist(start, end)
        count = max(1, math.ceil(distance / 100.0))
        maximum = -math.inf
        for index in range(count + 1):
            fraction = index / count
            x = start[0] + (end[0] - start[0]) * fraction
            y = start[1] + (end[1] - start[1]) * fraction
            maximum = max(maximum, float(self._tile(x, y)["terrain_z_max_m"]))
        return maximum


def validate_config(payload: dict, terrain: TerrainMap) -> TargetConfig:
    model = str(payload.get("model", ""))
    if model not in MODEL_RULES:
        raise ValueError("靶机型号必须选择小摩托或小蝙蝠")
    mode = payload.get("mode", "center_pass")
    if mode not in ("center_pass", "continuous"):
        raise ValueError("靶机模式只支持 center_pass 或 continuous")
    fields = ("speed", "altitude", "latitude", "longitude", "heading") if mode == "continuous" else (
        "speed", "altitude", "distance",
    )
    try:
        if any(isinstance(payload[field], bool) for field in fields):
            raise ValueError("不能使用布尔值")
        values = [float(payload[field]) for field in fields]
    except (KeyError, TypeError, ValueError, OverflowError) as error:
        raise ValueError("速度、海拔高度及航迹参数必须完整且为有效数值") from error
    if not all(math.isfinite(value) for value in values):
        raise ValueError("速度、海拔高度及航迹参数必须是有限数值")
    speed, altitude = values[:2]
    minimum_speed, maximum_speed = MODEL_RULES[model]["speed"]
    if not minimum_speed <= speed <= maximum_speed:
        raise ValueError(
            f"{MODEL_RULES[model]['display']}巡航速度必须在 "
            f"{minimum_speed:g}～{maximum_speed:g} m/s"
        )
    if mode == "continuous":
        latitude, longitude, heading = values[2:]
        if not -90 <= latitude <= 90 or not -180 <= longitude <= 180:
            raise ValueError("纬度必须在 -90～90 度，经度必须在 -180～180 度")
        return TargetConfig(
            model, speed, altitude, mode=mode,
            initial_xy=local_position(
                terrain.center_latitude, terrain.center_longitude, latitude, longitude,
            ),
            initial_heading=heading % 360.0,
        )
    distance = values[2]
    direction = str(payload.get("direction", ""))
    if direction not in DIRECTIONS:
        raise ValueError("来源方向无效")
    if not 100 <= distance <= 10000:
        raise ValueError("来源距离必须在 100～10000 米")
    unit = DIRECTIONS[direction]["unit"]
    source = distance * unit[0], distance * unit[1]
    endpoint = -500.0 * unit[0], -500.0 * unit[1]
    minimum_altitude = (
        terrain.origin_elevation
        + terrain.maximum_route_height(source, endpoint)
        + 30.0
    )
    if altitude < minimum_altitude:
        raise ValueError(
            f"海拔高度至少应为 {minimum_altitude:.1f} m，"
            "以高出整段航线最高地形30米"
        )
    return TargetConfig(model, speed, altitude, distance, direction)


def make_visual_model(source: Path, output: Path, model_name: str) -> Path:
    """生成无传感器、碰撞和控制插件的静态视觉模型运行副本。"""
    tree = ET.parse(source)
    root = tree.getroot()
    model = root.find("model")
    if model is None:
        raise ValueError(f"靶机 SDF 中没有顶层 model：{source}")
    model = copy.deepcopy(model)
    model.set("name", model_name)
    static = model.find("static")
    if static is None:
        static = ET.Element("static")
        model.insert(0, static)
    static.text = "true"
    pose = model.find("pose")
    if pose is not None:
        pose.text = "0 0 0 0 0 0"
    for parent in model.iter():
        for child in list(parent):
            if child.tag in {"sensor", "collision", "plugin"}:
                parent.remove(child)

    output.parent.mkdir(parents=True, exist_ok=True)
    result = ET.Element("sdf", version=root.get("version", "1.9"))
    result.append(model)
    ET.indent(result, space="  ")
    ET.ElementTree(result).write(output, encoding="utf-8", xml_declaration=True)
    return output


def mavlink2_frame(
    message_id: int,
    payload: bytes,
    crc_extra: int,
    sequence: int,
    system_id: int,
    component_id: int = 1,
) -> bytes:
    """打包一个不签名的 MAVLink 2 数据帧，不依赖 pymavlink。"""
    if not 0 <= message_id <= 0xFFFFFF:
        raise ValueError("MAVLink message_id 超出范围")
    if len(payload) > 255:
        raise ValueError("MAVLink payload 过长")
    header = bytes(
        (
            len(payload),
            0,
            0,
            sequence & 0xFF,
            system_id,
            component_id,
            message_id & 0xFF,
            (message_id >> 8) & 0xFF,
            (message_id >> 16) & 0xFF,
        )
    )
    checksum = 0xFFFF
    for value in header + payload + bytes((crc_extra,)):
        temporary = value ^ (checksum & 0xFF)
        temporary = (temporary ^ (temporary << 4)) & 0xFF
        checksum = (
            (checksum >> 8)
            ^ (temporary << 8)
            ^ (temporary << 3)
            ^ (temporary >> 4)
        ) & 0xFFFF
    return b"\xfd" + header + payload + struct.pack("<H", checksum)


class MavlinkTargetProxy:
    """向 QGC 单向广播轻量靶机状态；不监听或响应任何控制指令。"""

    def __init__(self, address: tuple[str, int], system_id: int):
        if not 1 <= system_id <= 254:
            raise ValueError("靶机 MAVLink SYSID 必须在 1～254 范围内")
        self.address = address
        self.system_id = system_id
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sequence = 0
        self.started_at = time.monotonic()
        self.last_heartbeat = -math.inf
        self.last_position = -math.inf
        self.last_gps = -math.inf

    def _send(self, message_id: int, payload: bytes, crc_extra: int) -> None:
        frame = mavlink2_frame(
            message_id,
            payload,
            crc_extra,
            self.sequence,
            self.system_id,
        )
        self.sequence = (self.sequence + 1) & 0xFF
        self.socket.sendto(frame, self.address)

    def update(
        self,
        config: TargetConfig,
        truth: tuple[float, float, float, float, bool],
        latitude: float,
        longitude: float,
        origin_elevation: float,
    ) -> None:
        now = time.monotonic()
        _x, _y, _z, speed, done = truth
        if now - self.last_heartbeat >= 1.0:
            # 固定翼、通用飞控、已解锁、活动状态。使用 GENERIC 避免 QGC
            # 将此轻量代理误判为版本不受支持的 PX4 飞控。
            heartbeat = struct.pack("<IBBBBB", 0, 1, 0, 129, 4, 3)
            self._send(0, heartbeat, 50)
            self.last_heartbeat = now

        heading = int(round(config.heading * 100)) % 36000
        unit_east, unit_north = config.velocity_unit
        velocity = 0.0 if done else speed
        velocity_east = unit_east * velocity
        velocity_north = unit_north * velocity
        boot_ms = int((now - self.started_at) * 1000) & 0xFFFFFFFF
        altitude_mm = int(round(config.altitude * 1000))
        relative_altitude_mm = int(round((config.altitude - origin_elevation) * 1000))

        if now - self.last_position >= 0.2:
            # QGC 地图机头读取姿态 yaw，不读取 GLOBAL_POSITION_INT.hdg。
            # MAVLink 使用北为零、顺时针为正的 NED yaw，与 Gazebo ENU yaw 不同。
            yaw_ned = math.radians((config.heading + 180.0) % 360.0 - 180.0)
            attitude = struct.pack(
                "<Iffffff", boot_ms, 0.0, 0.0, yaw_ned, 0.0, 0.0, 0.0,
            )
            self._send(30, attitude, 39)
            global_position = struct.pack(
                "<IiiiihhhH",
                boot_ms,
                int(round(latitude * 1e7)),
                int(round(longitude * 1e7)),
                altitude_mm,
                relative_altitude_mm,
                int(round(velocity_north * 100)),
                int(round(velocity_east * 100)),
                0,
                heading,
            )
            self._send(33, global_position, 104)
            vfr_hud = struct.pack(
                "<ffffhH",
                float(velocity),
                float(velocity),
                float(config.altitude),
                0.0,
                heading // 100,
                0,
            )
            self._send(74, vfr_hud, 20)
            self.last_position = now

        if now - self.last_gps >= 1.0:
            gps_raw = struct.pack(
                "<QiiiHHHHBB",
                int(time.time() * 1_000_000),
                int(round(latitude * 1e7)),
                int(round(longitude * 1e7)),
                altitude_mm,
                100,
                150,
                int(round(velocity * 100)),
                heading,
                3,
                12,
            )
            self._send(24, gps_raw, 24)
            self.last_gps = now

    def close(self) -> None:
        self.socket.close()


class TargetSession:
    def __init__(self, manager: "TargetManager", config: TargetConfig):
        self.manager = manager
        self.config = config
        self.stop_event = threading.Event()
        self.collided_event = threading.Event()
        self.truth_event = threading.Event()
        self.truth_lock = threading.Lock()
        self.truth: tuple[float, float, float, float, bool] | None = None
        self.mavlink_proxy = MavlinkTargetProxy(
            manager.qgc_address, manager.mavlink_system_id
        )
        self.mavlink_failed = False
        self.model_created = False
        self.cleaned = False
        self.state = "starting"

    def _status(self, state: str, message: str) -> None:
        with self.manager.lock:
            if state in ("starting", "running") and (
                self.stop_event.is_set() or self.collided_event.is_set()
            ):
                return
            self.state = state
            if state == "error":
                self.manager.publish_status(state, message, self.config.model, error=message)
            else:
                self.manager.publish_status(state, message, self.config.model)

    def _run_gz_service(self, service: str, request_type: str, request: str) -> bool:
        command = [
            "gz", "service", "-s", service,
            "--reqtype", request_type,
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "5000",
            "--req", request,
        ]
        result = subprocess.run(command, capture_output=True, text=True, timeout=8)
        return result.returncode == 0 and "data: true" in result.stdout

    def create_model(self) -> None:
        source = (
            self.manager.assets_root
            / "models" / "vehicles" / self.config.model / "model.sdf"
        )
        output = (
            self.manager.runtime_dir / "target" / f"{self.config.model}_visual.sdf"
        )
        make_visual_model(source, output, self.config.model)
        x, y = self.config.source_xy
        z = self.config.altitude - self.manager.terrain.origin_elevation
        yaw = self.config.yaw
        request = (
            f'sdf_filename: "{output}" '
            f'name: "{self.config.model}" allow_renaming: false '
            f'pose: {{position: {{x: {x:.9f}, y: {y:.9f}, z: {z:.9f}}}, '
            f'orientation: {{z: {math.sin(yaw / 2):.12f}, w: {math.cos(yaw / 2):.12f}}}}}'
        )
        if not self._run_gz_service(
            f"/world/{self.manager.world}/create", "gz.msgs.EntityFactory", request
        ):
            raise RuntimeError("Gazebo 未能生成靶机视觉模型")
        self.model_created = True

    def remove_model(self) -> None:
        if not self.model_created:
            return
        if not self._run_gz_service(
            f"/world/{self.manager.world}/remove",
            "gz.msgs.Entity",
            f'name: "{self.config.model}" type: MODEL',
        ):
            raise RuntimeError("Gazebo 未能移除靶机视觉模型")
        self.model_created = False

    def trajectory_command(self) -> str:
        start_x, start_y = self.config.source_xy
        z = self.config.altitude - self.manager.terrain.origin_elevation
        yaw = self.config.yaw
        if self.config.mode == "continuous":
            return (
                f"START_CONTINUOUS {self.config.model} {start_x:.12g} {start_y:.12g} "
                f"{z:.12g} {self.config.speed:.12g} {yaw:.12g}"
            )
        end_x, end_y = self.config.endpoint_xy
        return (
            f"START {self.config.model} {start_x:.12g} {start_y:.12g} "
            f"{z:.12g} {end_x:.12g} {end_y:.12g} "
            f"{self.config.speed:.12g} {yaw:.12g}"
        )

    def on_truth(
        self, model: str, x: float, y: float, z: float, speed: float, done: bool
    ) -> None:
        if model != self.config.model:
            return
        with self.truth_lock:
            self.truth = (x, y, z, speed, done)
        self.truth_event.set()

    def on_collision(self, relative_speed: float) -> None:
        if self.collided_event.is_set() or self.stop_event.is_set():
            return
        self.collided_event.set()
        self.manager.publish_trajectory("STOP")
        self._status(
            "collided",
            f"靶机已与 PX4 飞机碰撞（相对速度 {relative_speed:.1f} m/s），"
            "航迹和 QGC 状态已停止，请终止靶机清理模型",
        )

    def _send_mavlink(self) -> None:
        with self.truth_lock:
            truth = self.truth
        if truth is None:
            return
        x, y, _z, _speed, _done = truth
        latitude, longitude = coordinate_offset(
            self.manager.terrain.center_latitude,
            self.manager.terrain.center_longitude,
            x,
            y,
        )
        try:
            self.mavlink_proxy.update(
                self.config,
                truth,
                latitude,
                longitude,
                self.manager.terrain.origin_elevation,
            )
        except (OSError, ValueError, OverflowError, struct.error) as error:
            if not self.mavlink_failed:
                self.mavlink_failed = True
                self._status(
                    "running",
                    f"靶机仍在运行，但 QGC 状态发送失败：{error}；"
                    "将继续重试",
                )
            return
        if self.mavlink_failed:
            self.mavlink_failed = False
            self._status("running", "QGC 状态发送已恢复，靶机继续运行")

    def run(self) -> None:
        if self.stop_event.is_set():
            return
        self._status("starting", "正在生成靶机视觉模型…")
        self.create_model()
        if self.stop_event.is_set():
            return
        command = self.trajectory_command()
        deadline = time.monotonic() + 10.0
        while (time.monotonic() < deadline and not self.stop_event.is_set()
               and not self.collided_event.is_set()):
            with self.manager.lock:
                if self.stop_event.is_set() or self.collided_event.is_set():
                    break
                self.manager.publish_trajectory(command)
            if self.truth_event.wait(0.2):
                break
        else:
            if self.stop_event.is_set():
                return
            if not self.collided_event.is_set():
                raise RuntimeError("场景航迹插件未返回靶机位置")

        if not self.stop_event.is_set() and not self.collided_event.is_set():
            destination = (
                f"沿航向 {self.config.heading:g}° 持续飞行，等待 agent 停止"
                if self.config.mode == "continuous" else "飞向中心点"
            )
            self._status(
                "running",
                f"{MODEL_RULES[self.config.model]['display']}正以 "
                f"{self.config.speed:g} m/s {destination}",
            )
        endpoint_reported = False
        while not self.stop_event.wait(0.05):
            with self.manager.lock:
                if self.stop_event.is_set() or self.collided_event.is_set():
                    continue
                self._send_mavlink()
            with self.truth_lock:
                done = self.truth is not None and self.truth[4]
            if self.config.mode != "continuous" and done and not endpoint_reported:
                self._status("running", "靶机已越过中心点500米，正在终点等待")
                endpoint_reported = True

    def cleanup(self) -> None:
        if self.cleaned:
            return
        self.stop_event.set()
        self.manager.publish_trajectory("STOP")
        self.mavlink_proxy.close()
        self.remove_model()
        self.cleaned = True


class TargetManager:
    def __init__(self, args):
        from gz.msgs10.stringmsg_pb2 import StringMsg
        from gz.transport13 import Node

        self.StringMsg = StringMsg
        self.world = args.world
        self.runtime_dir = args.runtime_dir.resolve()
        self.assets_root = args.assets_root.resolve()
        self.qgc_address = parse_station(args.qgc_address)
        self.mavlink_system_id = args.mavlink_system_id
        if not 1 <= self.mavlink_system_id <= 254:
            raise ValueError("靶机 MAVLink SYSID 必须在 1～254 范围内")
        world_dir = self.assets_root / "worlds" / self.world
        self.terrain = TerrainMap(world_dir / "metadata.json", world_dir / "assets")
        # transport 回调可能在订阅成功后立即到达，先准备所有共享状态。
        self.lock = threading.RLock()
        self.session: TargetSession | None = None
        self.pending_config: TargetConfig | None = None
        self.worker: threading.Thread | None = None
        self.shutdown_event = threading.Event()
        self.node = Node()
        self.status_publisher = self.node.advertise(STATUS_TOPIC, StringMsg)
        self.trajectory_publisher = self.node.advertise(
            TRAJECTORY_COMMAND_TOPIC, StringMsg
        )
        if not self.node.subscribe(StringMsg, COMMAND_TOPIC, self.on_command):
            raise RuntimeError(f"无法订阅靶机命令话题：{COMMAND_TOPIC}")
        if not self.node.subscribe(
            StringMsg, TRAJECTORY_STATUS_TOPIC, self.on_trajectory_status
        ):
            raise RuntimeError(f"无法订阅航迹状态话题：{TRAJECTORY_STATUS_TOPIC}")
        if not self.node.subscribe(
            StringMsg, COLLISION_EVENT_TOPIC, self.on_collision_event
        ):
            raise RuntimeError(f"无法订阅碰撞事件话题：{COLLISION_EVENT_TOPIC}")

    def _message(self, value: str):
        message = self.StringMsg()
        message.data = value
        return message

    def publish_trajectory(self, value: str) -> None:
        self.trajectory_publisher.publish(self._message(value))

    def publish_status(
        self, state: str, message: str, model: str | None = None,
        *, error: str | None = None,
    ) -> None:
        payload = {"state": state, "message": message}
        if model:
            payload["model"] = model
        if error:
            payload["error"] = error
        value = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.status_publisher.publish(self._message(value))
        print(value, flush=True)

    def reject_command(self, message: str) -> None:
        """拒绝请求时保留当前会话状态，避免 GUI 将仍在飞行的靶机隐藏。"""
        with self.lock:
            session = self.session
            state = session.state if session is not None else "error"
            model = session.config.model if session is not None else None
            self.publish_status(state, message, model, error=message)

    def on_trajectory_status(self, message) -> None:
        fields = message.data.split()
        if len(fields) != 7 or fields[0] != "POSE":
            return
        try:
            model = fields[1]
            x, y, z, speed = map(float, fields[2:6])
            done = bool(int(fields[6]))
        except (ValueError, TypeError):
            return
        with self.lock:
            session = self.session
            if session is not None:
                session.on_truth(model, x, y, z, speed, done)

    def on_command(self, message) -> None:
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("命令必须是 JSON 对象")
            command = payload.get("command")
            if command == "start":
                self.start(payload)
            elif command == "stop":
                self.stop()
            else:
                raise ValueError("未知靶机命令")
        except (json.JSONDecodeError, ValueError) as error:
            self.reject_command(str(error))

    def on_collision_event(self, message) -> None:
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                return
            if payload.get("collision_type") != "px4_target":
                return
            relative_speed = float(payload["relative_speed_m_s"])
            target_model = str(payload.get("entity_2", ""))
            if not math.isfinite(relative_speed) or relative_speed < 0:
                return
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            return
        with self.lock:
            session = self.session
            if session is not None and target_model == session.config.model:
                session.on_collision(relative_speed)

    def _ensure_worker(self) -> None:
        """由持有 lock 的调用者启动唯一的会话工作线程。"""
        if self.worker is None:
            self.worker = threading.Thread(target=self._run_sessions, daemon=True)
            self.worker.start()

    def start(self, payload: dict) -> None:
        with self.lock:
            if self.shutdown_event.is_set():
                self.reject_command("靶机管理服务正在退出")
                return
            if payload.get("mode", "center_pass") == "center_pass" and (
                self.session is not None or self.pending_config is not None
            ):
                self.reject_command("靶机正在运行，请先终止靶机")
                return
            try:
                config = validate_config(payload, self.terrain)
            except ValueError as error:
                self.reject_command(str(error))
                return
            # 替换期间只保留最新有效配置；旧会话清理完才消费它。
            self.pending_config = config
            if self.session is not None:
                self.session.stop_event.set()
                self.session._status("stopping", "正在清理当前靶机并准备替换…")
            self._ensure_worker()

    def _run_sessions(self) -> None:
        while True:
            with self.lock:
                if self.shutdown_event.is_set():
                    self.pending_config = None
                if self.session is None:
                    config = self.pending_config
                    self.pending_config = None
                    if config is None:
                        self.worker = None
                        return
                    try:
                        self.session = TargetSession(self, config)
                    except Exception as error:
                        self.worker = None
                        self.publish_status("error", f"无法创建靶机会话：{error}")
                        return
                session = self.session
            error = None
            try:
                session.run()
            except Exception as caught:
                if session.stop_event.is_set():
                    print(f"靶机启动过程已按请求终止：{caught}", flush=True)
                else:
                    error = caught
                    print(f"靶机运行失败：{caught}", flush=True)
            try:
                session.cleanup()
            except Exception as caught:
                # 保留旧会话以便 stop 重试清理，禁止生成同名或第二个模型。
                with self.lock:
                    self.pending_config = None
                    self.worker = None
                    session._status("error", f"靶机清理失败，未启动新靶机：{caught}")
                return
            with self.lock:
                self.session = None
                if self.shutdown_event.is_set():
                    self.pending_config = None
                if self.pending_config is not None:
                    continue
                self.worker = None
                if not self.shutdown_event.is_set():
                    if error is not None:
                        self.publish_status("error", f"靶机启动或航迹控制失败：{error}")
                    else:
                        self.publish_status("idle", "靶机已终止，航迹和 QGC 状态已停止")
                return

    def stop(self) -> None:
        with self.lock:
            self.pending_config = None
            session = self.session
            if session is None:
                self.publish_status("idle", "当前没有运行中的靶机")
                return
            session.stop_event.set()
            session._status("stopping", "正在终止靶机并停止航迹与 QGC 状态…")
            self._ensure_worker()

    def shutdown(self) -> None:
        self.shutdown_event.set()
        with self.lock:
            self.pending_config = None
            session = self.session
            if session:
                session.stop_event.set()
                self._ensure_worker()
            worker = self.worker
        if worker and worker is not threading.current_thread():
            worker.join(timeout=20)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", required=True)
    parser.add_argument("--runtime-dir", type=Path, required=True)
    parser.add_argument("--assets-root", type=Path, required=True)
    parser.add_argument("--qgc-address", default="127.0.0.1:14550")
    parser.add_argument("--mavlink-system-id", type=int, default=200)
    args = parser.parse_args()
    manager = TargetManager(args)

    def request_shutdown(_signum, _frame):
        manager.shutdown_event.set()

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    print(
        f"靶机管理服务已就绪：command={COMMAND_TOPIC} status={STATUS_TOPIC} "
        f"mode=scene-trajectory qgc={args.qgc_address} "
        f"sysid={args.mavlink_system_id}",
        flush=True,
    )
    manager.publish_status("idle", "等待启动靶机")
    try:
        while not manager.shutdown_event.wait(0.2):
            pass
    finally:
        manager.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
