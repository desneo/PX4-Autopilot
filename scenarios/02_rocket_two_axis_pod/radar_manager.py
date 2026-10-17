#!/usr/bin/env python3
"""场景 02 航迹级雷达：低频扫描、量测误差、航迹确认和 UDP 输出。"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import random
import signal
import socket
import threading
import time


COMMAND_TOPIC = "/scenario/02/radar/command"
STATUS_TOPIC = "/scenario/02/radar/status"
TARGET_TRAJECTORY_STATUS_TOPIC = "/scenario/02/target/trajectory/status"
EARTH_RADIUS_M = 6_371_000.0
MIN_RANGE_M = 50.0
MIN_ELEVATION_DEG = -5.0
MAX_ELEVATION_DEG = 60.0
RATE_RANGE_HZ = (1.0, 2.0)
PROFILE_RANGES_M = {
    "rocket_two_axis_pod": 3000.0,
    "xiaomotuo": 5000.0,
    "xiaobianfu": 5000.0,
}


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


def coordinate_offset(
    latitude_deg: float, longitude_deg: float, east_m: float, north_m: float
) -> tuple[float, float]:
    latitude = latitude_deg + math.degrees(north_m / EARTH_RADIUS_M)
    longitude = longitude_deg + math.degrees(
        east_m / (EARTH_RADIUS_M * math.cos(math.radians(latitude_deg)))
    )
    return latitude, longitude


def profile_for_model(name: str) -> tuple[str, float] | None:
    if name.startswith("rocket_two_axis_pod_"):
        return "small_uav", PROFILE_RANGES_M["rocket_two_axis_pod"]
    if name in ("xiaomotuo", "xiaobianfu"):
        return "small_target", PROFILE_RANGES_M[name]
    return None


@dataclass
class Observation:
    x: float
    y: float
    z: float
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    sample_time: float = 0.0


@dataclass
class Track:
    track_id: int
    hits: int = 0
    misses: int = 0
    confidence: float = 0.0
    last_measurement: dict | None = None


class TerrainMap:
    """用资产元数据近似低空地形遮挡；不会泄露目标真值身份。"""

    def __init__(self, metadata_path: Path):
        metadata = json.loads(metadata_path.read_text())
        self.origin_elevation = float(metadata["origin_elevation_source_m"])
        self.center_latitude = float(metadata["center_wgs84"]["latitude_deg"])
        self.center_longitude = float(metadata["center_wgs84"]["longitude_deg"])
        self.tiles = metadata["tiles"]

    def _terrain_max(self, x: float, y: float) -> float | None:
        for tile in self.tiles:
            center_x, center_y = tile["center_enu_m"]
            size_x, size_y = tile["size_m"]
            if (
                center_x - size_x / 2 <= x <= center_x + size_x / 2
                and center_y - size_y / 2 <= y <= center_y + size_y / 2
            ):
                return float(tile["terrain_z_max_m"])
        return None

    def line_of_sight(self, target: Observation) -> bool:
        distance = math.hypot(target.x, target.y)
        # 每个地形瓦片最多检查两次，跳过雷达和目标附近，避免把所在瓦片
        # 的局部最高点误认为必然遮挡。
        count = max(2, math.ceil(distance / 250.0))
        for index in range(1, count):
            fraction = index / count
            x = target.x * fraction
            y = target.y * fraction
            line_z = 3.0 + (target.z - 3.0) * fraction
            terrain_z = self._terrain_max(x, y)
            if terrain_z is not None and terrain_z + 0.5 >= line_z:
                return False
        return True


class RadarManager:
    def __init__(self, args):
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.msgs10.stringmsg_pb2 import StringMsg
        from gz.transport13 import Node

        self.PoseV = Pose_V
        self.StringMsg = StringMsg
        self.world = args.world
        self.terrain = TerrainMap(args.metadata)
        self.node = Node()
        self.status_publisher = self.node.advertise(STATUS_TOPIC, StringMsg)
        if not self.node.subscribe(StringMsg, COMMAND_TOPIC, self.on_command):
            raise RuntimeError(f"无法订阅雷达命令话题：{COMMAND_TOPIC}")
        # SceneBroadcaster 是正式 world 已有的系统；使用它的动态位姿话题，
        # 不额外添加高频 PosePublisher，避免重复广播整张地图的静态实体。
        self.pose_topic = f"/world/{self.world}/dynamic_pose/info"
        if not self.node.subscribe(Pose_V, self.pose_topic, self.on_poses):
            raise RuntimeError(f"无法订阅 Gazebo 位姿话题：{self.pose_topic}")
        if not self.node.subscribe(
            StringMsg, TARGET_TRAJECTORY_STATUS_TOPIC, self.on_target_pose
        ):
            raise RuntimeError(
                f"无法订阅靶机航迹状态话题：{TARGET_TRAJECTORY_STATUS_TOPIC}"
            )
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.random = random.Random(args.seed)
        self.lock = threading.Lock()
        self.shutdown_event = threading.Event()
        self.running = False
        self.rate_hz = 1.0
        self.station: tuple[str, int] | None = None
        self.observations: dict[str, Observation] = {}
        self.tracks: dict[str, Track] = {}
        self.next_track_id = 1
        self.sequence = 0
        self.last_scan = -math.inf
        self.pending: list[tuple[float, bytes, int]] = []

    def _message(self, value: str):
        message = self.StringMsg()
        message.data = value
        return message

    def publish_status(self, state: str, message: str, track_count: int = 0) -> None:
        value = json.dumps(
            {"state": state, "message": message, "track_count": track_count},
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.status_publisher.publish(self._message(value))
        print(value, flush=True)

    def on_command(self, message) -> None:
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("命令必须是 JSON 对象")
            command = payload.get("command")
            if command == "start":
                rate = float(payload.get("rate_hz", 1.0))
                if not math.isfinite(rate) or not RATE_RANGE_HZ[0] <= rate <= RATE_RANGE_HZ[1]:
                    raise ValueError("雷达航迹频率必须在 1.0～2.0 Hz")
                station = parse_station(str(payload.get("station_address", "")))
                with self.lock:
                    if self.running:
                        self.publish_status("running", "雷达正在运行，请先停止雷达", len(self.tracks))
                        return
                    self.rate_hz = rate
                    self.station = station
                    self.running = True
                    self.last_scan = -math.inf
                    self.sequence = 0
                    self.tracks.clear()
                    self.pending.clear()
                self.publish_status("running", f"雷达已启动，航迹频率 {rate:g} Hz")
            elif command == "stop":
                with self.lock:
                    was_running = self.running
                    self.running = False
                    self.station = None
                    self.tracks.clear()
                    self.pending.clear()
                self.publish_status("idle", "雷达已停止，航迹发送已终止" if was_running else "雷达未启动")
            else:
                raise ValueError("未知雷达命令")
        except (json.JSONDecodeError, TypeError, ValueError) as error:
            self.publish_status("error", str(error))

    @staticmethod
    def _sample_time(message) -> float:
        try:
            return float(message.header.stamp.sec) + float(message.header.stamp.nsec) * 1e-9
        except (AttributeError, TypeError):
            return time.monotonic()

    def on_poses(self, message) -> None:
        sample_time = self._sample_time(message)
        with self.lock:
            for pose in message.pose:
                name = str(pose.name)
                if profile_for_model(name) is None:
                    continue
                previous = self.observations.get(name)
                current = Observation(
                    float(pose.position.x), float(pose.position.y), float(pose.position.z),
                    sample_time=sample_time,
                )
                if previous is not None and sample_time > previous.sample_time:
                    dt = sample_time - previous.sample_time
                    current.vx = (current.x - previous.x) / dt
                    current.vy = (current.y - previous.y) / dt
                    current.vz = (current.z - previous.z) / dt
                self.observations[name] = current

    def on_target_pose(self, message) -> None:
        fields = message.data.split()
        if len(fields) != 7 or fields[0] != "POSE":
            return
        try:
            name = fields[1]
            x, y, z = map(float, fields[2:5])
        except (TypeError, ValueError):
            return
        if profile_for_model(name) is None:
            return
        sample_time = time.monotonic()
        with self.lock:
            previous = self.observations.get(name)
            current = Observation(x, y, z, sample_time=sample_time)
            if previous is not None and sample_time > previous.sample_time:
                dt = sample_time - previous.sample_time
                current.vx = (x - previous.x) / dt
                current.vy = (y - previous.y) / dt
                current.vz = (z - previous.z) / dt
            self.observations[name] = current

    def _measure(self, observation: Observation) -> dict:
        dx, dy, dz = observation.x, observation.y, observation.z - 3.0
        true_range = math.sqrt(dx * dx + dy * dy + dz * dz)
        true_azimuth = math.atan2(dy, dx)
        true_elevation = math.atan2(dz, math.hypot(dx, dy))
        range_sigma = max(2.0, true_range * 0.0025)
        measured_range = max(0.0, true_range + self.random.gauss(0.0, range_sigma))
        measured_azimuth = true_azimuth + math.radians(self.random.gauss(0.0, 0.18))
        measured_elevation = true_elevation + math.radians(self.random.gauss(0.0, 0.25))
        horizontal = measured_range * math.cos(measured_elevation)
        east = horizontal * math.cos(measured_azimuth)
        north = horizontal * math.sin(measured_azimuth)
        up = measured_range * math.sin(measured_elevation) + 3.0
        unit = (dx / true_range, dy / true_range, dz / true_range)
        radial_speed = (
            observation.vx * unit[0]
            + observation.vy * unit[1]
            + observation.vz * unit[2]
            + self.random.gauss(0.0, 0.35)
        )
        latitude, longitude = coordinate_offset(
            self.terrain.center_latitude, self.terrain.center_longitude, east, north
        )
        return {
            "range_m": round(measured_range, 2),
            "azimuth_deg": round(math.degrees(measured_azimuth) % 360.0, 3),
            "elevation_deg": round(math.degrees(measured_elevation), 3),
            "radial_speed_m_s": round(radial_speed, 2),
            "latitude": round(latitude, 8),
            "longitude": round(longitude, 8),
            "altitude_msl_m": round(self.terrain.origin_elevation + up, 2),
        }

    def _detected(self, observation: Observation, nominal_range: float) -> bool:
        dx, dy, dz = observation.x, observation.y, observation.z - 3.0
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        elevation = math.degrees(math.atan2(dz, math.hypot(dx, dy)))
        if distance < MIN_RANGE_M or elevation < MIN_ELEVATION_DEG or elevation > MAX_ELEVATION_DEG:
            return False
        if not self.terrain.line_of_sight(observation):
            return False
        if distance <= nominal_range:
            return True
        if distance > nominal_range * 1.1:
            return False
        probability = 1.0 - (distance - nominal_range) / (nominal_range * 0.1)
        return self.random.random() < probability

    def _build_scan(self, now: float) -> tuple[bytes, int]:
        tracks = []
        seen: set[str] = set()
        for name, observation in sorted(self.observations.items()):
            profile = profile_for_model(name)
            if profile is None or not self._detected(observation, profile[1]):
                continue
            seen.add(name)
            track = self.tracks.get(name)
            if track is None:
                track = Track(self.next_track_id)
                self.next_track_id += 1
                self.tracks[name] = track
            track.hits += 1
            track.misses = 0
            track.confidence = min(0.99, 0.45 + 0.18 * track.hits)
            if track.hits < 2:
                continue
            measured = self._measure(observation)
            measured.update({
                "track_id": track.track_id,
                "state": "confirmed",
                "classification": profile[0],
                "confidence": round(track.confidence, 2),
            })
            track.last_measurement = dict(measured)
            tracks.append(measured)

        for name in list(self.tracks):
            if name in seen:
                continue
            track = self.tracks[name]
            track.misses += 1
            track.confidence = max(0.0, track.confidence - 0.22)
            if track.misses > 3:
                del self.tracks[name]
            elif track.hits >= 2 and track.last_measurement is not None:
                coasted = dict(track.last_measurement)
                coasted["state"] = "coasting"
                coasted["confidence"] = round(track.confidence, 2)
                tracks.append(coasted)

        self.sequence += 1
        payload = {
            "type": "radar_tracks",
            "source": "radar_center",
            "schema_version": 1,
            "timestamp_ms": int(time.time() * 1000),
            "scan_sequence": self.sequence,
            "scan_rate_hz": self.rate_hz,
            "radar": {
                "latitude": self.terrain.center_latitude,
                "longitude": self.terrain.center_longitude,
                "altitude_msl_m": self.terrain.origin_elevation + 3.0,
            },
            "tracks": tracks,
        }
        return json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode(), len(tracks)

    def run(self) -> None:
        self.publish_status("idle", "等待启动雷达")
        while not self.shutdown_event.wait(0.02):
            now = time.monotonic()
            with self.lock:
                if self.running and now - self.last_scan >= 1.0 / self.rate_hz:
                    packet, count = self._build_scan(now)
                    delay = 0.12 + self.random.uniform(0.0, 0.08)
                    self.pending.append((now + delay, packet, count))
                    self.last_scan = now
                ready = []
                if self.running and self.station is not None:
                    while self.pending and self.pending[0][0] <= now:
                        ready.append((self.pending.pop(0), self.station))
            for (_deadline, packet, count), station in ready:
                self.socket.sendto(packet, station)
                if self.sequence % max(1, int(round(self.rate_hz * 5))) == 0:
                    self.publish_status("running", f"雷达运行中，当前航迹 {count} 条", count)

    def shutdown(self) -> None:
        self.shutdown_event.set()
        self.socket.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", required=True)
    parser.add_argument("--metadata", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=20261017)
    args = parser.parse_args()
    manager = RadarManager(args)

    def request_shutdown(_signum, _frame):
        manager.shutdown_event.set()

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    print(
        f"雷达管理服务已就绪：command={COMMAND_TOPIC} status={STATUS_TOPIC} "
        f"pose={manager.pose_topic} default_rate=1Hz",
        flush=True,
    )
    try:
        manager.run()
    finally:
        manager.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
