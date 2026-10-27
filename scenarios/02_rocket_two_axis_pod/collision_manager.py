#!/usr/bin/env python3
"""场景 02 碰撞致损服务：记录碰撞并关闭对应 PX4 的全部电机。"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import queue
import re
import signal
import subprocess
import threading
import time


COLLISION_TOPIC = "/scenario/02/collision/event"
NORMAL_LANDING_NAV_STATES = {18, 20}  # AUTO_LAND、AUTO_PRECLAND
AUTO_RTL_NAV_STATE = 5
LAND_SETPOINT_TYPE = 4
EARTH_RADIUS_M = 6_371_000.0


def coordinate_offset(
    latitude_deg: float, longitude_deg: float, east_m: float, north_m: float
) -> tuple[float, float]:
    latitude = latitude_deg + math.degrees(north_m / EARTH_RADIUS_M)
    longitude = longitude_deg + math.degrees(
        east_m / (EARTH_RADIUS_M * math.cos(math.radians(latitude_deg)))
    )
    return latitude, longitude


def parse_nav_state(output: str) -> int | None:
    match = re.search(r"(?m)^\s*nav_state:\s*(\d+)\s*$", output)
    return int(match.group(1)) if match else None


def parse_current_setpoint(output: str) -> dict | None:
    # listener 分别打印 previous / current / next，不能把下一航点的 LAND
    # 当成已进入降落阶段；只读取当前有效设定点。
    section = re.search(
        r"(?ms)^[ \t]*current[ \t]+\(position_setpoint\):[ \t]*\n"
        r"(.*?)(?=^[ \t]*(?:previous|next)[ \t]+\(position_setpoint\):|\Z)",
        output,
    )
    if not section:
        return None
    valid = re.search(r"(?m)^[ \t]*valid:[ \t]*(True|False|true|false)[ \t]*$", section[1])
    kind = re.search(r"(?m)^[ \t]*type:[ \t]*(\d+)[ \t]*$", section[1])
    if not valid or not kind:
        return None
    return {"valid": valid[1].lower() == "true", "type": int(kind[1])}


def validate_event(payload: object, aircraft_count: int) -> dict:
    if not isinstance(payload, dict):
        raise ValueError("碰撞事件必须是 JSON 对象")
    collision_type = payload.get("collision_type")
    if collision_type not in {"px4_px4", "px4_surface", "px4_target"}:
        raise ValueError("未知碰撞类型")
    ids = payload.get("aircraft_ids")
    if not isinstance(ids, list) or not ids:
        raise ValueError("碰撞事件缺少 aircraft_ids")
    if any(
        isinstance(value, bool)
        or not isinstance(value, int)
        or not 1 <= value <= aircraft_count
        for value in ids
    ):
        raise ValueError("碰撞事件中的飞机 ID 超出场景范围")
    expected = 2 if collision_type == "px4_px4" else 1
    if len(set(ids)) != expected:
        raise ValueError(f"{collision_type} 需要 {expected} 个不同的飞机 ID")
    try:
        relative_speed = float(payload["relative_speed_m_s"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("碰撞事件缺少有效相对速度") from error
    if not math.isfinite(relative_speed) or relative_speed < 0:
        raise ValueError("碰撞相对速度必须是有限非负数")
    result = dict(payload)
    result["aircraft_ids"] = sorted(set(ids))
    result["relative_speed_m_s"] = relative_speed
    return result


class Px4FailureController:
    def __init__(self, px4_bin: Path):
        self.px4_bin = px4_bin.resolve()
        self.listener_bin = self.px4_bin.with_name("px4-listener")
        self.failure_bin = self.px4_bin.with_name("px4-failure")

    @staticmethod
    def instance(aircraft_id: int) -> int:
        return aircraft_id - 1

    def _listen(self, aircraft_id: int, topic: str) -> str | None:
        try:
            result = subprocess.run(
                [
                    str(self.listener_bin),
                    "--instance",
                    str(self.instance(aircraft_id)),
                    topic,
                    "-n",
                    "1",
                ],
                capture_output=True,
                text=True,
                timeout=3,
            )
        except (OSError, subprocess.TimeoutExpired):
            return None
        if result.returncode != 0:
            return None
        return result.stdout + "\n" + result.stderr

    def navigation_state(self, aircraft_id: int) -> int | None:
        output = self._listen(aircraft_id, "vehicle_status")
        return parse_nav_state(output) if output is not None else None

    def current_setpoint(self, aircraft_id: int) -> dict | None:
        output = self._listen(aircraft_id, "position_setpoint_triplet")
        return parse_current_setpoint(output) if output is not None else None

    def stop_all_motors(self, aircraft_id: int) -> tuple[bool, str]:
        command = [
            str(self.failure_bin),
            "--instance",
            str(self.instance(aircraft_id)),
            "motor",
            "off",
            "-i",
            "0",
        ]
        last_output = ""
        for attempt in range(1, 4):
            try:
                result = subprocess.run(
                    command, capture_output=True, text=True, timeout=5
                )
                last_output = (result.stdout + "\n" + result.stderr).strip()
                if result.returncode == 0:
                    return True, last_output
            except (OSError, subprocess.TimeoutExpired) as error:
                last_output = str(error)
            if attempt < 3:
                time.sleep(0.1)
        return False, last_output


class CollisionProcessor:
    def __init__(
        self,
        aircraft_count: int,
        controller: Px4FailureController,
        event_log: Path,
        center_latitude: float,
        center_longitude: float,
        origin_elevation: float,
        enable_aircraft_collision: bool = False,
    ):
        self.aircraft_count = aircraft_count
        self.controller = controller
        self.event_log = event_log
        self.center_latitude = center_latitude
        self.center_longitude = center_longitude
        self.origin_elevation = origin_elevation
        self.enable_aircraft_collision = enable_aircraft_collision
        self.damaged: set[int] = set()
        self.event_log.parent.mkdir(parents=True, exist_ok=True)

    def _add_global_position(self, event: dict) -> None:
        position = event.get("position_enu_m")
        if not isinstance(position, dict):
            return
        try:
            east = float(position["x"])
            north = float(position["y"])
            up = float(position["z"])
        except (KeyError, TypeError, ValueError):
            return
        if not all(math.isfinite(value) for value in (east, north, up)):
            return
        latitude, longitude = coordinate_offset(
            self.center_latitude, self.center_longitude, east, north
        )
        event["position_wgs84"] = {
            "latitude": latitude,
            "longitude": longitude,
            "altitude_msl_m": self.origin_elevation + up,
        }

    def _write(self, event: dict) -> None:
        with self.event_log.open("a", encoding="utf-8") as stream:
            stream.write(
                json.dumps(event, ensure_ascii=False, separators=(",", ":")) + "\n"
            )

    def process(self, payload: object) -> dict:
        event = validate_event(payload, self.aircraft_count)
        event["processed_timestamp_ms"] = int(time.time() * 1000)
        self._add_global_position(event)
        collision_type = event["collision_type"]
        candidates = list(event["aircraft_ids"])

        if collision_type == "px4_px4" and not self.enable_aircraft_collision:
            event["damage_triggered"] = False
            event["reason"] = "aircraft_collision_disabled"
            event["damage_results"] = []
            self._write(event)
            return event

        if collision_type == "px4_surface":
            nav_state = self.controller.navigation_state(candidates[0])
            event["navigation_state"] = nav_state
            rtl_landing = False
            if nav_state == AUTO_RTL_NAV_STATE:
                setpoint = self.controller.current_setpoint(candidates[0])
                event["current_position_setpoint"] = setpoint
                rtl_landing = bool(
                    setpoint is not None
                    and setpoint["valid"]
                    and setpoint["type"] == LAND_SETPOINT_TYPE
                )
            if nav_state in NORMAL_LANDING_NAV_STATES or rtl_landing:
                event["damage_triggered"] = False
                event["reason"] = "rtl_landing_setpoint" if rtl_landing else "normal_landing_mode"
                event["damage_results"] = []
                self._write(event)
                return event

        results = []
        for aircraft_id in candidates:
            if aircraft_id in self.damaged:
                results.append(
                    {
                        "aircraft_id": aircraft_id,
                        "success": True,
                        "already_damaged": True,
                    }
                )
                continue
            success, output = self.controller.stop_all_motors(aircraft_id)
            results.append(
                {
                    "aircraft_id": aircraft_id,
                    "success": success,
                    "already_damaged": False,
                    "detail": output[-500:],
                }
            )
            if success:
                self.damaged.add(aircraft_id)

        event["damage_triggered"] = True
        event["damage_action"] = "all_motors_off"
        event["damage_results"] = results
        event["reason"] = "collision_damage_policy"
        self._write(event)
        return event


class CollisionManager:
    def __init__(self, args):
        from gz.msgs10.stringmsg_pb2 import StringMsg
        from gz.transport13 import Node

        metadata = json.loads(args.metadata.read_text())
        center = metadata["center_wgs84"]
        self.processor = CollisionProcessor(
            args.aircraft_count,
            Px4FailureController(args.px4_bin),
            args.event_log,
            float(center["latitude_deg"]),
            float(center["longitude_deg"]),
            float(metadata["origin_elevation_source_m"]),
            args.enable_collision,
        )
        self.events: queue.Queue[dict] = queue.Queue()
        self.shutdown_event = threading.Event()
        self.node = Node()
        if not self.node.subscribe(StringMsg, COLLISION_TOPIC, self.on_event):
            raise RuntimeError(f"无法订阅碰撞事件话题：{COLLISION_TOPIC}")

    def on_event(self, message) -> None:
        try:
            payload = json.loads(message.data)
            self.events.put_nowait(payload)
        except (json.JSONDecodeError, queue.Full) as error:
            print(f"忽略无效碰撞事件：{error}", flush=True)

    def run(self) -> None:
        while not self.shutdown_event.is_set():
            try:
                event = self.events.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                result = self.processor.process(event)
                print(
                    json.dumps(result, ensure_ascii=False, separators=(",", ":")),
                    flush=True,
                )
            except ValueError as error:
                print(f"忽略无效碰撞事件：{error}", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--aircraft-count", type=int, required=True)
    parser.add_argument("--px4-bin", type=Path, required=True)
    parser.add_argument("--metadata", type=Path, required=True)
    parser.add_argument("--event-log", type=Path, required=True)
    parser.add_argument(
        "--enable-collision",
        action="store_true",
        help="启用飞机之间的互相致损；默认忽略 px4_px4 事件",
    )
    args = parser.parse_args()
    if args.aircraft_count < 1:
        parser.error("--aircraft-count 必须是正整数")
    manager = CollisionManager(args)

    def request_shutdown(_signum, _frame):
        manager.shutdown_event.set()

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    print(
        f"碰撞致损服务已就绪：topic={COLLISION_TOPIC} "
        f"aircraft={args.aircraft_count} "
        f"aircraft_collision={'enabled' if args.enable_collision else 'disabled'} "
        "surface_target_policy=damage-on-contact",
        flush=True,
    )
    manager.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
