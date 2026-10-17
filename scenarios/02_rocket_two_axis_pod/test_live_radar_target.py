#!/usr/bin/env python3
"""对已运行的场景 02 验证雷达航迹与靶机真值共用 UDP 地址。"""

from __future__ import annotations

import argparse
import json
import socket
import time


RADAR_COMMAND = "/scenario/02/radar/command"
TARGET_COMMAND = "/scenario/02/target/command"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--altitude", type=float, default=104.02)
    args = parser.parse_args()
    from gz.msgs10.stringmsg_pb2 import StringMsg
    from gz.transport13 import Node

    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("127.0.0.1", 0))
    receiver.settimeout(0.5)
    station = f"127.0.0.1:{receiver.getsockname()[1]}"
    node = Node()
    radar = node.advertise(RADAR_COMMAND, StringMsg)
    target = node.advertise(TARGET_COMMAND, StringMsg)
    if not radar.valid() or not target.valid():
        raise RuntimeError("无法创建场景命令发布器")

    def publish(publisher, payload: dict) -> None:
        message = StringMsg()
        message.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        if not publisher.publish(message):
            raise RuntimeError("发送场景测试命令失败")

    time.sleep(0.5)
    publish(radar, {"command": "start", "rate_hz": 1.0, "station_address": station})
    publish(target, {
        "command": "start",
        "model": "xiaobianfu",
        "speed": 18.0,
        "altitude": args.altitude,
        "distance": 2000.0,
        "direction": "east_west",
        "station_address": station,
    })
    packets: dict[str, list[dict]] = {"target_truth": [], "radar_tracks": []}
    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline:
            try:
                data, _address = receiver.recvfrom(65535)
            except socket.timeout:
                continue
            value = json.loads(data)
            packet_type = value.get("type")
            if packet_type in packets:
                packets[packet_type].append(value)
    finally:
        publish(target, {"command": "stop"})
        publish(radar, {"command": "stop"})
        receiver.close()

    truth = packets["target_truth"]
    scans = packets["radar_tracks"]
    if len(truth) < max(10, int(args.duration * 5)):
        raise RuntimeError(f"靶机真值频率不足：{len(truth)} packets/{args.duration:g}s")
    if len(scans) < max(2, int(args.duration) - 2):
        raise RuntimeError(f"雷达航迹频率不足：{len(scans)} packets/{args.duration:g}s")
    tracked = [scan for scan in scans if scan.get("tracks")]
    if not tracked:
        raise RuntimeError("雷达未形成靶机确认航迹")
    radar_text = json.dumps(tracked, ensure_ascii=False)
    if "xiaobianfu" in radar_text or "rocket_two_axis_pod" in radar_text:
        raise RuntimeError("雷达输出泄露了 Gazebo 模型身份")
    if any(value.get("source") != "simulation_truth" for value in truth):
        raise RuntimeError("靶机真值 source 标记错误")
    if any(value.get("source") != "radar_center" for value in scans):
        raise RuntimeError("雷达航迹 source 标记错误")
    print(
        f"正式场景联合测试通过：truth={len(truth)}，radar={len(scans)}，"
        f"tracked_scans={len(tracked)}，station={station}"
    )
    print(json.dumps(tracked[-1], ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
