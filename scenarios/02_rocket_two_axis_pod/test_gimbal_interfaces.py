#!/usr/bin/env python3
"""验证已运行场景的逐机吊仓身份、双轴控制及多机控制隔离；结束时回零。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time


def wait_for(predicate, label: str, timeout: float = 8.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(0.02)
    raise RuntimeError(f"{label}超时")


def angles(client):
    attitude = client.snapshot()["attitude_deg"]
    return {axis: float(attitude[axis]["pos"]) for axis in ("yaw", "pitch")}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("-n", type=int, default=1)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--assets-root", type=Path, default=Path(__file__).resolve().parents[3]
                        / "px4-gazebo-assets")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not 1 <= args.n <= 10:
        parser.error("-n 必须为 1～10")
    sys.path.insert(0, str(args.assets_root.resolve()
                          / "models/gimbals/two_axis_pod/ground_station"))
    from ground_station import UdpClient

    clients = [UdpClient(args.host, 8770 + index, aircraft_id=index + 1)
               for index in range(args.n)]
    results = []

    def home(client):
        seq = client.send_reliable("home")
        if seq < 0:
            raise RuntimeError("网关未接受回零请求")
        reply = wait_for(lambda: client.find_result(seq=seq), "回零回执")
        if not reply.get("ok"):
            raise RuntimeError(f"吊仓回零失败：{reply}")
        def settled():
            attitude = client.snapshot()["attitude_deg"]
            return all(abs(attitude[axis]["pos"]) < 0.1
                       and abs(attitude[axis]["vel"]) < 0.25
                       for axis in ("yaw", "pitch"))
        wait_for(settled, "吊仓回零并静止")

    try:
        for client in clients:
            client.start()
        for index, client in enumerate(clients):
            wait_for(lambda: client.public_angles_ready, f"飞机{index + 1}身份与遥测")
            expected = f"/two_axis_pod_{index + 1}"
            if client.hello.get("video_rtsp_path") != expected:
                raise RuntimeError(f"飞机{index + 1}视频路径与身份不匹配")
            if client.snapshot().get("mount", {}).get("static") is not False:
                raise RuntimeError(f"飞机{index + 1}吊仓没有使用动态安装姿态")
            home(client)

        for active, client in enumerate(clients):
            before = [angles(item) for item in clients]
            try:
                deadline = time.monotonic() + 1.2
                while time.monotonic() < deadline:
                    if client.send_manual(yaw=12.0, pitch=8.0) < 0:
                        raise RuntimeError("双轴速度指令被拒绝")
                    time.sleep(0.1)
            finally:
                client.send_manual()
            time.sleep(0.3)
            after = [angles(item) for item in clients]
            changes = [{axis: end[axis] - start[axis] for axis in ("yaw", "pitch")}
                       for start, end in zip(before, after)]
            if changes[active]["yaw"] < 6 or changes[active]["pitch"] < 3:
                raise RuntimeError(f"飞机{active + 1}双轴没有按指令转动：{changes}")
            for other, delta in enumerate(changes):
                if other != active and any(abs(value) > 0.75 for value in delta.values()):
                    raise RuntimeError(f"控制飞机{active + 1}影响了飞机{other + 1}：{changes}")
            results.append({"aircraft_id": active + 1, "udp_port": 8770 + active,
                            "video_path": client.hello["video_rtsp_path"],
                            "all_aircraft_angle_changes_deg": changes, "passed": True})
            home(client)
            print(f"PASS: 飞机{active + 1}身份、双轴控制及其他飞机隔离", flush=True)
    finally:
        for client in clients:
            if client.public_angles_ready:
                client.send_manual()
                client.send_reliable("home")
        time.sleep(0.3)
        for client in clients:
            client.stop()
    report = {"passed": True, "aircraft": results}
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
