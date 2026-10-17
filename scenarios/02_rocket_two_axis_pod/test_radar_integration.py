#!/usr/bin/env python3
"""独立 Gazebo 分区中的雷达冒烟测试，不启动 PX4 或场景 02 正式实例。"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import socket
import subprocess
import tempfile
import time


WORLD = "scenario02_radar_test"
PARTITION = f"scenario02_radar_test_{os.getpid()}"


WORLD_SDF = f"""<?xml version='1.0'?>
<sdf version='1.10'>
  <world name='{WORLD}'>
    <plugin filename='gz-sim-physics-system' name='gz::sim::systems::Physics'/>
    <plugin filename='gz-sim-scene-broadcaster-system' name='gz::sim::systems::SceneBroadcaster'/>
    <gravity>0 0 0</gravity>
    <model name='xiaomotuo'>
      <pose>2000 0 100 0 0 0</pose>
      <link name='base_link'>
        <inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
      </link>
    </model>
  </world>
</sdf>
"""


def stop(process: subprocess.Popen | None) -> None:
    if process is None or process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--metadata", type=Path, required=True)
    args = parser.parse_args()
    env = os.environ.copy()
    env.update({"GZ_PARTITION": PARTITION, "GZ_IP": "127.0.0.1"})
    scenario_dir = Path(__file__).resolve().parent
    server = manager = None
    with tempfile.TemporaryDirectory() as directory:
        world = Path(directory) / "world.sdf"
        world.write_text(WORLD_SDF)
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(8)
        port = receiver.getsockname()[1]
        try:
            server = subprocess.Popen(
                ["gz", "sim", "-r", "-s", "-v1", str(world)],
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            manager = subprocess.Popen(
                [
                    "python3", "-u", str(scenario_dir / "radar_manager.py"),
                    "--world", WORLD, "--metadata", str(args.metadata),
                ],
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            time.sleep(1.5)
            command = json.dumps({
                "command": "start", "rate_hz": 2.0,
                "station_address": f"127.0.0.1:{port}",
            }, separators=(",", ":"))
            result = subprocess.run(
                [
                    "gz", "topic", "-t", "/scenario/02/radar/command",
                    "-m", "gz.msgs.StringMsg", "-p", f'data: "{command.replace(chr(34), chr(92) + chr(34))}"',
                ],
                env=env,
                capture_output=True,
                text=True,
                timeout=10,
            )
            if result.returncode != 0:
                raise RuntimeError(f"发送雷达启动命令失败：{result.stderr}")
            packets = []
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline:
                packet, _address = receiver.recvfrom(65535)
                payload = json.loads(packet)
                packets.append(payload)
                if payload.get("tracks"):
                    break
            if not packets or not packets[-1].get("tracks"):
                raise RuntimeError("在超时前未收到已确认雷达航迹")
            value = packets[-1]
            encoded = json.dumps(value, ensure_ascii=False)
            assert value["type"] == "radar_tracks"
            assert value["source"] == "radar_center"
            assert "xiaomotuo" not in encoded
            assert abs(value["tracks"][0]["range_m"] - 2000.0) < 100.0
            print(json.dumps(value, ensure_ascii=False, indent=2))
            print("雷达 Gazebo 冒烟测试通过")
        finally:
            receiver.close()
            stop(manager)
            stop(server)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
