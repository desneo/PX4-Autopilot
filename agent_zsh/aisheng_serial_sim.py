#!/usr/bin/env python3
"""通过 USB 转串口持续发送艾晟 DYT 104-byte 遥测报文。"""

import argparse
from pathlib import Path
import struct
import sys
import time


FRAME_LENGTH = 104
DEFAULT_BAUD = 230400
DEFAULT_RATE_HZ = 60.0


def build_frame(sequence: int) -> bytes:
    frame = bytearray(FRAME_LENGTH)
    frame[:5] = bytes((0x55, 0xAA, 0x68, 0x01, 0x11))

    gate_x = 900 + sequence % 101
    gate_y = 500 + sequence % 61
    struct.pack_into("<hhhhhhhh", frame, 5,
                     125, -250, gate_x, gate_y, 10, -20, 5, -5)
    frame[21] = 95
    struct.pack_into("<hh", frame, 22, 15, -25)
    frame[26:28] = bytes((0x01, 0x01))

    for target in range(4):
        struct.pack_into("<BBiiH", frame, 28 + target * 12,
                         target + 1, 90 - target * 10,
                         1163970000 + target, 399070000 + target, 1200 + target)

    frame[76] = 0x01
    struct.pack_into("<HHhhhBH", frame, 77, 90, 98, 100, -200, 300, 1, 250)
    frame[90] = 0x01
    struct.pack_into("<hhHBBI", frame, 91, 50, -75, 9000, 8, 2,
                     sequence & 0xFFFFFFFF)
    frame[103] = sum(frame[3:103]) & 0xFF
    return bytes(frame)


def find_default_port() -> str:
    by_id = sorted(Path("/dev/serial/by-id").glob("usb-FTDI_*"))
    candidates = by_id or sorted(Path("/dev").glob("ttyUSB*"))

    if len(candidates) == 1:
        return str(candidates[0])

    if not candidates:
        raise RuntimeError("未找到 USB 转串口设备，请使用 --port 指定设备")

    raise RuntimeError("检测到多个 USB 转串口设备，请使用 --port 指定设备")


def send_frames(port: str, rate_hz: float, count: int) -> None:
    try:
        import serial
    except ImportError as error:
        raise RuntimeError("缺少 pyserial，请先安装 python3-serial") from error

    period = 1.0 / rate_hz
    sent = 0
    started = time.monotonic()
    next_send = started

    with serial.Serial(port=port, baudrate=DEFAULT_BAUD, bytesize=serial.EIGHTBITS,
                       parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                       timeout=0, write_timeout=1) as device:
        print(f"开始发送：port={port} baud={DEFAULT_BAUD} rate={rate_hz:g}Hz")

        while count == 0 or sent < count:
            device.write(build_frame(sent))
            sent += 1
            next_send += period

            if sent == 1:
                print(f"已发送首帧，{FRAME_LENGTH} bytes")
            elif sent % max(1, round(rate_hz)) == 0:
                elapsed = max(time.monotonic() - started, 1e-6)
                print(f"已发送 {sent} 帧，{sent * FRAME_LENGTH} bytes，平均 {sent / elapsed:.1f}Hz")

            delay = next_send - time.monotonic()

            if delay > 0:
                time.sleep(delay)
            elif delay < -period:
                next_send = time.monotonic()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="模拟艾晟 DYT，通过 USB 转串口发送 104-byte 遥测报文")
    parser.add_argument("--port", help="串口设备；缺省时自动选择唯一 FTDI/ttyUSB 设备")
    parser.add_argument("--rate-hz", type=float, default=DEFAULT_RATE_HZ, help="发送频率，默认 60")
    parser.add_argument("--count", type=int, default=0, help="发送帧数，0 表示持续发送")
    parser.add_argument("--dry-run", action="store_true", help="只输出一帧十六进制数据，不访问串口")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if not 0 < args.rate_hz <= 1000:
        print("错误：--rate-hz 必须大于 0 且不超过 1000", file=sys.stderr)
        return 2

    if args.count < 0:
        print("错误：--count 不能为负数", file=sys.stderr)
        return 2

    if args.dry_run:
        print(build_frame(0).hex(" "))
        return 0

    try:
        send_frames(args.port or find_default_port(), args.rate_hz, args.count)
    except KeyboardInterrupt:
        print("\n已停止发送")
    except (OSError, RuntimeError) as error:
        print(f"错误：{error}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
