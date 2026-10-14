#!/usr/bin/env python3
"""Hardware regression test for DYT gimbal commands on TELEM2.

The flight controller USB port is used for a MAVLink NSH session and a USB
serial adapter captures TELEM2 TX. The test switches the selected DYT brand,
sends one common uORB rate command through serial_probe, validates the vendor
frames, sends stop, and verifies that periodic transmission ceases.

Example:
  Tools/dyt_gimbal_hw_test.py \
      /dev/serial/by-id/usb-CUAV_PX4_CUAV_7_Nano_0-if00 \
      /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AN0M57RG-if00-port0

The original DYT_BRAND value is restored and saved even when a test fails.
The gimbal command is always left stopped.
"""

import argparse
import json
import re
import statistics
import sys
import threading
import time
from pathlib import Path

import serial
from pymavlink import mavutil


BRANDS = {
    "zhongke": {
        "value": 0, "baud": 115200, "command": 0x24, "repeat": 1,
        "status_name": "zhongkeyoucheng",
    },
    "aisheng": {
        "value": 1, "baud": 230400, "command": 0xB1, "repeat": 3,
        "status_name": "aisheng",
    },
}
SHELL_DEVICE = 10
SHELL_FLAGS = (mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE
               | mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND)


class NshClient:
    def __init__(self, port):
        self.connection = mavutil.mavlink_connection(
            port, baud=57600, autoreconnect=False)

        try:
            self._heartbeat()

            if self.connection.wait_heartbeat(timeout=8) is None:
                raise RuntimeError("PX4 heartbeat timeout")

            self._send("\n")
            self._receive_prompt(2.0)
        except Exception:
            self.close()
            raise

    def _heartbeat(self):
        self.connection.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GENERIC,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def _send(self, text, flags=SHELL_FLAGS):
        payload = text.encode("utf-8")

        while payload:
            chunk, payload = payload[:70], payload[70:]
            data = list(chunk) + [0] * (70 - len(chunk))
            self.connection.mav.serial_control_send(
                SHELL_DEVICE, flags, 0, 0, len(chunk), data)

    def _receive_prompt(self, timeout):
        deadline = time.monotonic() + timeout
        next_heartbeat = 0.0
        output = bytearray()

        while time.monotonic() < deadline:
            now = time.monotonic()

            if now >= next_heartbeat:
                self._heartbeat()
                next_heartbeat = now + 0.5

            message = self.connection.recv_match(
                type="SERIAL_CONTROL", blocking=True, timeout=0.1)

            if message is None or message.count == 0:
                continue

            output.extend(bytes(message.data[:message.count]))

            if b"nsh>" in output:
                return output.decode("utf-8", errors="replace")

        raise RuntimeError("NSH prompt timeout")

    def command(self, command, timeout=4.0):
        self._send(command + "\n")
        return self._receive_prompt(timeout)

    def close(self):
        try:
            self.connection.mav.serial_control_send(
                SHELL_DEVICE, 0, 0, 0, 0, [0] * 70)
        except Exception:
            pass
        finally:
            self.connection.close()


class ByteReader(threading.Thread):
    def __init__(self, port, baud):
        super().__init__(daemon=True)
        self.serial = serial.Serial(port, baudrate=baud, timeout=0.02)
        self.samples = []
        self._running = threading.Event()
        self._running.set()

    def run(self):
        self.serial.reset_input_buffer()

        while self._running.is_set():
            value = self.serial.read(1)

            if value:
                self.samples.append((time.monotonic(), value[0]))

    def finish(self):
        self._running.clear()
        self.join(timeout=1.0)
        self.serial.close()


def signed16(low, high):
    value = low | (high << 8)
    return value - 65536 if value >= 32768 else value


def parse_frames(samples, brand):
    data = bytes(value for _, value in samples)
    times = [stamp for stamp, _ in samples]
    header = b"\xeb\x90" if brand == "zhongke" else b"\xaa\x55"
    length = 16 if brand == "zhongke" else 45
    frames = []
    cursor = 0

    while cursor + length <= len(data):
        start = data.find(header, cursor)

        if start < 0 or start + length > len(data):
            break

        frame = data[start:start + length]

        if brand == "zhongke":
            valid = (sum(frame[:15]) & 0xff) == frame[15]
            command = frame[2]
            first = signed16(frame[3], frame[4])
            second = signed16(frame[5], frame[6])

        else:
            valid = (frame[2] == length
                     and (sum(frame[3:44]) & 0xff) == frame[44])
            command = frame[3]
            first = signed16(frame[4], frame[5])
            second = signed16(frame[6], frame[7])

        if valid:
            frames.append({
                "time": times[start],
                "command": command,
                "param1": first,
                "param2": second,
                "hex": frame.hex(" "),
            })
            cursor = start + length

        else:
            cursor = start + 1

    return frames


def read_brand(client):
    output = client.command("param show DYT_BRAND")

    for line in output.splitlines():
        if "DYT_BRAND" not in line:
            continue

        match = re.search(r":\s*(-?\d+)\s*$", line)

        if match:
            return int(match.group(1))

    raise RuntimeError("could not parse DYT_BRAND")


def select_brand(client, brand):
    config = BRANDS[brand]
    client.command("serial_probe gimbal stop")
    client.command("param set DYT_BRAND {}".format(config["value"]))
    client.command("serial_probe stop")
    start_output = client.command("serial_probe start")
    expected = "parsing {} at {}".format(
        config["status_name"], config["baud"])

    if expected not in start_output:
        raise RuntimeError(
            "serial_probe did not start as {}: {}".format(
                brand, start_output.strip()))


def exercise(client, serial_port, brand, yaw_rate, pitch_rate,
             motion_seconds, quiet_seconds):
    config = BRANDS[brand]
    reader = ByteReader(serial_port, config["baud"])
    reader.start()
    time.sleep(0.1)
    stopped = False

    try:
        client.command(
            "serial_probe gimbal {} {}".format(yaw_rate, pitch_rate))
        time.sleep(motion_seconds)
        stop_started = time.monotonic()
        client.command("serial_probe gimbal stop")
        stopped = True
        stop_finished = time.monotonic()
        time.sleep(quiet_seconds)

    finally:
        try:
            if not stopped:
                client.command("serial_probe gimbal stop")
        finally:
            reader.finish()

    frames = parse_frames(reader.samples, brand)
    command_frames = [
        frame for frame in frames if frame["command"] == config["command"]
    ]

    if not command_frames:
        raise RuntimeError("no valid {} motion frames captured".format(brand))

    zero_frames = [
        frame for frame in command_frames
        if frame["param1"] == 0 and frame["param2"] == 0
    ]
    nonzero_frames = [
        frame for frame in command_frames
        if frame["param1"] != 0 or frame["param2"] != 0
    ]

    if not nonzero_frames:
        raise RuntimeError("no nonzero {} motion frames captured".format(brand))

    if len(zero_frames) < config["repeat"]:
        raise RuntimeError("incomplete {} stop transmission".format(brand))

    first_zero_time = zero_frames[0]["time"]
    nonzero_after_stop = [
        frame for frame in command_frames
        if frame["time"] > first_zero_time
        and (frame["param1"] != 0 or frame["param2"] != 0)
    ]
    frames_after_quiet_boundary = [
        frame for frame in command_frames
        if frame["time"] > stop_finished + 0.05
    ]

    if nonzero_after_stop:
        raise RuntimeError("{} emitted nonzero motion after stop".format(brand))

    if frames_after_quiet_boundary:
        raise RuntimeError("{} keepalive continued after stop".format(brand))

    repeat = config["repeat"]

    if len(nonzero_frames) % repeat != 0:
        raise RuntimeError("{} command frame repeat count is invalid".format(brand))

    group_times = [frame["time"] for frame in nonzero_frames[::repeat]]
    intervals_ms = [
        (later - earlier) * 1000
        for earlier, later in zip(group_times, group_times[1:])
    ]

    if not intervals_ms:
        raise RuntimeError("not enough {} keepalive groups".format(brand))

    median_interval_ms = statistics.median(intervals_ms)

    if median_interval_ms < 70 or median_interval_ms > 140:
        raise RuntimeError(
            "{} keepalive median {:.1f} ms is outside tolerance".format(
                brand, median_interval_ms))

    distinct_values = []

    for frame in nonzero_frames[::repeat]:
        values = [frame["param1"], frame["param2"]]

        if not distinct_values or values != distinct_values[-1]:
            distinct_values.append(values)

    return {
        "brand": brand,
        "baud": config["baud"],
        "valid_command_frames": len(command_frames),
        "nonzero_frame_groups": len(nonzero_frames) // repeat,
        "stop_frame_groups": len(zero_frames) // repeat,
        "repeat_per_command": repeat,
        "distinct_motion_values": distinct_values,
        "keepalive_intervals_ms": [round(value, 1) for value in intervals_ms],
        "median_keepalive_ms": round(median_interval_ms, 1),
        "stop_command_latency_ms": round(
            (first_zero_time - stop_started) * 1000, 1),
        "frames_after_stop_window": len(frames_after_quiet_boundary),
        "first_nonzero_hex": nonzero_frames[0]["hex"],
        "last_nonzero_hex": nonzero_frames[-1]["hex"],
        "first_stop_hex": zero_frames[0]["hex"],
        "result": "PASS",
    }


def restore_brand(client, original_brand):
    brand_name = next(
        name for name, config in BRANDS.items()
        if config["value"] == original_brand)
    select_brand(client, brand_name)
    client.command("param save")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("shell_port", help="Nano7 USB MAVLink serial device")
    parser.add_argument("serial_port", help="USB adapter connected to TELEM2 TX")
    parser.add_argument(
        "--brand", choices=["all", "zhongke", "aisheng"], default="all")
    parser.add_argument("--yaw-rate", type=float, default=4.0, metavar="DEG_S")
    parser.add_argument("--pitch-rate", type=float, default=-2.0, metavar="DEG_S")
    parser.add_argument("--motion-seconds", type=float, default=0.65)
    parser.add_argument("--quiet-seconds", type=float, default=0.35)
    parser.add_argument("--output", type=Path, help="optional JSON result path")
    args = parser.parse_args()

    if args.motion_seconds < 0.3:
        parser.error("--motion-seconds must be at least 0.3")

    if args.quiet_seconds < 0.2:
        parser.error("--quiet-seconds must be at least 0.2")

    brands = list(BRANDS) if args.brand == "all" else [args.brand]
    client = NshClient(args.shell_port)
    original_brand = None
    results = []

    try:
        original_brand = read_brand(client)

        if original_brand not in (0, 1):
            raise RuntimeError(
                "unsupported original DYT_BRAND {}".format(original_brand))

        for brand in brands:
            select_brand(client, brand)
            results.append(exercise(
                client, args.serial_port, brand,
                args.yaw_rate, args.pitch_rate,
                args.motion_seconds, args.quiet_seconds))

    finally:
        try:
            if original_brand is not None:
                restore_brand(client, original_brand)
            else:
                try:
                    client.command("serial_probe gimbal stop")
                except Exception:
                    pass
        finally:
            client.close()

    report = {
        "shell_port": args.shell_port,
        "serial_port": args.serial_port,
        "yaw_rate_deg_s": args.yaw_rate,
        "pitch_rate_deg_s": args.pitch_rate,
        "original_brand": original_brand,
        "restored_brand": original_brand,
        "tests": results,
        "result": "PASS",
    }
    encoded = json.dumps(report, indent=2, ensure_ascii=False) + "\n"

    if args.output:
        args.output.write_text(encoded, encoding="utf-8")

    sys.stdout.write(encoded)


if __name__ == "__main__":
    main()
