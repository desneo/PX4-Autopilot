#!/usr/bin/env python3
"""用独立 PX4 实例验证真实 UDP→uORB；只启动遥测桥，不启动仿真或飞行模块。"""
import json
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import tempfile
import time
import unittest


REPOSITORY = Path(__file__).resolve().parents[4]
BIN = REPOSITORY / 'build/px4_sitl_default/bin'


def packet(offset_x):
    return dict(timestamp=1793606400123, status1=0x24, status2=0x81, status3=0x12,
                zoom=2.5, offset_x=offset_x, offset_y=-3.25, frame_roll=1.5,
                frame_pitch=-12.5, frame_yaw=45.25, roll_rate=-0.75,
                pitch_rate=2.25, yaw_rate=3.5, laser_range=123.5, self_check=0x80)


@unittest.skipUnless((BIN / 'px4-zhongke_udp_bridge').exists(),
                     '请先运行 make px4_sitl_default')
class SitlUdpTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='px4-zhongke-udp-')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.processes = []
        self.addCleanup(self.stop_processes)
        startup = self.root / 'startup.sh'
        startup.write_text('#!/bin/sh\nexit 0\n')
        self.instances = []
        self.ports = []
        self.logs = []
        reservations = []
        self.addCleanup(lambda: [sock.close() for sock in reservations])
        for _ in range(2):
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(('127.0.0.1', 0))
            reservations.append(sock)
            self.ports.append(sock.getsockname()[1])
        candidate = 200 + os.getpid() % 10000
        for index in range(2):
            while Path(f'/tmp/px4_lock-{candidate}').exists():
                candidate += 1
            instance = candidate
            candidate += 1
            self.instances.append(instance)
            work = self.root / str(instance)
            work.mkdir()
            log = self.root / f'{instance}.log'
            self.logs.append(log)
            with log.open('w') as output:
                process = subprocess.Popen(
                    [str(BIN / 'px4'), '-d', '-i', str(instance), '-w', str(work),
                     '-s', str(startup)], stdin=subprocess.DEVNULL,
                    stdout=output, stderr=subprocess.STDOUT)
            self.processes.append(process)
            self.wait_for(lambda: 'Startup script returned successfully' in log.read_text(),
                          'PX4 启动未完成')
            reservations[index].close()
            self.command(index, 'zhongke_udp_bridge', 'start', '-p', str(self.ports[index]))
            self.wait_for(lambda: f'中科遥测接收已就绪 UDP 0.0.0.0:{self.ports[index]}' in log.read_text(),
                          '中科 UDP 桥未就绪')

    def stop_processes(self):
        for process in self.processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)

    def wait_for(self, predicate, message):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            if predicate():
                return
            self.assertTrue(all(process.poll() is None for process in self.processes),
                            '\n'.join(log.read_text() for log in self.logs))
            time.sleep(0.02)
        self.fail(message + '\n' + '\n'.join(log.read_text() for log in self.logs))

    def command(self, index, module, *args):
        result = subprocess.run(
            [str(BIN / f'px4-{module}'), '--instance', str(self.instances[index]), *args],
            stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        return result.stdout

    def counters(self, index):
        status = self.command(index, 'zhongke_udp_bridge', 'status')
        return {key: int(value) for key, value in re.findall(
            r'(接收|有效|解析错误|超长|发布)=(\d+)', status)}

    def send(self, index, payload):
        if isinstance(payload, dict):
            payload = json.dumps(payload, separators=(',', ':')).encode()
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.sendto(payload, ('127.0.0.1', self.ports[index]))

    def snapshot(self, index):
        return self.command(index, 'listener', 'dyt_zhongke_telemetry')

    def test_udp_publication_isolation_rejection_and_port_ownership(self):
        for index in range(2):
            self.assertIn('never published', self.snapshot(index))
            self.assertEqual(self.counters(index)['接收'], 0)
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
                with self.assertRaises(OSError):
                    probe.bind(('0.0.0.0', self.ports[index]))

        self.send(0, packet(7.5))
        self.wait_for(lambda: self.counters(0).get('发布') == 1, '未收到首机 uORB')
        snapshot = self.snapshot(0)
        for field, expected in packet(7.5).items():
            match = re.search(rf'^\s*{field}:\s*([-+0-9.eE]+)', snapshot, re.MULTILINE)
            self.assertIsNotNone(match, snapshot)
            actual = float(match.group(1))
            if field == 'timestamp':
                # 无仿真时钟推进时，lockstep 的 hrt 保持为 0。
                self.assertGreaterEqual(actual, 0)
                self.assertNotEqual(actual, expected)
            else:
                self.assertAlmostEqual(actual, expected, places=3, msg=field)
        self.assertIn('never published', self.snapshot(1))
        self.assertIn('never published', self.command(0, 'listener', 'dyt_aisheng_telemetry'))

        serialized = json.dumps(packet(7.5), separators=(',', ':')).encode()
        for invalid in (b'{}', b'{"status1":', b'',
                        serialized[:-1] + b',"status1":1}', serialized + b' ' * 4096):
            self.send(0, invalid)
        self.wait_for(lambda: self.counters(0).get('接收') == 6, '错误包计数未更新')
        self.assertEqual(self.counters(0), {'接收': 6, '有效': 1, '解析错误': 4, '超长': 1, '发布': 1})
        self.assertIn('offset_x: 7.500', self.snapshot(0))

        self.send(1, packet(-8.5))
        self.wait_for(lambda: self.counters(1).get('发布') == 1, '未收到第二架 uORB')
        self.assertIn('offset_x: -8.500', self.snapshot(1))
        self.assertIn('offset_x: 7.500', self.snapshot(0))
        self.assertEqual(self.counters(0)['发布'], 1)

        self.command(0, 'zhongke_udp_bridge', 'stop')
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.bind(('0.0.0.0', self.ports[0]))


if __name__ == '__main__':
    unittest.main()
