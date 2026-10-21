#!/usr/bin/env python3
"""验证运行中多机吊舱身份、隔离、限位和独立光学/轴心真值；结束时回零。"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import threading
import time
import xml.etree.ElementTree as ET

LIMITS = {"yaw": (-60., 60.), "pitch": (-80., 30.)}
BALL_CENTER = (.015, 0., .056)
SENSOR_OFFSET = .0237


def wait_for(predicate, label: str, timeout: float = 12.0):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(.02)
    raise RuntimeError(f"{label}超时")


def qmul(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return (w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
            w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v)


def unit(q):
    norm = math.sqrt(sum(v*v for v in q))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("姿态四元数无效")
    return tuple(v/norm for v in q)


def inverse(q):
    w, x, y, z = unit(q)
    return (w, -x, -y, -z)


def rotate(q, point):
    q = unit(q)
    return qmul(qmul(q, (0., *point)), inverse(q))[1:]


def plus(a, b):
    return tuple(x+y for x, y in zip(a, b))


def minus(a, b):
    return tuple(x-y for x, y in zip(a, b))


def distance(a, b):
    return math.sqrt(sum((x-y)**2 for x, y in zip(a, b)))


def rotation_error_deg(a, b):
    similarity = min(1., abs(sum(x*y for x, y in zip(unit(a), unit(b)))))
    return math.degrees(2*math.acos(similarity))


def joint_rotation(yaw_deg, pitch_deg):
    y, p = -math.radians(yaw_deg)/2, -math.radians(pitch_deg)/2
    return qmul((math.cos(y), math.sin(y), 0., 0.),
                (math.cos(p), 0., math.sin(p), 0.))


def sensor_rotation():
    return (math.sqrt(.5), 0., -math.sqrt(.5), 0.)


def compose(parent, child):
    return dict(position=plus(parent['position'], rotate(parent['quaternion'], child['position'])),
                quaternion=qmul(parent['quaternion'], child['quaternion']))


def optical_check(telemetry, model, mount, camera, sensor):
    """实际pose_v模型/link链与固定机械定义独立比对；link位姿相对model。"""
    yaw = float(telemetry['joints_deg']['yaw']['pos'])
    pitch = float(telemetry['joints_deg']['pitch']['pos'])
    if telemetry.get('protocol_version') != 6 or telemetry.get('angle_convention') != 'motion_head_joint':
        raise ValueError('实际吊舱必须使用协议6 / motion_head_joint')
    if telemetry.get('mount', {}).get('static') is not False:
        raise ValueError('实际吊舱必须启用动态安装姿态')
    mount_world = compose(model, mount)
    camera_world = compose(model, camera)
    optical_world = compose(camera_world, sensor)
    mount_inverse = inverse(mount_world['quaternion'])
    optical_mount = qmul(mount_inverse, optical_world['quaternion'])
    expected = qmul(joint_rotation(yaw, pitch), sensor_rotation())
    actual_center = rotate(mount_inverse, minus(camera_world['position'], mount_world['position']))
    lens_center = rotate(mount_inverse, minus(optical_world['position'], mount_world['position']))
    los = rotate(optical_mount, (1., 0., 0.))
    expected_los = rotate(expected, (1., 0., 0.))
    expected_lens = plus(BALL_CENTER, tuple(SENSOR_OFFSET*v for v in expected_los))
    checks = dict(rotation_error_deg=rotation_error_deg(optical_mount, expected),
                  telemetry_rotation_error_deg=rotation_error_deg(optical_world['quaternion'], telemetry['camera']['quat_world']),
                  mount_rotation_error_deg=rotation_error_deg(mount_world['quaternion'], telemetry['mount']['quat_world']),
                  ball_center_error_m=distance(actual_center, BALL_CENTER),
                  lens_center_error_m=distance(lens_center, expected_lens),
                  los_error=distance(los, telemetry['los_mount']))
    if (max(checks[k] for k in ('rotation_error_deg', 'telemetry_rotation_error_deg', 'mount_rotation_error_deg')) > .3
            or max(checks[k] for k in ('ball_center_error_m', 'lens_center_error_m')) > .001
            or checks['los_error'] > .006):
        raise ValueError('实际轴/光学/球心外参不符: '+json.dumps(checks))
    return dict(joints_deg=dict(yaw=yaw, pitch=pitch), actual_los_mount=los,
                actual_image_up_mount=rotate(optical_mount, (0., 0., 1.)),
                actual_ball_center_mount_m=actual_center, actual_lens_center_mount_m=lens_center,
                errors=checks)


def angles(client):
    attitude = client.snapshot()['attitude_deg']
    return {axis: float(attitude[axis]['pos']) for axis in LIMITS}


def check_bounds(clients):
    for client in clients:
        for axis, (lower, upper) in LIMITS.items():
            value = angles(client)[axis]
            if not lower-.2 <= value <= upper+.2:
                raise RuntimeError(f'实际{axis}角度越界: {value}')


class PoseMonitor:
    def __init__(self, world, count):
        from gz.msgs10.empty_pb2 import Empty
        from gz.msgs10.scene_pb2 import Scene
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node
        self.node, self.lock, self.poses = Node(), threading.Lock(), {}
        self.topic = f'/world/{world}/dynamic_pose/info'
        ok, scene = self.node.request(f'/world/{world}/scene/info', Empty(), Empty, Scene, 5000)
        if not ok:
            raise RuntimeError('无法读取实际Gazebo scene，不能按实体ID区分多机link')
        self.aircraft = {}
        for aircraft_id in range(1, count+1):
            name = f'rocket_two_axis_pod_{aircraft_id}'
            model = next((item for item in scene.model if item.name == name), None)
            if model is None:
                raise RuntimeError(f'实际scene缺少{name}')
            links = {item.name.rsplit('::', 1)[-1]: item for item in model.link}
            if not {'mount_link', 'yaw_arm_link', 'camera_link'} <= links.keys():
                raise RuntimeError(f'{name}缺少独立安装/外轴/相机link')
            sensor = next((item for item in links['camera_link'].sensor if item.name.rsplit('::', 1)[-1] == 'camera'), None)
            if sensor is None:
                raise RuntimeError(f'{name}缺少实际camera sensor')
            s = self.read_pose(sensor.pose)
            if distance(s['position'], (0., 0., SENSOR_OFFSET)) > 1e-8 or rotation_error_deg(s['quaternion'], sensor_rotation()) > 1e-5:
                raise RuntimeError(f'{name}实际镜头外参不是新CAD的+Z 23.7mm/固定光学基底')
            self.aircraft[aircraft_id] = dict(model_id=model.id, mount_id=links['mount_link'].id,
                camera_id=links['camera_link'].id, yaw_id=links['yaw_arm_link'].id, sensor=s)
        if not self.node.subscribe(Pose_V, self.topic, self.receive):
            raise RuntimeError('无法订阅实际Gazebo位姿')

    @staticmethod
    def read_pose(pose):
        p, q = pose.position, pose.orientation
        return dict(position=(p.x, p.y, p.z), quaternion=(q.w, q.x, q.y, q.z))

    def receive(self, message):
        now = time.monotonic()
        with self.lock:
            for pose in message.pose:
                self.poses[pose.id] = (self.read_pose(pose), now)

    def capture(self, aircraft_id, telemetry):
        ids = self.aircraft[aircraft_id]
        def fresh():
            with self.lock:
                samples = [self.poses.get(ids[key]) for key in ('model_id', 'mount_id', 'camera_id')]
            return [item[0] for item in samples] if all(item and time.monotonic()-item[1] < 1 for item in samples) else None
        model, mount, camera = wait_for(fresh, f'飞机{aircraft_id}实际位姿')
        return optical_check(telemetry, model, mount, camera, ids['sensor'])

    def close(self):
        self.node.unsubscribe(self.topic)


def runtime_model_check(world_file, count):
    world = ET.parse(world_file).getroot().find('world')
    for aircraft_id in range(1, count+1):
        model = world.find(f"model[@name='rocket_two_axis_pod_{aircraft_id}']")
        if model is None:
            raise ValueError(f'运行world缺少飞机{aircraft_id}')
        for name, expected in (('yaw_joint', (-1., 0., 0.)), ('pitch_joint', (0., -1., 0.))):
            joint = model.find(f"joint[@name='{name}']")
            actual = tuple(map(float, joint.findtext('axis/xyz').split()))
            limits = tuple(math.degrees(float(joint.findtext('axis/limit/'+bound))) for bound in ('lower', 'upper'))
            if actual != expected or max(abs(a-b) for a, b in zip(limits, LIMITS[name.split('_')[0]])) > 1e-7:
                raise ValueError(f'运行world的{name}未继承新CAD轴向/范围')
        meshes = model.findall("link/visual/geometry/mesh/uri")
        if not any('pod_fixed_cowling.dae' in item.text for item in meshes):
            raise ValueError('运行world没有继承新CAD固定罩壳网格')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('-n', type=int, default=1)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--assets-root', type=Path, default=Path(__file__).resolve().parents[3]/'px4-gazebo-assets')
    parser.add_argument('--runtime-dir', type=Path, default=Path('/tmp/px4-scenarios/02_rocket_two_axis_pod'))
    parser.add_argument('--world', default='geo_32_217857_119_105878')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not 1 <= args.n <= 10:
        parser.error('-n 必须为1～10')
    runtime_model_check(args.runtime_dir/'world.sdf', args.n)
    sys.path.insert(0, str(args.assets_root.resolve()/'models/gimbals/two_axis_pod/ground_station'))
    from ground_station import UdpClient
    clients = [UdpClient(args.host, 8770+i, aircraft_id=i+1) for i in range(args.n)]
    report = dict(passed=False, protocol_version=6, angle_convention='motion_head_joint', aircraft=[])
    monitor, failure = None, None

    def settled(client, before):
        t = client.snapshot()
        return t and t.get('ts_mono', -1) > before and all(abs(t['attitude_deg'][axis]['vel']) < .25 for axis in LIMITS)

    def home(client):
        before = client.snapshot().get('ts_mono', -1)
        seq = client.send_reliable('home')
        if seq < 0:
            raise RuntimeError('网关未接受回零请求')
        reply = wait_for(lambda: client.find_result(seq=seq), '回零回执')
        if not reply.get('ok'):
            raise RuntimeError(f'吊舱回零失败: {reply}')
        wait_for(lambda: settled(client, before) and all(abs(v) < .1 for v in angles(client).values()), '回零并静止', 20)

    def stop(client):
        before = client.snapshot().get('ts_mono', -1)
        client.send_manual()
        wait_for(lambda: settled(client, before), '机械停止')

    def isolated(before, active):
        changes = [{axis: end[axis]-start[axis] for axis in LIMITS}
                   for start, end in zip(before, [angles(c) for c in clients])]
        for other, change in enumerate(changes):
            if other != active and any(abs(value) > .75 for value in change.values()):
                raise RuntimeError(f'控制飞机{active+1}影响飞机{other+1}: {changes}')
        return changes

    try:
        for client in clients:
            client.start()
        for i, client in enumerate(clients):
            wait_for(lambda: client.public_angles_ready, f'飞机{i+1}身份与遥测')
            t = client.snapshot()
            if (client.hello.get('version') != 6 or client.hello.get('angle_convention') != 'motion_head_joint'
                    or client.hello.get('video_rtsp_path') != f'/two_axis_pod_{i+1}'
                    or t.get('aircraft_id') != i+1):
                raise RuntimeError(f'飞机{i+1}身份/协议/视频路径不符')
            home(client)
        monitor = PoseMonitor(args.world, args.n)
        for active, client in enumerate(clients):
            entry = dict(aircraft_id=active+1, udp_port=8770+active,
                         video_path=client.hello['video_rtsp_path'], optical_samples=[], actual_limits_deg={})
            report['aircraft'].append(entry)
            entry['optical_samples'].append(dict(label='zero', **monitor.capture(active+1, client.snapshot())))
            before = [angles(c) for c in clients]
            try:
                deadline = time.monotonic()+1.2
                while time.monotonic() < deadline:
                    if client.send_manual(yaw=12., pitch=8.) < 0:
                        raise RuntimeError('双轴速度指令被拒绝')
                    check_bounds(clients)
                    time.sleep(.1)
            finally:
                stop(client)
            changes = isolated(before, active)
            if changes[active]['yaw'] < 6 or changes[active]['pitch'] < 3:
                raise RuntimeError(f'飞机{active+1}双轴未按指令转动: {changes}')
            entry['all_aircraft_angle_changes_deg'] = changes
            entry['optical_samples'].append(dict(label='combined_motion', **monitor.capture(active+1, client.snapshot())))
            home(client)
            for axis, (lower, upper) in LIMITS.items():
                for endpoint in (lower, upper):
                    home(client)
                    before = [angles(c) for c in clients]
                    deadline = time.monotonic()+20
                    rate = math.copysign(60., endpoint)
                    while time.monotonic() < deadline:
                        if client.send_manual(**{axis: rate}) < 0:
                            raise RuntimeError('限位运动指令被拒绝')
                        check_bounds(clients)
                        t = client.snapshot()
                        if abs(angles(client)[axis]-endpoint) < .4 and t['limits'][axis]:
                            break
                        time.sleep(.1)
                    else:
                        raise RuntimeError(f'飞机{active+1}/{axis}未到端点{endpoint}')
                    stop(client)
                    isolated(before, active)
                    label = f'{axis}_{endpoint:g}'
                    entry['actual_limits_deg'][label] = angles(client)[axis]
                    entry['optical_samples'].append(dict(label=label, **monitor.capture(active+1, client.snapshot())))
            home(client)
            entry['optical_samples'].append(dict(label='final_zero', **monitor.capture(active+1, client.snapshot())))
            entry['passed'] = True
            print(f'PASS: 飞机{active+1}身份、双轴隔离、四端限位、实际光学与球心', flush=True)
        report['passed'] = True
    except Exception as error:
        failure = str(error)
        report['error'] = failure
    finally:
        for client in clients:
            try:
                if client.public_angles_ready:
                    stop(client)
                    home(client)
            except Exception as error:
                report['passed'] = False
                report.setdefault('cleanup_errors', []).append(str(error))
            finally:
                client.stop()
        if monitor is not None:
            monitor.close()
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
