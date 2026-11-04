#!/usr/bin/env python3
"""02场景的中科D80独立入口；只管理本运行目录登记的进程。"""
from __future__ import annotations

import argparse
import configparser
from contextlib import contextmanager
import fcntl
import hashlib
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
PX4 = HERE.parents[1]
MODEL = 'rocket_zhongke_d80'
WORLD = 'geo_32_217857_119_105878'
DEFAULT_RUNTIME = Path(os.environ.get('TMPDIR', '/tmp')) / 'px4-scenarios/02_rocket_zhongke_d80'


def read_manifest(runtime_dir):
    """读取场景独立清单；不建立UDP租约、不导入Gazebo。"""
    result = configparser.ConfigParser(interpolation=None)
    result.read(Path(runtime_dir) / 'session.ini', encoding='utf-8')
    if result.get('session', 'kind', fallback='') != 'scenario02_zhongke_d80':
        raise ValueError('运行目录缺少有效的中科02场景清单')
    return result


def write_manifest(runtime_dir, manifest):
    path = Path(runtime_dir) / 'session.ini'
    temporary = path.with_suffix('.ini.tmp')
    with temporary.open('w', encoding='utf-8') as stream:
        stream.write('# 中科02场景运行清单；仅本目录记录的PID和启动时刻用于清理。\n')
        manifest.write(stream)
    temporary.replace(path)


def process_identity(pid):
    """Linux PID与start_ticks绑定，拒绝僵尸和已重用PID。"""
    try:
        values = (Path('/proc') / str(int(pid)) / 'stat').read_text().rsplit(')', 1)[1].split()
        if values[0] == 'Z':
            return None
        return {'pid': str(int(pid)), 'start_ticks': values[19],
                'ppid': values[1], 'pgid': values[2]}
    except (OSError, ValueError, IndexError):
        return None


def identity_alive(identity):
    current = process_identity(identity.get('pid', '0'))
    return current is not None and current['start_ticks'] == identity.get('start_ticks')


@contextmanager
def runtime_lock(runtime_dir):
    Path(runtime_dir).mkdir(parents=True, exist_ok=True)
    with (Path(runtime_dir) / '.launcher.lock').open('a') as stream:
        fcntl.flock(stream, fcntl.LOCK_EX)
        yield


def stop_runtime(runtime_dir):
    """只终止身份仍匹配的登记进程及其当前子进程；不搜索其他场景。"""
    runtime_dir = Path(runtime_dir).resolve()
    with runtime_lock(runtime_dir):
        if not (runtime_dir / 'session.ini').exists():
            print('此中科运行目录没有已登记的进程。')
            return
        manifest = read_manifest(runtime_dir)
        owned = {section: dict(manifest[section]) for section in manifest.sections()
                 if section.startswith('process ') and identity_alive(manifest[section])}
        identities = {value['pid']: value for value in owned.values()}
        # 从已验证身份向下遍历，避免按进程名或其他分区进行全局清理。
        while True:
            previous = len(identities)
            for stat in Path('/proc').glob('[0-9]*/stat'):
                child = process_identity(stat.parent.name)
                if child and child['ppid'] in identities:
                    parent = identities[child['ppid']]
                    if identity_alive(parent):
                        identities[child['pid']] = child
            if len(identities) == previous:
                break
        for signum, grace in ((signal.SIGTERM, 5), (signal.SIGKILL, 1)):
            for identity in reversed(list(identities.values())):
                if identity_alive(identity):
                    try:
                        os.kill(int(identity['pid']), signum)
                    except ProcessLookupError:
                        pass
            deadline = time.monotonic() + grace
            while time.monotonic() < deadline and any(identity_alive(v) for v in identities.values()):
                time.sleep(.1)
        manifest['session']['state'] = 'stopped'
        write_manifest(runtime_dir, manifest)
        print(f'中科02场景已停止；清单与日志保留：{runtime_dir}')


def update_ini(source, output, replacements):
    """保留性能INI注释，只覆盖场景拥有的身份、目标和端点。"""
    section = None
    remaining = set(replacements)
    lines = []
    for line in Path(source).read_text(encoding='utf-8').splitlines():
        match = re.match(r'\s*\[([^]]+)\]', line)
        if match:
            section = match.group(1)
        setting = re.match(r'\s*([a-zA-Z_][\w]*)\s*=', line)
        key = (section, setting.group(1)) if setting else None
        if key in replacements:
            line = f'{key[1]} = {replacements[key]}'
            remaining.remove(key)
        lines.append(line)
    if remaining:
        raise ValueError(f'性能INI缺少场景所需配置项：{sorted(remaining)}')
    Path(output).parent.mkdir(parents=True, exist_ok=True)
    Path(output).write_text('# 02场景逐机性能配置；修改后重新生成和启动本场景。\n' + '\n'.join(lines) + '\n', encoding='utf-8')


def checked(command, **kwargs):
    print('执行：' + shlex.join([str(value) for value in command]), flush=True)
    subprocess.run([str(value) for value in command], check=True, **kwargs)


def prepare_runtime(args):
    """离线生成外部INI、组合模型、世界和清单；不启动或停止任何进程。"""
    runtime = Path(args.runtime_dir).expanduser().resolve()
    assets = Path(args.assets_root).expanduser().resolve()
    component = assets / 'models/gimbals/zhongke-d80'
    generator = assets / f'assembledmodels/{MODEL}/tools/generate_dynamic_model.py'
    for source in (generator, component / 'parameters.ini', PX4 / 'build/px4_sitl_default/etc',
                   assets / f'worlds/{WORLD}/world.sdf'):
        if not source.exists():
            raise ValueError(f'缺少必要文件或目录：{source}')
    legacy_runtime = Path(os.environ.get('TMPDIR', '/tmp')) / 'px4-scenarios/02_rocket_two_axis_pod'
    if runtime == legacy_runtime.resolve():
        raise ValueError('中科不能使用旧02场景的运行目录，请指定独立--runtime-dir')
    if runtime.exists() and not (runtime / 'session.ini').exists():
        if any(item.name != '.launcher.lock' for item in runtime.iterdir()):
            raise ValueError('运行目录非空且不属于中科02场景，拒绝覆盖已有文件')
    if (runtime / 'session.ini').exists():
        previous = read_manifest(runtime)
        if any(identity_alive(previous[section]) for section in previous.sections() if section.startswith('process ')):
            raise ValueError('此运行目录仍有已登记进程；请先显式停止该中科运行目录')
    runtime.mkdir(parents=True, exist_ok=True)
    for directory in ('logs', 'config', 'generated', 'models'):
        (runtime / directory).mkdir(exist_ok=True)
    # 使用当前本机资产，不修改资产仓分支或获取远程代码。
    sys.path.insert(0, str(component))
    from common.config import load_config
    source_config = Path(args.config).expanduser().resolve() if args.config else component / 'parameters.ini'
    targets = [f'{MODEL}_{i}' for i in range(1, args.count + 1)] + ['xiaomotuo', 'xiaobianfu']
    partition = args.partition or 'px4_scenario_02_zhongke_' + hashlib.sha256(str(runtime).encode()).hexdigest()[:12]
    manifest = configparser.ConfigParser(interpolation=None)
    manifest['session'] = dict(kind='scenario02_zhongke_d80', state='prepared', world=WORLD,
        gz_partition=partition, world_path=str(runtime / 'world.sdf'), assets_root=str(assets),
        count=str(args.count), aircraft_model=MODEL, runtime_dir=str(runtime),
        component_root=str(component), python=args.python, gui=str(int(args.gui)), no_video=str(int(args.no_video)))
    models = []
    for number in range(1, args.count + 1):
        index = number - 1
        entity = f'{MODEL}_{number}'
        prefix = f'/uav_{number}/pod'
        config_path = runtime / f'config/aircraft_{number}.ini'
        control_port, rtsp_port = args.udp_base + index, args.rtsp_base + index
        update_ini(source_config, config_path, {
            ('model', 'name'): entity, ('model', 'aircraft_id'): number,
            ('tracking', 'target_names'): ','.join(targets),
            ('network', 'udp_bind'): '0.0.0.0', ('network', 'udp_port'): control_port,
            ('video', 'rtsp_bind'): '0.0.0.0', ('video', 'rtsp_port'): rtsp_port,
            ('video', 'rtsp_mount'): f'/zhongke_d80_{number}', ('demo', 'world_name'): WORLD,
        })
        config = load_config(config_path)
        generated = runtime / f'generated/aircraft_{number}/{MODEL}'
        checked([args.python, generator, '--config', config_path, '--output-dir', generated, '--topic-prefix', prefix])
        models.extend(['--model', str(number), str(generated / 'model.sdf')])
        station_path = runtime / f'config/ground_station_{number}.ini'
        address = args.advertise_host or '127.0.0.1'
        rtsp_url = f'rtsp://{address}:{rtsp_port}/zhongke_d80_{number}'
        station_path.write_text(
            '# 外部独立地面站连接；性能参数与本机模型、网关使用同一INI。\n'
            f'[connection]\nudp_host = {address}\nudp_port = {control_port}\nlocal_bind = 0.0.0.0\nlocal_port = 0\n'
            f'rtsp_url = {rtsp_url}\nparameters_file = {config_path}\n\n'
            f'[video]\ndisplay_scale = 0.5\ndecoder = nvdec\nwindow_title = 中科 D80 · 飞机 {number}\n\n'
            '# JSON仅由地面站按按钮外发；此端口没有PX4中科接收桥。\n'
            f'[forward]\nip = 127.0.0.1\nport = {9100 + index}\n', encoding='utf-8')
        manifest[f'aircraft {number}'] = dict(entity=entity, topic_prefix=prefix,
            image_topic=prefix + '/image', diagnostics_topic=prefix + '/diagnostics',
            telemetry_topic=prefix + '/telemetry', config_path=str(config_path),
            config_hash=config['config_hash'], station_config_path=str(station_path),
            generated_dir=str(generated), udp_host='127.0.0.1', udp_port=str(control_port),
            rtsp_url=rtsp_url, rtsp_port=str(rtsp_port), rtsp_mount=f'/zhongke_d80_{number}',
            mavsdk_udp=str(14280 + index), px4_instance=str(index))
    prepare_command = [args.python, HERE / 'prepare_world.py', '--source-world', assets / f'worlds/{WORLD}/world.sdf',
        '--output', runtime / 'world.sdf', '--model-basename', MODEL, '--tracking-backend', 'sim_truth', *models]
    if args.gui:
        prepare_command.append('--target-control')
    if args.enable_collision:
        prepare_command.append('--enable-collision')
    checked(prepare_command)
    etc = runtime / 'etc'
    shutil.copytree(PX4 / 'build/px4_sitl_default/etc', etc, dirs_exist_ok=True)
    airframes = list((runtime / f'generated/aircraft_1/{MODEL}/px4').glob(f'*_gz_{MODEL}'))
    if len(airframes) != 1:
        raise ValueError('组合生成器必须提供唯一的中科PX4 airframe')
    target = etc / 'init.d-posix/airframes' / airframes[0].name
    data = airframes[0].read_text()
    if not re.search(r'^param set-default EKF2_HGT_REF ', data, re.M):
        raise ValueError('组合airframe缺少EKF2_HGT_REF配置')
    target.write_text(re.sub(r'^param set-default EKF2_HGT_REF .*$', 'param set-default EKF2_HGT_REF 1', data, flags=re.M))
    # 世界已显式声明全部传感器与物理系统，不加载其他云台的server.config。
    (runtime / 'server.config').write_text('<server_config><plugins/></server_config>\n')
    write_manifest(runtime, manifest)
    (runtime / 'count').write_text(str(args.count) + '\n')
    (runtime / 'assets_root').write_text(str(assets) + '\n')
    print(f'中科场景离线准备完成：{runtime / "session.ini"}')
    return manifest


def check_ports(manifest, no_video=False):
    """占用时拒绝启动；不接管、不停止其他组件。"""
    sockets = []
    try:
        for section in manifest.sections():
            if not section.startswith('aircraft '):
                continue
            aircraft = manifest[section]
            # PX4本地MAVLink绑定端口及MAVSDK目的端口均在启动前检查。
            index = int(aircraft['px4_instance'])
            udp_ports = [int(aircraft['udp_port']), int(aircraft['mavsdk_udp']), 18570 + index, 14580 + index, 13030 + index]
            for port in udp_ports:
                item = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sockets.append(item)
                try:
                    item.bind(('0.0.0.0', port))
                except OSError as error:
                    raise ValueError(f'UDP端口{port}已占用，未启动任何场景进程：{error}') from error
            if not no_video:
                item = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sockets.append(item)
                try:
                    item.bind(('0.0.0.0', int(aircraft['rtsp_port'])))
                except OSError as error:
                    raise ValueError(f'RTSP端口{aircraft["rtsp_port"]}已占用，未接管已有服务：{error}') from error
    finally:
        for item in sockets:
            item.close()


def record_process(runtime, manifest, name, pid):
    identity = process_identity(pid)
    if identity is None:
        raise ValueError(f'进程{name}启动后立即退出')
    manifest['process ' + name] = identity
    write_manifest(runtime, manifest)


def start_process(runtime, manifest, name, command, env):
    logfile = Path(runtime) / f'logs/{name}.log'
    print(f'启动{name}：' + shlex.join([str(v) for v in command]), flush=True)
    with logfile.open('wb') as output:
        process = subprocess.Popen([str(v) for v in command], stdout=output, stderr=subprocess.STDOUT,
                                   stdin=subprocess.DEVNULL, env=env, start_new_session=True)
    record_process(runtime, manifest, name, process.pid)
    return process


def wait_log(runtime, manifest, name, text, timeout):
    deadline = time.monotonic() + timeout
    path = Path(runtime) / f'logs/{name}.log'
    while time.monotonic() < deadline:
        if not identity_alive(manifest['process ' + name]):
            raise ValueError(f'{name}启动失败，详见{path}')
        if text in path.read_text(errors='replace'):
            return
        time.sleep(.25)
    raise ValueError(f'等待{name}就绪超时，详见{path}')


def start_runtime(args, manifest):
    runtime = Path(args.runtime_dir).expanduser().resolve()
    session = manifest['session']
    component = Path(session['component_root'])
    library = component / 'controller/build/lib/libgz-zhongke-d80-controller.so'
    px4_binary = PX4 / 'build/px4_sitl_default/bin/px4'
    for required in (library, px4_binary):
        if not required.is_file():
            raise ValueError(f'请先完成已有组件构建：{required}')
    check_ports(manifest, args.no_video)
    build = runtime / 'build/scenario_gui'
    checked(['cmake', '-S', HERE / 'gui', '-B', build, '-DCMAKE_BUILD_TYPE=Release',
             '-DSCENARIO02_BUILD_GUI=' + ('ON' if args.gui else 'OFF'), '-DBUILD_TESTING=OFF'])
    checked(['cmake', '--build', build, '-j', str(min(os.cpu_count() or 1, 8))])
    checked(['gz', 'sdf', '-k', runtime / 'world.sdf'])
    assets = Path(session['assets_root'])
    resources = [str(runtime / f'generated/aircraft_{i}') for i in range(1, args.count + 1)]
    resources += [str(assets / name) for name in ('assembledmodels', 'models/vehicles', 'models/gimbals')]
    env = os.environ.copy()
    env.update(GZ_PARTITION=session['gz_partition'], GZ_IP='127.0.0.1',
        GZ_SIM_RESOURCE_PATH=':'.join(resources + [env.get('GZ_SIM_RESOURCE_PATH', '')]),
        GZ_SIM_SYSTEM_PLUGIN_PATH=':'.join([str(library.parent), str(build / 'target-system-plugins'), env.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')]),
        GZ_SIM_SERVER_CONFIG_PATH=str(runtime / 'server.config'))
    if args.gui:
        env['GZ_GUI_PLUGIN_PATH'] = ':'.join([str(build), str(assets / f'worlds/{WORLD}/gui/build'), env.get('GZ_GUI_PLUGIN_PATH', '')])
    session['state'] = 'starting'
    write_manifest(runtime, manifest)
    headless_env = env.copy()
    headless_env.pop('DISPLAY', None)
    start_process(runtime, manifest, 'gazebo', ['gz', 'sim', '-r', '-s', '--headless-rendering', '-v2', runtime / 'world.sdf'], headless_env)
    for number in range(1, args.count + 1):
        aircraft = manifest[f'aircraft {number}']
        imu = f'/world/{WORLD}/model/{aircraft["entity"]}/link/base_link/sensor/imu_sensor/imu'
        deadline = time.monotonic() + 120
        while True:
            try:
                result = subprocess.run(['gz', 'topic', '-e', '-t', imu, '-n', '1'], env=env,
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=8)
                if result.returncode == 0:
                    break
            except subprocess.TimeoutExpired:
                pass
            if time.monotonic() >= deadline or not identity_alive(manifest['process gazebo']):
                raise ValueError(f'等待飞机{number}的真实IMU超时：{imu}')
        pxenv = env.copy()
        pxenv.update(PX4_SIM_MODEL='gz_' + MODEL, PX4_GZ_STANDALONE='1', PX4_GZ_WORLD=WORLD,
                     PX4_GZ_MODEL_NAME=aircraft['entity'])
        rootfs = runtime / f'rootfs_{number}'
        rootfs.mkdir(exist_ok=True)
        name = f'px4_{number}'
        start_process(runtime, manifest, name, [px4_binary, '-d', runtime / 'etc', '-i', aircraft['px4_instance'], '-w', rootfs], pxenv)
        wait_log(runtime, manifest, name, 'Startup script returned successfully', 90)
        command = [args.python, '-u', component / 'controller/service/gimbal_bridge.py', '--config', aircraft['config_path'],
                   '--world', WORLD, '--model', aircraft['entity'], '--topic-prefix', aircraft['topic_prefix'],
                   '--image-topic', aircraft['image_topic']]
        if args.no_video:
            command.append('--no-video')
        name = f'gateway_{number}'
        start_process(runtime, manifest, name, command, env)
        wait_log(runtime, manifest, name, '中科网关启动', 30)
        if not args.no_video:
            # relay使用独立守护进程，必须登记其实际PID，不能只登记网关父进程。
            sys.path.insert(0, str(component / 'controller/service'))
            from rtsp_relay import request
            deadline = time.monotonic() + 60
            while True:
                try:
                    relay = request(int(aircraft['rtsp_port']), {'op': 'ping'})
                    relay_identity = process_identity(relay['pid'])
                    gateway_identity = manifest['process ' + name]
                    expected_script = str(component / 'controller/service/rtsp_relay.py')
                    command_line = (Path('/proc') / str(relay['pid']) / 'cmdline').read_bytes().split(b'\0')
                    if (relay_identity and int(relay_identity['start_ticks']) >= int(gateway_identity['start_ticks'])
                            and expected_script.encode() in command_line and relay.get('bind') == '0.0.0.0'):
                        record_process(runtime, manifest, f'rtsp_relay_{number}', int(relay['pid']))
                        if aircraft['rtsp_mount'] in relay.get('paths', []):
                            break
                except (OSError, ValueError, KeyError, RuntimeError):
                    pass
                if time.monotonic() > deadline:
                    raise ValueError(f'等待飞机{number}独立RTSP中继超时')
                time.sleep(.25)
    if args.gui:
        start_process(runtime, manifest, 'target_manager', [args.python, '-u', HERE / 'target_manager.py',
            '--world', WORLD, '--runtime-dir', runtime, '--assets-root', assets,
            '--qgc-address', args.target_qgc_address, '--mavlink-system-id', str(args.target_sysid)], env)
        wait_log(runtime, manifest, 'target_manager', '靶机管理服务已就绪', 15)
        start_process(runtime, manifest, 'radar_manager', [args.python, '-u', HERE / 'radar_manager.py',
            '--world', WORLD, '--metadata', assets / f'worlds/{WORLD}/metadata.json'], env)
        wait_log(runtime, manifest, 'radar_manager', '雷达管理服务已就绪', 15)
        start_process(runtime, manifest, 'gui', ['gz', 'sim', '-g', '-v2'], env)
    session['state'] = 'ready'
    write_manifest(runtime, manifest)
    print(f'中科02场景已就绪，PX4保持待命：{runtime / "session.ini"}')
    for number in range(1, args.count + 1):
        aircraft = manifest[f'aircraft {number}']
        print(f'飞机{number}：二进制UDP {aircraft["udp_port"]}；MAVSDK udpout://127.0.0.1:{aircraft["mavsdk_udp"]}；{aircraft["rtsp_url"]}')
        print('独立地面站：' + shlex.join([args.python, str(component / 'ground_station/ground_station.py'), '--station-config', aircraft['station_config_path']]))
    print('只停止此场景：' + shlex.join(['bash', str(HERE / 'stop.sh'), '--aircraft-model', MODEL, '--runtime-dir', str(runtime)]))


def choose_python(assets_root):
    """显式解释器优先，否则使用中科已有虚拟环境。"""
    explicit = os.environ.get('GIMBAL_PYTHON')
    if explicit:
        return explicit
    virtual = Path(assets_root).expanduser().resolve() / 'models/gimbals/.venv-zhongke-d80/bin/python'
    return str(virtual) if virtual.is_file() else sys.executable


def preflight_dependencies(args):
    """在任何场景进程启动前检查解释器及硬件视频插件依赖。"""
    script = """
import cv2, numpy, gi
import gz.transport13
from gz.msgs10 import image_pb2, pose_v_pb2, stringmsg_pb2
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer
Gst.init(None)
"""
    if not args.no_video:
        script += """
required = ['appsrc', 'videoconvert', 'nvcudah264enc', 'nvh264dec', 'rtph264pay', 'rtph264depay', 'udpsrc', 'udpsink']
missing = [name for name in required if Gst.ElementFactory.find(name) is None]
if missing:
    raise RuntimeError('缺少NVENC/NVDEC或视频插件：' + ','.join(missing))
"""
    try:
        result = subprocess.run([args.python, '-c', script], capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.TimeoutExpired) as error:
        raise ValueError(f'中科解释器依赖预检查失败：{args.python}：{error}') from error
    if result.returncode:
        setup = Path(args.assets_root).expanduser().resolve() / 'models/gimbals/zhongke-d80/scripts/setup.sh'
        detail = (result.stderr or result.stdout).strip()
        raise ValueError(f'中科解释器依赖不完整：{args.python}；{detail}。请使用已有中科虚拟环境或先按基础组件 {setup} 配置环境；未启动场景进程')
    print(f'中科解释器及依赖预检查通过：{args.python}', flush=True)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='02场景中科D80独立启动；不调用旧场景的全局进程清理')
    parser.add_argument('-n', '--count', type=int, default=1)
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--runtime-dir', default=str(DEFAULT_RUNTIME))
    parser.add_argument('--partition')
    parser.add_argument('--prepare-only', action='store_true')
    parser.add_argument('--no-video', action='store_true', help='内部验收时仅关闭视频转码，Gazebo真实相机仍工作')
    parser.add_argument('--stop', action='store_true')
    parser.add_argument('--assets-root', default=os.environ.get('PX4_GAZEBO_ASSETS_ROOT', str(PX4.parent / 'px4-gazebo-assets')))
    parser.add_argument('--config', help='源性能INI；生成逐机外部副本，不修改源文件')
    parser.add_argument('--python', help='覆盖中科Python解释器，默认优先已有中科虚拟环境')
    parser.add_argument('--udp-base', type=int, default=8880)
    parser.add_argument('--rtsp-base', type=int, default=8566)
    parser.add_argument('--advertise-host', default='')
    parser.add_argument('--tracking-backend', choices=('sim_truth',), default='sim_truth')
    parser.add_argument('--encoder', choices=('nvenc',), default='nvenc')
    parser.add_argument('--enable-collision', action='store_true')
    parser.add_argument('--target-qgc-address', default='127.0.0.1:14550')
    parser.add_argument('--target-sysid', type=int, default=200)
    args = parser.parse_args(argv)
    args.python = args.python or choose_python(args.assets_root)
    if not 1 <= args.count <= 10:
        parser.error('架数必须为1～10')
    if any(not 1 <= base <= 65536 - args.count for base in (args.udp_base, args.rtsp_base)):
        parser.error('端口范围超出1～65535')
    if not args.count < args.target_sysid <= 254:
        parser.error('靶机SYSID必须在普通飞机ID之后且不大于254')
    if args.partition and (not re.fullmatch(r'[a-zA-Z0-9_-]+', args.partition)):
        parser.error('分区名只允许字母、数字、下划线和连字符')
    if args.partition and 'zhongke' not in args.partition.lower():
        parser.error('中科分区名须包含zhongke，不能复用旧场景分区')
    if args.advertise_host:
        try:
            socket.inet_pton(socket.AF_INET, args.advertise_host)
        except OSError:
            parser.error('advertise-host必须为IPv4地址')
    return args


def main(argv=None):
    args = parse_args(argv)
    started = False
    try:
        if args.stop:
            stop_runtime(args.runtime_dir)
            return 0
        if not args.prepare_only:
            preflight_dependencies(args)
        with runtime_lock(args.runtime_dir):
            manifest = prepare_runtime(args)
            if not args.prepare_only:
                started = True
                start_runtime(args, manifest)
        return 0
    except (OSError, ValueError, subprocess.CalledProcessError, KeyboardInterrupt) as error:
        print(f'中科场景操作失败：{error}', file=sys.stderr)
        if started:
            stop_runtime(args.runtime_dir)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
