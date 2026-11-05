"""用隔离工具替身执行统一场景脚本，不启动仿真或终止真实进程。"""
from __future__ import annotations

import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
ASSETS = HERE.parents[1].parent / 'px4-gazebo-assets'
WORLD = 'geo_32_217857_119_105878'
MODELS = ('rocket_two_axis_pod', 'rocket_aisheng_seeker', 'rocket_zhongke_d80')

# 所有外部进程、端口检测及 kill 均被截获；shell 分支、参数和清理流程保持真实。
TOOL = r'''import json, os, pathlib, sys, time, types
name = pathlib.Path(sys.argv[0]).name
args = sys.argv[1:]
root = pathlib.Path(os.environ['FIXTURE_ROOT'])
log = root / 'calls.jsonl'
def record(tool, argv):
    row = {'tool':tool, 'argv':argv,
           'partition':os.environ.get('GZ_PARTITION'),
           'server_config':os.environ.get('GZ_SIM_SERVER_CONFIG_PATH'),
           'system_plugin_path':os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH')}
    with log.open('a') as stream:
        stream.write(json.dumps(row, ensure_ascii=False)+'\n')
record(name, args)
def value(flag):
    return args[args.index(flag)+1]
if name == 'git':
    if 'branch' in args: print('main')
    if 'rev-parse' in args: print('fixture12345')
elif name == 'pgrep':
    if os.environ.get('FIXTURE_EXISTING') or (root/'started').exists():
        (root/'stopping').touch()
        print('990001')
        sys.exit(0)
    sys.exit(1)
elif name == 'kill':
    alive = root/'alive'
    if args[0] == '-0':
        # 等待就绪时模拟进程存在；停止时使用独立模拟存活状态。
        failed = root/'failed_pid'
        if failed.exists() and args[-1] == failed.read_text(): sys.exit(1)
        stopping = os.environ.get('FIXTURE_STOP') or (root/'stopping').exists()
        sys.exit(0 if alive.exists() or not stopping else 1)
    if args[0] in ('-TERM','-KILL'):
        if args[0] == '-KILL' or not os.environ.get('FIXTURE_STUBBORN'):
            alive.unlink(missing_ok=True)
elif name == 'sleep':
    time.sleep(0.01)
elif name == 'setsid':
    (root/'started').touch()
    fail = os.environ.get('FIXTURE_FAIL_GATEWAY')
    if fail and any('zhongke-d80/controller/service/gimbal_bridge.py' in part for part in args):
        (root/'failed_pid').write_text(str(os.getpid()))
        print('中科网关启动\n中科RTSP已启动\n测试网关启动失败', flush=True)
    else:
        print('Startup script returned successfully\n遥测/指令服务\n网关启动\n中科网关启动\n中科RTSP已启动\nRTSP 服务已启动\n靶机管理服务已就绪\n雷达管理服务已就绪\n碰撞致损服务已就绪', flush=True)
elif name.startswith('px4-'):
    runtime = pathlib.Path(os.environ['TMPDIR'])/'px4-scenarios/02_rocket_two_axis_pod'
    if 'start' in args:
        instance = int(value('--instance'))
        with (runtime/f'logs/px4_{instance+1}.log').open('a') as stream:
            stream.write('listening for Aisheng JSON on UDP 0.0.0.0:'+value('-p')+'\n')
            stream.write('完整遥测接收已就绪 UDP 0.0.0.0:'+value('-p')+'\n')
elif name in ('python3','component-python'):
    if args and args[0] == '-':
        source = sys.stdin.read()
        if 'import socket' not in source: sys.exit(0)
        class Probe:
            def __init__(self, *values): self.values=values
            def __enter__(self): return self
            def __exit__(self, *values): self.close()
            def setsockopt(self, *values): pass
            def bind(self, endpoint):
                record('port_bind', list(endpoint))
                if str(endpoint[1]) == os.environ.get('FIXTURE_OCCUPIED_PORT'):
                    raise OSError('测试端口已被使用')
            def close(self): record('port_close', [])
        socket = types.ModuleType('socket')
        socket.AF_INET=2; socket.SOCK_DGRAM=2; socket.SOCK_STREAM=1
        socket.SOL_SOCKET=1; socket.SO_REUSEADDR=2; socket.socket=Probe
        sys.modules['socket'] = socket
        sys.argv = args
        exec(compile(source, '<端口检测>', 'exec'), {'__name__':'__main__'})
    elif any(part.endswith('generate_dynamic_model.py') for part in args):
        output = pathlib.Path(value('--output-dir'))
        output.mkdir(parents=True)
        (output/'model.sdf').write_text('<sdf version="1.9"><model name="fixture"/></sdf>')
        (output/'px4').mkdir()
        (output/'px4'/('4001_gz_'+output.name)).write_text('param set-default EKF2_HGT_REF 0\n')
    elif any(part.endswith('prepare_world.py') for part in args):
        pathlib.Path(value('--output')).write_text('<sdf version="1.9"><world name="fixture"/></sdf>')
'''


class UnifiedLauncherTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='scenario02-shell-')
        self.root = Path(self.temp.name)
        self.px4 = self.root/'PX4-Autopilot'
        self.scene = self.px4/'scenarios/02_rocket_two_axis_pod'
        self.scene.mkdir(parents=True)
        self.bin = self.root/'bin'
        self.bin.mkdir()
        self.assets = self.root/'assets'
        (self.assets/'.git').mkdir(parents=True)
        for name in ('start.sh','stop.sh'):
            shutil.copy2(HERE/name, self.scene/name)
        self.runtime = self.root/'tmp/px4-scenarios/02_rocket_two_axis_pod'
        self.runtime.parent.mkdir(parents=True)
        for name in ('python3','component-python','git','pgrep','kill','sleep','setsid',
                     'gz','cmake','make'):
            self.executable(self.bin/name, '#!'+sys.executable+'\n'+TOOL)
        self.touch(self.scene/'prepare_world.py')
        for name in ('collision_manager.py','target_manager.py','radar_manager.py'):
            self.touch(self.scene/name)
        self.touch(self.assets/f'worlds/{WORLD}/world.sdf')
        self.touch(self.assets/f'worlds/{WORLD}/metadata.json')
        for model in MODELS:
            self.touch(self.assets/f'assembledmodels/{model}/tools/generate_dynamic_model.py')
        for model in ('xiaomotuo','xiaobianfu'):
            self.touch(self.assets/f'models/vehicles/{model}/model.sdf')
        for component in ('two_axis_pod','aisheng-ASD-2-075-DD0','zhongke-d80'):
            base = self.assets/f'models/gimbals/{component}'
            for file in ('controller/CMakeLists.txt','controller/service/gimbal_bridge.py',
                         'controller/service/rtsp_relay.py',
                         'gateway/main.py','gateway/rtsp_server.py',
                         'ground_station/main.py','ground_station/ground_station.py',
                         'common/protocol.py','parameters.ini','scripts/server.config'):
                self.touch(base/file)
        for name in ('px4','px4-dyt_udp_bridge','px4-aisheng_udp_bridge'):
            self.executable(self.px4/f'build/px4_sitl_default/bin/{name}',
                            '#!'+sys.executable+'\n'+TOOL)
        (self.px4/'build/px4_sitl_default/etc/init.d-posix/airframes').mkdir(parents=True)
        bash_env = self.root/'bash_env'
        bash_env.write_text('kill() { "$FIXTURE_BIN/kill" "$@"; }\n')
        self.env = dict(os.environ, FIXTURE_ROOT=str(self.root), FIXTURE_BIN=str(self.bin),
                        BASH_ENV=str(bash_env), TMPDIR=str(self.root/'tmp'),
                        PX4_GAZEBO_ASSETS_ROOT=str(self.assets),
                        GIMBAL_PYTHON=str(self.bin/'component-python'),
                        PATH=str(self.bin)+os.pathsep+os.environ['PATH'])

    def tearDown(self):
        self.temp.cleanup()

    @staticmethod
    def touch(path):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.touch()

    @staticmethod
    def executable(path, source):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(source)
        path.chmod(0o755)

    def calls(self, tool=None):
        path = self.root/'calls.jsonl'
        rows = [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []
        return [row for row in rows if tool is None or row['tool'] == tool]

    def run_script(self, name='start.sh', argv=(), env=None):
        return subprocess.run(['bash', str(self.scene/name), *argv], env=env or self.env,
                              text=True, capture_output=True, timeout=30)

    def start(self, model=MODELS[2], extra=()):
        result = self.run_script(argv=['--aircraft-model',model,'--advertise-host','192.0.2.56',*extra])
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        return result

    def commands(self, ending):
        return [row['argv'] for row in self.calls() if any(isinstance(part,str) and part.endswith(ending) for part in row['argv'])]

    @staticmethod
    def option(argv, flag):
        return argv[argv.index(flag)+1]

    def test_vibration_defaults_to_off_without_optional_asset_dependency(self):
        self.start(extra=('--flight-vibration','off'))
        self.assertFalse(any('rocket_racing_quad/controller' in ' '.join(row['argv'])
                             for row in self.calls('cmake')))
        prepare = self.commands('prepare_world.py')[0]
        self.assertNotIn('--flight-vibration', prepare)
        self.assertFalse(self.commands('configure_vibration.py'))
        self.assertTrue(all('rocket_racing_quad/controller' not in (row['system_plugin_path'] or '')
                            for row in self.calls('setsid')))

    def test_vibration_enabled_builds_plugin_and_passes_profile_for_every_model(self):
        # 外部工具被隔离替身截获；验证实际shell分支与传参。
        directory = self.assets/'models/vehicles/rocket_racing_quad'
        for file in ('controller/CMakeLists.txt','tools/configure_vibration.py','config/flight_hover.json'):
            self.touch(directory/file)
        for model in MODELS:
            with self.subTest(model=model):
                self.start(model,extra=('--flight-vibration','afly5','-n','2'))
                build = [row['argv'] for row in self.calls('cmake')
                         if str(directory/'controller') in row['argv']]
                self.assertTrue(build)
                prepare = self.commands('prepare_world.py')[-1]
                self.assertEqual(self.option(prepare,'--flight-vibration'),'afly5')
                self.assertEqual(prepare.count('--model'),2)
                configure = self.commands('configure_vibration.py')[-1]
                self.assertEqual(self.option(configure,'--profile'),str(directory/'config/flight_hover.json'))
                self.assertIn('/etc/init.d-posix/airframes/',self.option(configure,'--airframe'))
                gz = [row for row in self.calls('setsid') if 'gz' in row['argv']][-1]
                self.assertIn(str(directory/'controller/build/lib'),
                              next(part for part in gz['argv'] if part.startswith('GZ_SIM_SYSTEM_PLUGIN_PATH=')))
                self.run_script('stop.sh')
                (self.root/'started').unlink(missing_ok=True)
                (self.root/'stopping').unlink(missing_ok=True)

    def test_bad_or_missing_vibration_profile_fails_before_process_probe(self):
        for argv in [('--flight-vibration',),('--flight-vibration','unknown')]:
            with self.subTest(argv=argv):
                result = self.run_script(argv=argv)
                self.assertEqual(result.returncode,2,result.stdout+result.stderr)
                self.assertFalse(self.calls())

    def test_enabled_vibration_missing_asset_reports_before_launch(self):
        result = self.run_script(argv=('--flight-vibration','afly5'))
        self.assertEqual(result.returncode,1,result.stdout+result.stderr)
        self.assertIn('rocket_racing_quad/controller/CMakeLists.txt', result.stderr)
        self.assertFalse(self.calls('setsid'))

    def test_three_models_share_the_public_flow_and_select_their_bridge(self):
        for model, bridge, port in zip(MODELS, ('dyt_udp_bridge','aisheng_udp_bridge',None), (8770,8001,8880)):
            with self.subTest(model=model):
                (self.root/'started').unlink(missing_ok=True)
                (self.root/'calls.jsonl').unlink(missing_ok=True)
                if model == MODELS[2]:
                    for name in ('px4-dyt_udp_bridge','px4-aisheng_udp_bridge'):
                        (self.px4/f'build/px4_sitl_default/bin/{name}').unlink()
                extra = ('--tracking-backend','csrt') if model != MODELS[2] else ()
                self.start(model,extra)
                generators = self.commands('generate_dynamic_model.py')
                self.assertEqual(len(generators), 1)
                self.assertIn('/assembledmodels/'+model+'/', generators[0][0])
                self.assertEqual(self.option(generators[0],'--topic-prefix'), '/uav_1/pod')
                self.assertEqual(len(self.commands('prepare_world.py')), 1)
                self.assertEqual(self.option(self.commands('prepare_world.py')[0],'--tracking-backend'),
                                 'csrt' if extra else 'sim_truth')
                bridge_calls = [r for r in self.calls() if r['tool'].startswith('px4-')]
                self.assertEqual({r['tool'] for r in bridge_calls}, {'px4-'+bridge} if bridge else set())
                self.assertFalse(self.calls('make'),'已有PX4时中科不应因缺少JSON桥而触发构建')
                ports = [r['argv'][1] for r in self.calls('port_bind')]
                self.assertIn(port, ports)
                self.assertTrue(self.runtime.joinpath('interfaces.tsv').exists())
                self.assertFalse(list(self.runtime.rglob('*.ini')))

    def test_two_zhongke_aircraft_use_identical_generator_and_gateway_overrides(self):
        result = self.start(extra=('-n','2','--gui','--enable-collision'))
        generators = self.commands('generate_dynamic_model.py')
        gateways = [argv for argv in self.commands('gimbal_bridge.py') if any('zhongke-d80/controller/service/' in p for p in argv)]
        self.assertEqual(len(generators), 2)
        self.assertEqual(len(gateways), 2)
        fields = ('--config','--model','--aircraft-id','--world','--udp-bind','--udp-port',
                  '--rtsp-bind','--rtsp-port','--rtsp-mount','--topic-prefix')
        for number, (generator,gateway) in enumerate(zip(generators,gateways), 1):
            with self.subTest(aircraft=number):
                for flag in fields:
                    self.assertEqual(self.option(generator,flag), self.option(gateway,flag), flag)
                self.assertEqual(self.option(generator,'--config'),
                                 str(self.assets/'models/gimbals/zhongke-d80/parameters.ini'))
                self.assertEqual(self.option(generator,'--model'), f'rocket_zhongke_d80_{number}')
                self.assertEqual(self.option(generator,'--aircraft-id'), str(number))
                self.assertEqual(self.option(generator,'--udp-port'), str(8879+number))
                self.assertEqual(self.option(generator,'--rtsp-port'), str(8565+number))
                self.assertEqual(self.option(generator,'--rtsp-mount'), f'/zhongke_d80_{number}')
                targets = lambda argv: [argv[i+1] for i, item in enumerate(argv) if item == '--target-name']
                expected = ['rocket_zhongke_d80_1','rocket_zhongke_d80_2','xiaomotuo','xiaobianfu']
                self.assertEqual(targets(generator), expected)
                self.assertEqual(targets(gateway), expected)
        for script in ('target_manager.py','radar_manager.py','collision_manager.py'):
            self.assertEqual(len(self.commands(script)), 1)
        self.assertIn('--enable-collision', self.commands('collision_manager.py')[0])
        self.assertIn('--target-control', self.commands('prepare_world.py')[0])
        gazebo = [row for row in self.calls('setsid') if 'sim' in row['argv']]
        self.assertEqual(len(gazebo), 2)
        self.assertTrue(all(row['partition'] == 'px4_scenario_02_rocket_two_axis_pod' for row in gazebo))
        self.assertTrue(all('two_axis_pod/scripts/server.config' not in (row['server_config'] or '') for row in gazebo))
        stations = [shlex.split(line.split('command: ',1)[1]) for line in result.stdout.splitlines()
                    if '地面站 command: ' in line]
        self.assertEqual(len(stations), 2)
        for number, command in enumerate(stations, 1):
            self.assertIn('--config',command)
            self.assertEqual(self.option(command,'--udp-port'), str(8879+number))
            self.assertEqual(self.option(command,'--rtsp-url'), f'rtsp://192.0.2.56:{8565+number}/zhongke_d80_{number}')
            self.assertNotIn('--station-config',command)
        table = self.runtime.joinpath('interfaces.tsv').read_text()
        self.assertIn('14280',table)
        self.assertIn('14281',table)
        self.assertNotIn('9000',table)
        self.assertFalse(list(self.runtime.rglob('*.ini')))
        checked_ports = {row['argv'][1] for row in self.calls('port_bind')}
        self.assertTrue({8880,8881,8566,8567,14280,14281,18570,18571,14580,14581,13030,13031} <= checked_ports)
        self.assertEqual(len(self.calls('port_bind')),len(self.calls('port_close')))

    def test_default_model_and_no_argument_stop_keep_public_contract(self):
        result = self.run_script(argv=['--advertise-host','192.0.2.56'])
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)
        self.assertIn('/rocket_two_axis_pod/',self.commands('generate_dynamic_model.py')[0][0])
        result = self.run_script('stop.sh',argv=['--aircraft-model',MODELS[2]])
        self.assertEqual(result.returncode,2,result.stdout+result.stderr)

    def test_removed_options_and_unsupported_tracking_fail_before_any_probe(self):
        options = [('--config','x'),('--udp-base','1234'),('--rtsp-base','1234'),
                   ('--runtime-dir','x'),('--partition','x'),('--prepare-only',),('--no-video',),
                   ('--tracking-backend','csrt')]
        for argv in options:
            with self.subTest(argv=argv):
                result = self.run_script(argv=['--aircraft-model',MODELS[2],*argv])
                self.assertEqual(result.returncode,2,result.stdout+result.stderr)
                self.assertFalse(self.calls())

    def test_existing_process_rejects_without_killing_it(self):
        env = dict(self.env, FIXTURE_EXISTING='1')
        result = self.run_script(argv=['--aircraft-model',MODELS[2]],env=env)
        self.assertEqual(result.returncode,1)
        self.assertTrue(self.calls('pgrep'))
        self.assertFalse(self.calls('kill'))
        self.assertFalse(self.calls('setsid'))

    def test_occupied_port_rejects_before_start_and_closes_every_probe(self):
        env = dict(self.env,FIXTURE_OCCUPIED_PORT='8881')
        result = self.run_script(argv=['--aircraft-model',MODELS[2],'-n','2'],env=env)
        self.assertEqual(result.returncode,1,result.stdout+result.stderr)
        self.assertIn('8881',result.stdout+result.stderr)
        self.assertEqual(len(self.calls('port_bind')),len(self.calls('port_close')))
        self.assertFalse(self.calls('setsid'))
        self.assertFalse(self.calls('kill'))

    def test_dead_gateway_with_ready_log_runs_global_cleanup(self):
        # 已退出网关即使留下就绪日志也必须失败；stop.sh 的 kill 仍由函数截获。
        env = dict(self.env,FIXTURE_FAIL_GATEWAY='1')
        result = self.run_script(argv=['--aircraft-model',MODELS[2]],env=env)
        self.assertEqual(result.returncode,1,result.stdout+result.stderr)
        self.assertIn('清理已启动的进程',result.stderr)
        self.assertFalse(self.runtime.exists())
        self.assertTrue(any(row['argv'][0] == '-TERM' for row in self.calls('kill')))

    def test_stop_discovers_zhongke_and_rtsp_globally_and_escalates(self):
        (self.root/'started').touch()
        (self.root/'alive').touch()
        (self.runtime/'pids').mkdir(parents=True)
        (self.runtime/'pids/gateway.pid').write_text('990001\n')
        env = dict(self.env,FIXTURE_STOP='1',FIXTURE_STUBBORN='1')
        result = self.run_script('stop.sh',env=env)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)
        patterns = '\n'.join(' '.join(row['argv']) for row in self.calls('pgrep'))
        self.assertIn('zhongke',patterns.replace('[z]','z'))
        self.assertIn('rtsp',patterns)
        signals = [row['argv'][0] for row in self.calls('kill')]
        self.assertEqual(signals.count('-TERM'),1,'重复的全局搜索结果应去重')
        self.assertEqual(signals.count('-KILL'),1)
        self.assertFalse((self.root/'alive').exists())


@unittest.skipUnless((ASSETS/'assembledmodels/rocket_zhongke_d80/tools/generate_dynamic_model.py').exists(),
                     '需要同级资产仓进行真实离线 world 集成验证')
class PreparedWorldTests(unittest.TestCase):
    def test_real_generator_cli_preserves_identity_and_hash_through_world_preparation(self):
        source_ini = ASSETS/'models/gimbals/zhongke-d80/parameters.ini'
        original_ini = source_ini.read_bytes()
        targets = ['rocket_zhongke_d80_1','rocket_zhongke_d80_2','xiaomotuo','xiaobianfu']
        generator = ASSETS/'assembledmodels/rocket_zhongke_d80/tools/generate_dynamic_model.py'
        with tempfile.TemporaryDirectory(prefix='scenario02-world-') as directory:
            root = Path(directory)
            model_args = []
            manifests = []
            for number in (1,2):
                output = root/f'generated/aircraft_{number}/rocket_zhongke_d80'
                command = [sys.executable,str(generator),'--output-dir',str(output),
                           '--model',f'rocket_zhongke_d80_{number}','--aircraft-id',str(number),
                           '--world',WORLD,'--topic-prefix',f'/uav_{number}/pod',
                           '--udp-bind','0.0.0.0','--udp-port',str(8879+number),
                           '--rtsp-bind','0.0.0.0','--rtsp-port',str(8565+number),
                           '--rtsp-mount',f'/zhongke_d80_{number}']
                for target in targets:
                    command += ['--target-name',target]
                result = subprocess.run(command,text=True,capture_output=True,timeout=60)
                self.assertEqual(result.returncode,0,result.stdout+result.stderr)
                manifests.append(json.loads((output/'manifest.json').read_text()))
                model_args += ['--model',str(number),str(output/'model.sdf')]
            world_path = root/'run/world.sdf'
            result = subprocess.run([sys.executable,str(HERE/'prepare_world.py'),
                '--source-world',str(ASSETS/f'worlds/{WORLD}/world.sdf'),
                '--output',str(world_path),'--model-basename','rocket_zhongke_d80',
                '--target-control',*model_args],text=True,capture_output=True,timeout=60)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            world = ET.parse(world_path).getroot().find('world')
            self.assertNotEqual(manifests[0]['config_hash'],manifests[1]['config_hash'])
            for number, manifest in enumerate(manifests,1):
                model = world.find(f"model[@name='rocket_zhongke_d80_{number}']")
                self.assertIsNotNone(model)
                plugin = model.find("plugin[@name='zhongke_d80::GimbalController']")
                self.assertEqual(plugin.findtext('model_name'),f'rocket_zhongke_d80_{number}')
                self.assertEqual(plugin.findtext('config_hash'),manifest['config_hash'])
                self.assertEqual([item.text for item in plugin.findall('sim_truth_target_name')],targets)
                self.assertEqual(plugin.findtext('command_topic'),f'/uav_{number}/pod/command')
                self.assertEqual(plugin.findtext('telemetry_topic'),f'/uav_{number}/pod/telemetry')
                self.assertEqual(model.findtext("link[@name='camera_link']/sensor[@name='camera']/topic"),
                                 f'/uav_{number}/pod/image')
                marker = world.find(f"model[@name='POD_LOCATION_MARKER_{number}']")
                self.assertIsNotNone(marker)
                self.assertFalse(marker.findall('.//collision'))
            self.assertEqual(source_ini.read_bytes(),original_ini)
            self.assertFalse(list(root.rglob('*.ini')))


if __name__ == '__main__':
    unittest.main()
