"""中科02入口的离线、端口和进程归属回归；不启动世界或终止真实进程。"""
from __future__ import annotations

import configparser
import importlib.util
import json
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from unittest.mock import patch

HERE = Path(__file__).resolve().parent
ASSETS = HERE.parents[1].parent / 'px4-gazebo-assets'


def load_module():
    spec = importlib.util.spec_from_file_location('scenario02_zhongke_launcher', HERE / 'start_zhongke.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


LAUNCHER = load_module()


class LegacyDispatchTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='scenario02-legacy-check-')
        self.directory = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def parser_snapshot(self, argv):
        # 运行真实参数解析及原机型case块，在任何pgrep/git/构建/进程操作之前截断。
        source = (HERE / 'start.sh').read_text()
        prefix = source.split("if pgrep -f ", 1)[0]
        block = re.search(r'case "\$AIRCRAFT_MODEL" in\n.*?\nesac', source, re.S).group()
        capture = '\nprintf "%s\\n" "$AIRCRAFT_MODEL" "$COUNT" "$GUI" "$TRACKING_BACKEND" "$BRIDGE_KIND" "$CONTROL_PORT_BASE" "$PX4_UDP_BRIDGE" "$MODEL_DIR"\n'
        script = self.directory / 'parser.sh'
        script.write_text(prefix + block + capture)
        env = dict(os.environ, PX4_GAZEBO_ASSETS_ROOT='/fixture/assets')
        completed = subprocess.run(['bash', str(script), *argv], env=env, text=True, capture_output=True)
        self.assertEqual(completed.returncode, 0, completed.stderr)
        return completed.stdout.splitlines()

    def test_old_default_and_explicit_two_axis_are_equivalent(self):
        expected = ['rocket_two_axis_pod', '1', '0', 'sim_truth', 'two_axis', '8770',
                    'dyt_udp_bridge', 'assembledmodels/rocket_two_axis_pod']
        self.assertEqual(self.parser_snapshot([]), expected)
        self.assertEqual(self.parser_snapshot(['--aircraft-model', 'rocket_two_axis_pod']), expected)

    def test_aisheng_and_legacy_options_keep_their_contract(self):
        self.assertEqual(self.parser_snapshot(['--aircraft-model', 'rocket_aisheng_seeker',
                         '-n', '2', '--gui', '--tracking-backend', 'csrt']),
                         ['rocket_aisheng_seeker', '2', '1', 'csrt', 'aisheng', '8001',
                          'aisheng_udp_bridge', 'assembledmodels/rocket_aisheng_seeker'])

    def test_new_dispatch_precedes_all_global_process_probes(self):
        capture = self.directory / 'argv.json'
        python = self.directory / 'capture-python'
        python.write_text('#!/usr/bin/env python3\nimport json,os,sys\nopen(os.environ["CHECK_ARGV"],"w").write(json.dumps(sys.argv[1:]))\n')
        python.chmod(0o755)
        sentinel = self.directory / 'pgrep-ran'
        pgrep = self.directory / 'pgrep'
        pgrep.write_text('#!/bin/sh\ntouch "$CHECK_PGREP"\nexit 99\n')
        pgrep.chmod(0o755)
        env = dict(os.environ, GIMBAL_PYTHON=str(python), CHECK_ARGV=str(capture),
                   CHECK_PGREP=str(sentinel), PATH=str(self.directory)+os.pathsep+os.environ['PATH'])
        completed = subprocess.run(['bash', str(HERE / 'start.sh'), '--aircraft-model',
            'rocket_zhongke_d80', '-n', '2', '--prepare-only', '--runtime-dir', str(self.directory/'runtime')],
            env=env, text=True, capture_output=True)
        self.assertEqual(completed.returncode, 0, completed.stderr)
        argv = json.loads(capture.read_text())
        self.assertEqual(Path(argv[0]), HERE/'start_zhongke.py')
        self.assertEqual(argv[argv.index('--count')+1], '2')
        self.assertIn('--prepare-only', argv)
        self.assertFalse(sentinel.exists(), '中科分派触发了旧全局pgrep')

    def test_old_stop_body_is_preserved_after_explicit_dispatch(self):
        source = (HERE/'stop.sh').read_text()
        self.assertIn('RUNTIME_DIR="${TMPDIR:-/tmp}/px4-scenarios/02_rocket_two_axis_pod"', source)
        self.assertLess(source.index('rocket_zhongke_d80'), source.index('declare -A seen=()'))
        self.assertIn('exec "${GIMBAL_PYTHON:-python3}" "$SCRIPT_DIR/start_zhongke.py" --stop "$@"', source)


class LauncherBoundaryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='scenario02-zhongke-check-')
        self.directory = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def manifest(self):
        value = configparser.ConfigParser(interpolation=None)
        value['session'] = {'kind':'scenario02_zhongke_d80', 'state':'ready'}
        value['aircraft 1'] = {'udp_port':'38080', 'mavsdk_udp':'14280',
                               'px4_instance':'0', 'rtsp_port':'38566'}
        return value

    def test_occupied_real_control_port_rejects_without_killing_owner(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as owner:
            owner.bind(('127.0.0.1', 0))
            manifest = self.manifest()
            manifest['aircraft 1']['udp_port'] = str(owner.getsockname()[1])
            with patch.object(LAUNCHER.os, 'kill') as kill:
                with self.assertRaisesRegex(ValueError, 'UDP端口.*已占用'):
                    LAUNCHER.check_ports(manifest, no_video=True)
                kill.assert_not_called()
            self.assertGreaterEqual(owner.fileno(), 0)

    def test_all_px4_local_ports_are_checked_and_probe_sockets_closed(self):
        made = []
        class ProbeSocket:
            def __init__(self, *_):
                self.closed = False
                made.append(self)
            def bind(self, endpoint):
                if endpoint[1] == 13030:
                    raise OSError('测试端口已占用')
            def close(self):
                self.closed = True
        with patch.object(LAUNCHER.socket, 'socket', ProbeSocket):
            with self.assertRaisesRegex(ValueError, '13030'):
                LAUNCHER.check_ports(self.manifest(), no_video=True)
        self.assertTrue(made)
        self.assertTrue(all(item.closed for item in made))

    def test_stale_ready_log_never_hides_dead_new_process(self):
        (self.directory/'logs').mkdir()
        (self.directory/'logs/gateway_1.log').write_text('中科网关启动\n')
        manifest = self.manifest()
        manifest['process gateway_1'] = {'pid':'999999', 'start_ticks':'100'}
        with patch.object(LAUNCHER, 'identity_alive', return_value=False):
            with self.assertRaisesRegex(ValueError, '启动失败'):
                LAUNCHER.wait_log(self.directory, manifest, 'gateway_1', '中科网关启动', .1)

    def test_start_truncates_old_log_before_new_process(self):
        (self.directory/'logs').mkdir()
        path = self.directory/'logs/gateway_1.log'
        path.write_text('中科网关启动\n')
        class NewProcess:
            pid = 999999
        def fake_popen(*args, **kwargs):
            self.assertEqual(path.read_bytes(), b'')
            return NewProcess()
        with patch.object(LAUNCHER.subprocess, 'Popen', side_effect=fake_popen), \
                patch.object(LAUNCHER, 'record_process') as record:
            LAUNCHER.start_process(self.directory, self.manifest(), 'gateway_1', ['fixture'], {})
            record.assert_called_once()

    def test_failure_cleanup_is_limited_to_selected_runtime(self):
        runtime = self.directory/'owned'
        manifest = self.manifest()
        with patch.object(LAUNCHER, 'preflight_dependencies'), \
                patch.object(LAUNCHER, 'prepare_runtime', return_value=manifest), \
                patch.object(LAUNCHER, 'start_runtime', side_effect=ValueError('测试启动失败')), \
                patch.object(LAUNCHER, 'stop_runtime') as stop:
            self.assertEqual(LAUNCHER.main(['--runtime-dir', str(runtime), '--no-video']), 1)
            stop.assert_called_once_with(str(runtime))

    def test_stop_ignores_reused_pid_and_never_uses_global_process_names(self):
        manifest = self.manifest()
        manifest['process owned'] = {'pid':'999991', 'start_ticks':'123'}
        manifest['process reused'] = {'pid':'999992', 'start_ticks':'234'}
        LAUNCHER.write_manifest(self.directory, manifest)
        alive = {'999991'}
        def identity_alive(identity):
            return identity.get('pid') in alive and identity.get('start_ticks') == '123'
        def kill(pid, _):
            alive.discard(str(pid))
        with patch.object(LAUNCHER, 'identity_alive', side_effect=identity_alive), \
                patch.object(LAUNCHER.Path, 'glob', return_value=[]), \
                patch.object(LAUNCHER.os, 'kill', side_effect=kill) as sent:
            LAUNCHER.stop_runtime(self.directory)
            self.assertEqual(sent.call_args_list[0].args, (999991, signal.SIGTERM))
            self.assertTrue(all(call.args[0] == 999991 for call in sent.call_args_list))
            sent.reset_mock()
            LAUNCHER.stop_runtime(self.directory)
            sent.assert_not_called()
        self.assertEqual(LAUNCHER.read_manifest(self.directory)['session']['state'], 'stopped')

    def test_nonempty_unowned_runtime_is_rejected_before_any_overwrite(self):
        runtime = self.directory/'legacy-run'
        runtime.mkdir()
        sentinel = runtime/'world.sdf'
        sentinel.write_text('现有场景内容')
        args = LAUNCHER.parse_args(['--prepare-only', '--assets-root', str(ASSETS),
                                   '--runtime-dir', str(runtime)])
        with patch.object(LAUNCHER, 'checked') as command:
            with self.assertRaises(ValueError):
                LAUNCHER.prepare_runtime(args)
            command.assert_not_called()
        self.assertEqual(sentinel.read_text(), '现有场景内容')
        self.assertFalse((runtime/'session.ini').exists())

    def test_legacy_partition_is_rejected(self):
        with self.assertRaises(SystemExit):
            LAUNCHER.parse_args(['--partition', 'px4_scenario_02_rocket_two_axis_pod'])

    def test_default_python_uses_existing_component_virtual_environment(self):
        virtual = self.directory/'assets/models/gimbals/.venv-zhongke-d80/bin/python'
        virtual.parent.mkdir(parents=True)
        virtual.touch()
        with patch.dict(os.environ, {}, clear=True):
            args = LAUNCHER.parse_args(['--assets-root', str(self.directory/'assets')])
        self.assertEqual(args.python, str(virtual))

    def test_explicit_environment_and_cli_interpreters_have_precedence(self):
        with patch.dict(os.environ, {'GIMBAL_PYTHON':'/fixture/env-python'}):
            self.assertEqual(LAUNCHER.parse_args([]).python, '/fixture/env-python')
            self.assertEqual(LAUNCHER.parse_args(['--python','/fixture/cli-python']).python,
                             '/fixture/cli-python')

    def test_dependency_failure_never_prepares_starts_or_stops_a_scene(self):
        runtime = self.directory/'not-created'
        with patch.object(LAUNCHER, 'preflight_dependencies', side_effect=ValueError('测试依赖缺失')), \
                patch.object(LAUNCHER, 'prepare_runtime') as prepare, \
                patch.object(LAUNCHER, 'start_runtime') as start, \
                patch.object(LAUNCHER, 'stop_runtime') as stop:
            self.assertEqual(LAUNCHER.main(['--runtime-dir', str(runtime)]), 1)
            prepare.assert_not_called()
            start.assert_not_called()
            stop.assert_not_called()
        self.assertFalse(runtime.exists())

    def test_failed_dependency_probe_reports_interpreter_and_setup_entry(self):
        args = LAUNCHER.parse_args(['--python','/fixture/missing-python'])
        failed = subprocess.CompletedProcess([], 1, stdout='', stderr='ModuleNotFoundError: gi')
        with patch.object(LAUNCHER.subprocess, 'run', return_value=failed):
            with self.assertRaisesRegex(ValueError, 'missing-python.*setup.sh.*未启动场景进程'):
                LAUNCHER.preflight_dependencies(args)

    def test_direct_python_rejects_target_sysid_conflicts(self):
        for argv in (['-n','2','--target-sysid','2'], ['--target-sysid','255']):
            with self.subTest(argv=argv), self.assertRaises(SystemExit):
                LAUNCHER.parse_args(argv)


class PreparedIdentityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix='scenario02-zhongke-prepare-')
        cls.directory = Path(cls.temp.name)
        cls.source = ASSETS/'models/gimbals/zhongke-d80/parameters.ini'
        cls.source_bytes = cls.source.read_bytes()
        args = LAUNCHER.parse_args(['--prepare-only', '-n','2', '--assets-root',str(ASSETS),
            '--runtime-dir',str(cls.directory/'run'), '--python',sys.executable,
            '--udp-base','38880','--rtsp-base','38566'])
        cls.manifest = LAUNCHER.prepare_runtime(args)
        cls.world = ET.parse(cls.directory/'run/world.sdf').getroot().find('world')

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_source_ini_is_unchanged_and_configs_are_external_per_aircraft(self):
        self.assertEqual(self.source.read_bytes(), self.source_bytes)
        paths = [Path(self.manifest[f'aircraft {i}']['config_path']) for i in (1,2)]
        self.assertNotEqual(paths[0], paths[1])
        self.assertTrue(all(path.is_relative_to(self.directory) for path in paths))
        self.assertNotEqual(self.manifest['aircraft 1']['config_hash'], self.manifest['aircraft 2']['config_hash'])

    def test_model_config_hash_targets_video_and_diagnostics_share_identity(self):
        component = ASSETS/'models/gimbals/zhongke-d80'
        sys.path.insert(0, str(component))
        from common.config import load_config
        expected_targets = ['rocket_zhongke_d80_1','rocket_zhongke_d80_2','xiaomotuo','xiaobianfu']
        for number in (1,2):
            with self.subTest(aircraft=number):
                entry = self.manifest[f'aircraft {number}']
                cfg = load_config(entry['config_path'])
                model = self.world.find(f"model[@name='{entry['entity']}']")
                self.assertIsNotNone(model)
                plugin = model.find("plugin[@name='zhongke_d80::GimbalController']")
                self.assertEqual(cfg['model']['name'], entry['entity'])
                self.assertEqual(plugin.findtext('model_name'), entry['entity'])
                self.assertEqual(plugin.findtext('config_hash'), cfg['config_hash'])
                self.assertEqual(entry['config_hash'], cfg['config_hash'])
                self.assertEqual(cfg['tracking']['target_names'], expected_targets)
                self.assertEqual([v.text for v in plugin.findall('sim_truth_target_name')], expected_targets)
                self.assertEqual(plugin.findtext('command_topic'), entry['topic_prefix']+'/command')
                self.assertEqual(plugin.findtext('telemetry_topic'), entry['telemetry_topic'])
                self.assertEqual(plugin.findtext('image_topic'), entry['image_topic'])
                camera = model.find("link[@name='camera_link']/sensor[@name='camera']")
                self.assertEqual(camera.findtext('topic'), entry['image_topic'])
                self.assertEqual(entry['diagnostics_topic'], entry['topic_prefix']+'/diagnostics')
                station = configparser.ConfigParser(interpolation=None)
                station.read(entry['station_config_path'])
                self.assertEqual(station['connection']['parameters_file'], entry['config_path'])
                self.assertEqual(station['connection']['udp_port'], entry['udp_port'])
                self.assertEqual(station['connection']['rtsp_url'], entry['rtsp_url'])
                self.assertEqual(cfg['video']['rtsp_port'], int(entry['rtsp_port']))
                self.assertEqual(cfg['video']['rtsp_mount'], entry['rtsp_mount'])
                marker = self.world.find(f"model[@name='POD_LOCATION_MARKER_{number}']")
                self.assertIsNotNone(marker)
                self.assertFalse(marker.findall('.//collision'))

    def test_runtime_partition_and_endpoints_are_independent_of_legacy_scene(self):
        section = self.manifest['session']
        self.assertIn('zhongke', section['gz_partition'])
        self.assertNotEqual(section['gz_partition'], 'px4_scenario_02_rocket_two_axis_pod')
        self.assertNotEqual(Path(section['runtime_dir']), Path('/tmp/px4-scenarios/02_rocket_two_axis_pod'))
        self.assertEqual([self.manifest[f'aircraft {i}']['udp_port'] for i in (1,2)], ['38880','38881'])
        self.assertEqual([self.manifest[f'aircraft {i}']['rtsp_port'] for i in (1,2)], ['38566','38567'])


if __name__ == '__main__':
    unittest.main()
