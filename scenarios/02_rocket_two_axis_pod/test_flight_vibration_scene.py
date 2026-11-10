"""02场景抖动开关的真实资产离线集成检查，不启动Gazebo。"""
from __future__ import annotations

import copy
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
ASSETS = HERE.parents[2] / 'px4-gazebo-assets'
WORLD = 'geo_32_217857_119_105878'
MODELS = ('rocket_two_axis_pod', 'rocket_aisheng_seeker', 'rocket_zhongke_d80',
          'rocket_jielian_a2416a')
SPEC = importlib.util.spec_from_file_location('vibration_scene_prepare', HERE/'prepare_world.py')
PREPARE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PREPARE)


def normalized(element):
    result = copy.deepcopy(element)
    for item in result.iter():
        item.tail = None
        if item.text is not None:
            item.text = item.text.strip() or None
    return ET.tostring(result)


@unittest.skipUnless((ASSETS/'models/vehicles/rocket_racing_quad/tools/configure_vibration.py').is_file(),
                     '需要同级资产仓的可选火箭抖动能力')
class FlightVibrationSceneTests(unittest.TestCase):
    def test_real_models_keep_pod_collision_and_tracking_with_isolated_imu_topics(self):
        source = ASSETS/f'worlds/{WORLD}/world.sdf'
        with tempfile.TemporaryDirectory(prefix='scenario02-vibration-') as directory:
            root = Path(directory)
            for basename in MODELS:
                with self.subTest(model=basename):
                    targets = [f'{basename}_1',f'{basename}_2','xiaomotuo','xiaobianfu']
                    models = []
                    originals = []
                    for number in (1,2):
                        generated = root/f'{basename}/aircraft_{number}/{basename}'
                        command = [sys.executable,str(ASSETS/f'assembledmodels/{basename}/tools/generate_dynamic_model.py'),
                                   '--output-dir',str(generated),'--topic-prefix',f'/uav_{number}/pod']
                        if basename == 'rocket_zhongke_d80':
                            command += ['--model',f'{basename}_{number}','--aircraft-id',str(number),'--world',WORLD,
                                        '--udp-bind','0.0.0.0','--udp-port',str(8879+number),
                                        '--rtsp-bind','0.0.0.0','--rtsp-port',str(8565+number),
                                        '--rtsp-mount',f'/zhongke_d80_{number}']
                            for target in targets:
                                command += ['--target-name',target]
                        result = subprocess.run(command,text=True,capture_output=True,timeout=60)
                        self.assertEqual(result.returncode,0,result.stdout+result.stderr)
                        path = generated/'model.sdf'
                        models.append((number,path))
                        originals.append(path.read_bytes())
                    outputs = {mode:root/basename/mode/'world.sdf' for mode in ('default','off','afly5')}
                    # 关闭时连可选工具都不加载，兼容未提供抖动能力的旧资产。
                    with patch.object(PREPARE,'load_flight_vibration',side_effect=AssertionError('off loaded tool')):
                        PREPARE.prepare(source,outputs['default'],models,model_basename=basename,target_control=True)
                        PREPARE.prepare(source,outputs['off'],models,model_basename=basename,target_control=True,
                                        flight_vibration='off')
                    self.assertEqual(outputs['default'].read_bytes(),outputs['off'].read_bytes())
                    self.assertFalse(outputs['off'].with_name('flight_vibration.json').exists())
                    PREPARE.prepare(source,outputs['afly5'],models,model_basename=basename,target_control=True,
                                    flight_vibration='afly5')
                    baseline = ET.parse(outputs['off']).getroot().find('world')
                    enabled = ET.parse(outputs['afly5']).getroot().find('world')
                    self.assertLessEqual(float(enabled.findtext('physics/max_step_size')),.001)
                    topics = set()
                    for number in (1,2):
                        name = f'{basename}_{number}'
                        plain = baseline.find(f"model[@name='{name}']")
                        model = enabled.find(f"model[@name='{name}']")
                        plugin = model.find("plugin[@name='rocket::FlightVibration']")
                        self.assertIsNotNone(plugin)
                        canonical = f'/world/{WORLD}/model/{name}/link/base_link/sensor/imu_sensor/imu'
                        self.assertEqual(plugin.findtext('output_imu_topic'),canonical)
                        self.assertNotIn(canonical,topics)
                        topics.add(canonical)
                        sensor = model.find("link[@name='base_link']/sensor[@name='imu_sensor']")
                        self.assertEqual(sensor.findtext('topic'),plugin.findtext('raw_imu_topic'))
                        self.assertGreaterEqual(float(sensor.findtext('update_rate')),1000)
                        # 允许差异只有新增机体插件及IMU topic/update_rate；其余树保持一致。
                        restored = copy.deepcopy(model)
                        restored.remove(restored.find("plugin[@name='rocket::FlightVibration']"))
                        changed_imu = restored.find("link[@name='base_link']/sensor[@name='imu_sensor']")
                        original_imu = plain.find("link[@name='base_link']/sensor[@name='imu_sensor']")
                        for tag in ('topic','update_rate'):
                            old = original_imu.find(tag)
                            changed = changed_imu.find(tag)
                            if old is None:
                                changed_imu.remove(changed)
                            else:
                                changed.text = old.text
                        self.assertEqual(normalized(restored),normalized(plain))
                    # 世界只允许physics及飞机节点发生上述差异；GUI/靶机/雷达/碰撞插件不变。
                    restored_world = copy.deepcopy(enabled)
                    for number in (1,2):
                        name = f'{basename}_{number}'
                        item = restored_world.find(f"model[@name='{name}']")
                        index = list(restored_world).index(item)
                        restored_world.remove(item)
                        restored_world.insert(index,copy.deepcopy(baseline.find(f"model[@name='{name}']")))
                    item = restored_world.find('physics')
                    index = list(restored_world).index(item)
                    restored_world.remove(item)
                    restored_world.insert(index,copy.deepcopy(baseline.find('physics')))
                    self.assertEqual(normalized(restored_world),normalized(baseline))
                    summary = json.loads(outputs['afly5'].with_name('flight_vibration.json').read_text())
                    self.assertEqual(summary['profile'],'afly5')
                    self.assertEqual(len(summary['models']),2)
                    self.assertEqual(len(summary['profile_sha256']),64)
                    self.assertEqual(summary['profile_values']['schema_version'],1)
                    self.assertLessEqual(summary['world_max_step_size_s'],.001)
                    # 复用输出位置时关掉开关，必须清除旧摘要且恢复原生world。
                    PREPARE.prepare(source,outputs['afly5'],models,model_basename=basename,
                                    target_control=True,flight_vibration='off')
                    self.assertEqual(outputs['afly5'].read_bytes(),outputs['off'].read_bytes())
                    self.assertFalse(outputs['afly5'].with_name('flight_vibration.json').exists())
                    for (_,path),original in zip(models,originals):
                        self.assertEqual(path.read_bytes(),original,'不可改写源生成模型')


if __name__ == '__main__':
    unittest.main()
