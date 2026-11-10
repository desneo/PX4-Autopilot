# A2416A 目标锁定验收

`check_jielian_tracking.py` 连接已运行的02场景，不另行启动或停止PX4。工具使用场景同款
`xiaomotuo` 视觉模型创建真实Gazebo实体，通过 `/world/<world>/set_pose` 将其放到
`rocket_jielian_a2416a_1` 固定 `+Z` 光轴前方500 m，并按以下顺序验收：

1. 接收插件发布的50 Hz真实测量和网关发布的100 Hz、104字节标准遥测；
2. 通过45字节标准控制帧开启识别和自动锁定；
3. 将目标从图像左侧横穿到右侧，核对同一实体持续锁定以及偏航失调角符号；
4. 将目标移出37度水平视场，核对 `TEMP_LOST` 仍绑定原目标；
5. 在2.5 s窗口内将目标移回视场，核对同一目标恢复为 `TRACKING`。

工具不注入插件遥测。判据同时取自 Gazebo 插件真值测量和网关二进制遥测，原始样本保存为
`jielian_tracking_samples.jsonl.gz`，结论保存为 `jielian_tracking_result.json`。

启动一架A2416A装配机并等待场景就绪后运行：

```bash
cd /home/aeroarx/zsh/PX4-Autopilot/scenarios/02_rocket_two_axis_pod
python3 check_jielian_tracking.py \
  --partition px4_scenario_02_rocket_two_axis_pod \
  --world geo_32_217857_119_105878 \
  --aircraft-model rocket_jielian_a2416a_1 \
  --topic-prefix /uav_1/pod \
  --udp-host 127.0.0.1 --udp-port 8001 \
  --output /tmp/jielian-a2416a-scenario02-acceptance
```

多机验收时，将飞机实体、topic前缀和控制端口一起改为对应实例，例如第二架使用
`rocket_jielian_a2416a_2`、`/uav_2/pod` 和 `8002`。工具要求测试开始前不存在
`xiaomotuo`；若人工靶机已经运行，先在02界面终止它。工具只删除带有本次唯一标志的自建
目标，不会删除同名的人工目标。

离线契约测试不启动Gazebo：

```bash
python3 -m unittest -v test_jielian_tracking_acceptance.py
```
