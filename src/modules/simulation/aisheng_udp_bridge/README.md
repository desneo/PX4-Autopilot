# ASD-2-075-DD0 完整遥测桥

`aisheng_udp_bridge` 是艾晟组合机的独立 SITL 模块，将地面站解析的完整 JSON 发布到 `dyt_aisheng_telemetry`。`rocket_two_axis_pod` 使用自身的 `dyt_udp_bridge`，两个模块的解析器与启动选择独立。

## 数据约定

- 字段以 `msg/DytAishengTelemetry.msg` 为准，除 `los_omega_n/e/d` 外共 37 项，全部必需。未知扩展字段可跳过，重复字段、缺项、非法数值和超长报文会拒绝发布。
- `target_type`、`target_confidence`、`target_lon`、`target_lat`、`target_alt` 保留四元素数组。
- 经纬度使用原始 int32，单位 1e-7°；其余比例、单位和状态位按消息定义。
- 输入 `timestamp` 是地面站接收帧时的 monotonic 微秒；发布时由 PX4 `hrt_absolute_time()` 生成自身时间戳。`device_timestamp` 保留设备 uint32 毫秒。
- 标准帧未提供三个 LOS 向量分量，输入不发送这三项，uORB 中设为 `NaN`。
- 默认监听 UDP 9000，可用 `-p` 指定；端口为独占绑定。

## 使用

```sh
aisheng_udp_bridge start -p 9000
aisheng_udp_bridge status
listener dyt_aisheng_telemetry
aisheng_udp_bridge stop
```

02 场景根据 `--aircraft-model rocket_aisheng_seeker` 自动选择此模块，并按飞机编号分配端口。地面站报文面板预置相应目的端点，点击发送后才向桥转发。

## 验证

```sh
python3 src/modules/simulation/aisheng_udp_bridge/test_json_parser.py
make px4_sitl_default
```

测试编译实际解析器与发布函数，核对全部字段、四目标数组、整数边界、状态位、缺失/非法数据和 PX4 时间戳及缺省 LOS 向量。
