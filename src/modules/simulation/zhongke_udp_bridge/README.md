# 中科 D80 遥测桥

`zhongke_udp_bridge` 是中科组合机的独立 SITL 模块，将地面站转发的 JSON 发布到已有 `dyt_zhongke_telemetry` uORB 主题。

## 数据约定

- 输入包含 `msg/DytZhongkeTelemetry.msg` 的全部 15 个字段：`timestamp` 和 14 个中科测量字段。缺失、顶层重复字段、非法类型、非有限数值、截断 JSON 和超长数据报均拒绝发布。
- `status1`、`status2`、`status3`、`self_check` 保留原始字节，必须为 0～255 的整数字面量，不接受小数、指数、布尔值或字符串。
- 其他测量必须为有限且可表示为 `float32` 的数值。角度为 deg、角速度为 deg/s、距离为 m、`zoom` 为倍率；地面站已完成二进制比例换算，桥直接映射这些物理量。
- 输入 `timestamp` 是地面站实际发送 JSON 时生成的 Unix 整数毫秒，接受 0～18446744073709551615；uORB `timestamp` 使用 PX4 `hrt_absolute_time()` 的启动后微秒。
- 未知扩展值可以包含对象、数组和字符串，最多嵌套 8 层；顶层最多 32 个字段，解码后的顶层字段名最多 63 个 UTF-8 字节。JSON 字符串校验 UTF-8 和 Unicode 转义，顶层转义同名字段也视为重复。
- 单个数据报最大 4095 字节，使用 `MSG_TRUNC` 检测 UDP 截断，超长包即使开头包含完整 JSON 也不发布。
- 默认独占监听 `0.0.0.0:9000`，可用 `-p` 覆盖；不接受二进制遥测或艾晟 JSON。

## 使用

```sh
zhongke_udp_bridge start -p 9000
zhongke_udp_bridge status
listener dyt_zhongke_telemetry
zhongke_udp_bridge stop
```

02 场景根据 `--aircraft-model rocket_zhongke_d80` 为每个 PX4 实例启动此模块，逐机监听 `9000 + aircraft_id - 1`。在现有中科地面站界面手动设置 JSON 目的地址为仿真主机、端口为该飞机的端口，再点击发送。地面站未开启转发时桥保持监听；各实例需要使用各自的目的端口。

## 验证

```sh
python3 src/modules/simulation/zhongke_udp_bridge/test_json_parser.py
make px4_sitl_default
python3 src/modules/simulation/zhongke_udp_bridge/test_sitl_udp.py
```

测试编译生产解析器、UDP 接收和发布函数，验证全部字段及单位、整数与浮点边界、PX4 时间戳、协议隔离、缺失与重复字段、未知扩展限制、实际 UDP 截断处理、独占端口和发布失败状态。

`test_sitl_udp.py` 使用两个真实 PX4 SITL 实例和本地回环 UDP，验证逐机发布隔离、艾晟主题未发布、坏包及超长包计数、独占端口和停止后释放。测试不启动 Gazebo 或飞行模块；没有仿真时钟推进时，lockstep 模式的 `hrt_absolute_time()` 为 0，测试按该时钟状态核对时间戳。
