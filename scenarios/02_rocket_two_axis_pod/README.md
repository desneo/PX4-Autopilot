# 场景 02：火箭穿越机与两轴吊仓

在资产仓的 `geo_32_217857_119_105878` world 中启动火箭穿越机与顶部两轴吊仓组合模型。所有飞机使用同一个 world，默认启动一架、不开启 Gazebo GUI，启动后保持待命，不自动解锁或飞行。

## 启动和停止

```bash
# 默认一架
bash scenarios/02_rocket_two_axis_pod/start.sh

# 四架，打开 Gazebo GUI
bash scenarios/02_rocket_two_axis_pod/start.sh -n 4 --gui

# QGC 在远程机器时，显式指定 QGC 的 UDP 地址
bash scenarios/02_rocket_two_axis_pod/start.sh --gui \
  --target-qgc-address 192.168.1.100:14550

# 打印给局域网客户端使用的指定地址
bash scenarios/02_rocket_two_axis_pod/start.sh -n 2 \
  --advertise-host 192.168.1.56

bash scenarios/02_rocket_two_axis_pod/stop.sh
```

`-n` 支持 1～10，默认为 1。飞机在 world 中按 5 米间隔排列。`GZ_IP=127.0.0.1` 只限制 Gazebo Transport 使用回环接口，不影响 MAVLink、吊仓 UDP 和 RTSP 的局域网访问。

未指定 `--advertise-host` 时，脚本读取系统主默认路由接口上的全局 IPv4；它不会使用 mihomo 等代理策略路由的 `198.18.0.0/15` 地址。若机器有多条默认路由，可显式传入 `--advertise-host 192.168.1.56`。

## 资产仓

资产仓默认位于 PX4 仓库同级的 `../px4-gazebo-assets`。首次启动时自动下载：

```text
https://github.com/desneo/px4-gazebo-assets.git
```

以后每次启动都会在干净的 `main` 分支执行 `git pull --ff-only origin main`，成功后才生成运行模型。下载或同步失败、分支不是 `main`、存在未提交修改，脚本都会报告原因并停止，不会执行 `reset` 或覆盖本地修改。

可用环境变量覆盖位置和下载地址：

```bash
PX4_GAZEBO_ASSETS_ROOT=/path/to/px4-gazebo-assets \
PX4_GAZEBO_ASSETS_URL=https://example/model-repo.git \
  bash scenarios/02_rocket_two_axis_pod/start.sh
```

脚本会增量构建 PX4 SITL、两轴吊仓插件和本场景的碰撞监控插件；使用 `--gui` 时也会增量构建 world 的地图中心 GUI 插件。

## 靶机控制

使用 `--gui` 启动时，Gazebo 界面右下角显示“靶机控制”面板。面板的“启用靶机”下拉框包含“不启用”“小摩托”和“小蝙蝠”。选择机型后显示巡航速度、海拔高度、来源距离、来源方向、地面站地址以及启动/终止按钮。场景默认无 GUI，因此不启动靶机管理服务；无 GUI 的既有用法保持不变。

面板固定在 3D 地图右下角，不能拖动或关闭；蓝色标题栏只保留折叠/展开按钮，避免误操作改变布局或在靶机运行时关闭控制入口。

靶机使用资产仓模型的视觉部分，由 Gazebo 场景航迹插件按仿真时间直接控制位姿；它不启动独立 PX4、不运行传感器或气动插件，也不占用 MAVLink 端口。同一时间只允许一架靶机。重复点击“启动靶机”会提示先终止当前靶机；点击“终止靶机”会从 world 删除模型，并立即停止位置数据发送。普通火箭机和吊仓仍使用真实 PX4 SITL，不受影响。

机型切换时自动设置经过模型飞行验证的默认巡航速度，并在界面显示有效范围：

- 小摩托：默认 `35 m/s`，允许 `25～65 m/s`；
- 小蝙蝠：默认 `18 m/s`，允许 `10～30 m/s`。

速度、距离、地址格式或其他参数无效时，界面和后端都会提示原因并阻止启动。界面显示来源距离范围提示，允许 `100～10000 m`，默认 `2000 m`。海拔高度使用 MSL，默认值是 world 中心海拔加100米，即当前 world 约 `104.02 m`；输入高度还必须高于整段航线最高地形至少30米。

“从东向西”在中心正东方生成靶机，“从东北45度”在中心东北方向生成靶机。模型直接生成在配置的海拔高度，随后按配置巡航速度直线飞向 world 中心；穿过中心后继续飞行500米，并在终点保持静止，直到终止靶机。该模式用于场景目标和吊仓搜索测试，不模拟靶机自身的起飞、姿态动力学或飞控故障。

地面站地址默认为 `192.168.1.41:7788`。靶机有效定位建立后，以 1 Hz 向该地址发送一个 UTF-8 UDP JSON 数据报；`altitude` 为 MSL 米，`speed` 为实时水平地速 m/s：

```json
{"type":"target_position","timestamp_ms":1792108800000,"model":"xiaomotuo","latitude":32.217857,"longitude":119.105878,"altitude":104.02,"speed":34.8}
```

启用 GUI 靶机功能需要系统 Python 能导入 `gz.transport13` 和 `gz.msgs10`，不需要 `pymavlink`。靶机位姿按每个仿真步更新，管理服务以20 Hz接收位置状态，UDP 经纬高仍以1 Hz发送。

靶机同时通过轻量 MAVLink 代理出现在 QGroundControl 中。默认发送到 `127.0.0.1:14550`，使用独立 `SYSID=200`；可通过 `--target-qgc-address HOST:PORT` 和 `--target-sysid ID` 修改。代理只发送 `HEARTBEAT`、`GLOBAL_POSITION_INT`、`GPS_RAW_INT` 和 `VFR_HUD`，不监听或执行 QGC 控制指令，也不占用普通飞机的 `14280+i` 端口。启动脚本会拒绝与当前普通飞机 `SYSID 1～n` 重叠的靶机 SYSID。终止靶机后代理立即停止发送，QGC 会在心跳超时后将该靶机显示为断开或移除。

## 碰撞致损

场景默认启用确定性的 `damage-on-contact`，用于发现集群路径规划中的碰撞问题。致损服务直接调用对应 SITL 实例的 PX4 内置故障注入命令，将全部电机设为关闭；它不占用 MAVSDK UDP 端口，也不会改变外部控制程序的连接。

- PX4 与 PX4：使用 Gazebo 原生碰撞体和接触检测；发生实际接触后，两架飞机都关闭全部电机。
- PX4 与地面或建筑物：使用 Gazebo 原生碰撞检测；飞机确认离地1米后才启用该规则，避免出生点的地面接触触发故障。明确处于 `AUTO_LAND` 或 `AUTO_PRECLAND` 的正常降落接触被豁免，其他飞行中表面接触会关闭全部电机。
- PX4 与靶机：靶机继续采用轻量场景航迹，不参与刚体碰撞。监控器使用与模型尺寸匹配的简化包围盒判定相交，并按速度矢量差计算相对速度：`|V_px4 - V_target|`。相对速度 **大于或等于10 m/s** 才关闭 PX4 的全部电机；低于 `10 m/s` 只记录事件，不触发坠毁。

PX4 与靶机达到致损条件后，靶机冻结在碰撞位置，状态变为“已碰撞”，立即停止航迹、1 Hz JSON 遥测和虚拟 MAVLink 遥测，但不模拟自由落体。模型保留在 world 中，点击“终止靶机”后才清理。

当前地理 world 的地面和高程结构统一包含在 `tile_*::terrain::ground` 网格碰撞体中，因此事件中不区分“地面”和“建筑物”，两者执行相同致损策略。重复接触不会重复注入故障。

所有碰撞事件写入：

```text
/tmp/px4-scenarios/02_rocket_two_axis_pod/logs/collision-events.jsonl
```

每行是一个 JSON 对象，包含碰撞类型、仿真时间、飞机编号、相对速度、ENU/WGS84 位置、是否触发致损以及各 PX4 实例的故障注入结果。低速靶机碰撞会记录 `reason=relative_speed_below_10_m_s`，正常降落豁免会记录 `reason=normal_landing_mode`。

## 多机接口

第 `i` 架飞机使用：

- PX4/MAVSDK UDP：`14280 + i - 1`；
- 吊仓指令与遥测 UDP：`8770 + i - 1`；
- 共享 RTSP 端口：`8555`；
- 视频路径：`/two_axis_pod_i`；
- Gazebo 吊仓话题：`/uav_i/pod/*`。

例如两架飞机：

```text
飞机1：udp://192.168.1.56:8770
       rtsp://192.168.1.56:8555/two_axis_pod_1
飞机2：udp://192.168.1.56:8771
       rtsp://192.168.1.56:8555/two_axis_pod_2
```

每架飞机启动前都会输出完整 PX4、吊仓网关和地面站命令，并将对应关系保存到运行目录的 `interfaces.tsv`。RTSP 内部 RTP 端口由资产仓网关自动分配。

视频默认使用兼容性更好的 `x264`，可使用 `--encoder nvenc` 选择 NVIDIA 硬件编码。

## 测试

```bash
python3 -m unittest scenarios/02_rocket_two_axis_pod/test_prepare_world.py
python3 -m unittest scenarios/02_rocket_two_axis_pod/test_target_manager.py
python3 -m unittest scenarios/02_rocket_two_axis_pod/test_collision_manager.py
bash -n scenarios/02_rocket_two_axis_pod/start.sh
bash -n scenarios/02_rocket_two_axis_pod/stop.sh

cmake -S scenarios/02_rocket_two_axis_pod/gui \
  -B scenarios/02_rocket_two_axis_pod/gui/build \
  -DSCENARIO02_BUILD_GUI=ON -DBUILD_TESTING=ON
cmake --build scenarios/02_rocket_two_axis_pod/gui/build
ctest --test-dir scenarios/02_rocket_two_axis_pod/gui/build --output-on-failure
```

启动场景后，可使用原生 MAVSDK 通过 `14280+i` 依次控制飞机完成 20 米见方的闭合航线，并验证实际水平位移、爬升、航线完成、返航和落地：

```bash
# 从 192.168.1.100 测试 192.168.1.56 上的一架 SITL
bash scenarios/02_rocket_two_axis_pod/test_mavsdk_route.sh -n 1 \
  --sitl-host 192.168.1.56 \
  --mavsdk-prefix /home/firefly/code/3rdparty/common/mavsdk/v3.17.4/arm64/ubuntu22.04
```

航点以各架飞机测试开始时的实时 GPS 位置为原点，因此适用于本 world 的地理坐标和多机出生偏移。多机按顺序测试，未轮到的飞机保持地面待命。构建脚本只链接已有 `libmavsdk.so` 编译小型测试程序，不会重新编译 MAVSDK 库；也可通过 `--mission-bin` 直接指定已经编译好的程序。

测试应在独立机器上运行。当前 `v4.0.6-udp-dyt` 的 `udpout://HOST:PORT` 会在测试机本地绑定同一个端口；若测试程序和 PX4 在同一台机器，`14280+i` 会与 PX4 自己的端口冲突。`--sitl-host` 因此为必填参数。

场景会在运行时机体副本中将 `EKF2_HGT_REF` 设为 GPS。该 world 的 GPS 海拔与气压高度原点不同，使用气压高度参考会使全局高度无效，PX4 因而拒绝 Mission 模式解锁；资产仓原文件不会被修改。
