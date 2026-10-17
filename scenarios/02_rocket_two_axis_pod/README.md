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

脚本会增量构建 PX4 SITL、两轴吊仓插件和本场景的碰撞监控插件；使用 `--gui` 时也会增量构建 world 的地图中心 GUI 插件，并启动靶机和雷达管理服务。

## 雷达

world 中心固定放置一套无碰撞体的雷达外观，天线相位中心距当地地面 `3 m`。雷达实体始终存在，但默认不扫描；使用 `--gui` 启动后，在地图右侧“雷达控制”面板中手动启动或停止。启动脚本不提供自动开启雷达的命令行参数。

“飞机定位与跟随”“雷达控制”和“靶机控制”位于同一个固定在 3D 地图右侧的纵向容器中，不能拖动、最小化或关闭，各自只保留折叠按钮。容器顶部预留独立安全区放置“回到地图中心”按钮，控制卡片从按钮下方开始排列，不再遮挡按钮；折叠后标题仍停留在右侧，内容超过窗口高度时在容器内滚动。

“飞机定位与跟随”列表包含本场景全部普通 PX4 飞机，并在靶机运行期间动态加入当前靶机。选择一架飞机后，Gazebo 视野立即定位到该模型并持续跟随移动，不改变飞机航迹、PX4 模式或控制状态。跟随过程中向前滚动鼠标抵近目标、向后滚动远离目标，最小跟随距离为 `0.25 m`；按住中键拖动可绕机旋转观察角度。距离和方位直接作为相对飞机的跟随偏移实时更新，调整期间不会解除跟随，因此飞行中的飞机始终保持为观察中心。为避免地面视角进入地下，环绕俯仰角最低限制为飞机中心上方约 `2°`。点击“停止跟随”恢复自由视角；点击“回到地图中心”同样会先退出跟随，再回到 world 中心点上方200米的俯视视角。靶机终止或被清理时，如果视野正在跟随该靶机，会自动退出跟随。

雷达按航迹级传感器建模，不使用目标的精确位置直接冒充输出。它具有以下特性：

- 360° 方位覆盖，俯仰覆盖 `-5°～60°`，最小作用距离 `50 m`；
- 航迹频率默认 `1 Hz`，可在界面修改为 `1.0～2.0 Hz`，支持 `1.5 Hz` 等小数；非法值会阻止启动，运行期间频率锁定；
- 对 `rocket_two_axis_pod_*` 等大疆级小型飞机的标称稳定探测距离为 `3 km`，对“小摩托”和“小蝙蝠”的标称稳定探测距离为 `5 km`；
- 标称距离内、满足视线和俯仰条件时稳定检出；标称距离外10%的过渡区按距离衰减，之后不检出；
- 使用资产地形瓦片高度近似低空地形遮挡，并叠加随距离增大的距离/角度误差、径向速度误差以及 `120～200 ms` 的处理延迟抖动；
- 连续两帧命中后确认航迹，短时丢失最多保持三帧；随机种子固定，便于重复验证；默认不生成虚警；
- 雷达探测所有满足条件的飞机，不判定敌我。输出只有稳定 `track_id` 和统一目标类别，不包含 Gazebo 模型名、飞机编号或敌我属性，敌我关联由地面站完成。

雷达航迹频率输入框下面会显示 `1.0～2.0 Hz` 范围。点击“停止雷达”后立即停止扫描和 UDP 输出，但 world 中的雷达外观保留。

雷达航迹与靶机真值共用“地面站地址（雷达/靶机真值）”输入框，默认 `192.168.1.41:7790`。两种 JSON 通过 `type` 和 `source` 区分。雷达每次扫描发送一个包含零条或多条航迹的数据报，例如：

```json
{"type":"radar_tracks","source":"radar_center","schema_version":1,"timestamp_ms":1792108800000,"scan_sequence":12,"scan_rate_hz":1.0,"radar":{"latitude":32.21785738,"longitude":119.10587784,"altitude_msl_m":7.02},"tracks":[{"track_id":3,"state":"confirmed","classification":"small_target","confidence":0.99,"range_m":1996.8,"azimuth_deg":0.12,"elevation_deg":2.7,"radial_speed_m_s":-34.6,"latitude":32.21789,"longitude":119.12708,"altitude_msl_m":101.5}]}
```

经纬高、距离和速度均为带误差的雷达量测，不是真值。`altitude_msl_m` 使用 MSL 米，`radial_speed_m_s` 以远离雷达为正。雷达关闭时，靶机真值和 QGC 虚拟 MAVLink 代理仍可独立运行。

## 靶机控制

使用 `--gui` 启动时，Gazebo 界面右侧显示“靶机控制”面板。面板的“启用靶机”下拉框包含“不启用”“小摩托”和“小蝙蝠”。选择机型后显示巡航速度、海拔高度、来源距离、来源方向以及启动/终止按钮。地面站地址统一使用上方雷达控制卡片中的共享输入框。场景默认无 GUI，因此不启动靶机和雷达管理服务；无 GUI 的既有用法保持不变。

靶机使用资产仓模型的视觉部分，由 Gazebo 场景航迹插件按仿真时间直接控制位姿；它不启动独立 PX4、不运行传感器或气动插件，也不占用 MAVLink 端口。同一时间只允许一架靶机。重复点击“启动靶机”会提示先终止当前靶机；点击“终止靶机”会从 world 删除模型，并立即停止位置数据发送。普通火箭机和吊仓仍使用真实 PX4 SITL，不受影响。

机型切换时自动设置经过模型飞行验证的默认巡航速度，并在界面显示有效范围：

- 小摩托：默认 `35 m/s`，允许 `25～65 m/s`；
- 小蝙蝠：默认 `18 m/s`，允许 `10～30 m/s`。

速度、距离、地址格式或其他参数无效时，界面和后端都会提示原因并阻止启动。界面显示来源距离范围提示，允许 `100～10000 m`，默认 `2000 m`。海拔高度使用 MSL，默认值是 world 中心海拔加100米，即当前 world 约 `104.02 m`；输入高度还必须高于整段航线最高地形至少30米。

“从东向西”在中心正东方生成靶机，“从东北45度”在中心东北方向生成靶机。模型直接生成在配置的海拔高度，随后按配置巡航速度直线飞向 world 中心；穿过中心后继续飞行500米，并在终点保持静止，直到终止靶机。该模式用于场景目标和吊仓搜索测试，不模拟靶机自身的起飞、姿态动力学或飞控故障。

靶机有效定位建立后，以 `10 Hz` 向共享地面站地址发送 UTF-8 UDP 真值 JSON；`altitude_msl_m` 为 MSL 米，`speed_m_s` 为实时水平地速 m/s。该数据用于调试雷达量测误差，明确标记为仿真真值：

```json
{"type":"target_truth","source":"simulation_truth","timestamp_ms":1792108800000,"model":"xiaomotuo","latitude":32.217857,"longitude":119.105878,"altitude_msl_m":104.02,"speed_m_s":34.8}
```

启用 GUI 靶机和雷达功能需要系统 Python 能导入 `gz.transport13` 和 `gz.msgs10`，不需要 `pymavlink`。靶机位姿按每个仿真步更新，管理服务以20 Hz接收位置状态，UDP 真值以10 Hz发送。

靶机同时通过轻量 MAVLink 代理出现在 QGroundControl 中。默认发送到 `127.0.0.1:14550`，使用独立 `SYSID=200`；可通过 `--target-qgc-address HOST:PORT` 和 `--target-sysid ID` 修改。代理只发送 `HEARTBEAT`、`GLOBAL_POSITION_INT`、`GPS_RAW_INT` 和 `VFR_HUD`，不监听或执行 QGC 控制指令，也不占用普通飞机的 `14280+i` 端口。启动脚本会拒绝与当前普通飞机 `SYSID 1～n` 重叠的靶机 SYSID。终止靶机后代理立即停止发送，QGC 会在心跳超时后将该靶机显示为断开或移除。

## 碰撞致损

场景默认启用确定性的 `damage-on-contact`，用于发现集群路径规划中的碰撞问题。致损服务直接调用对应 SITL 实例的 PX4 内置故障注入命令，将全部电机设为关闭；它不占用 MAVSDK UDP 端口，也不会改变外部控制程序的连接。

- PX4 与 PX4：使用 Gazebo 原生碰撞体和接触检测；发生实际接触后，两架飞机都关闭全部电机。
- PX4 与地面或建筑物：使用 Gazebo 原生碰撞检测；飞机确认离地1米后才启用该规则，避免出生点的地面接触触发故障。明确处于 `AUTO_LAND` 或 `AUTO_PRECLAND` 的正常降落接触被豁免，其他飞行中表面接触会关闭全部电机。
- PX4 与靶机：靶机继续采用轻量场景航迹，不参与刚体碰撞。监控器使用与模型尺寸匹配的简化包围盒判定相交，并按速度矢量差计算相对速度：`|V_px4 - V_target|`。相对速度 **大于或等于10 m/s** 才关闭 PX4 的全部电机；低于 `10 m/s` 只记录事件，不触发坠毁。

PX4 与靶机达到致损条件后，靶机冻结在碰撞位置，状态变为“已碰撞”，立即停止航迹、10 Hz JSON 真值和虚拟 MAVLink 遥测，但不模拟自由落体。模型保留在 world 中，点击“终止靶机”后才清理。雷达若仍在运行，会按自身的航迹保持/丢失规则处理该目标。

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

最新资产仓的吊仓网关仅支持 NVIDIA 硬件编码，场景默认使用 `nvenc`；`--encoder` 保留用于显式配置，但当前只接受 `nvenc`。

## 测试

```bash
python3 -m unittest scenarios/02_rocket_two_axis_pod/test_prepare_world.py
python3 -m unittest scenarios/02_rocket_two_axis_pod/test_target_manager.py
python3 -m unittest scenarios/02_rocket_two_axis_pod/test_radar_manager.py
python3 -m unittest scenarios/02_rocket_two_axis_pod/test_collision_manager.py
bash -n scenarios/02_rocket_two_axis_pod/start.sh
bash -n scenarios/02_rocket_two_axis_pod/stop.sh

cmake -S scenarios/02_rocket_two_axis_pod/gui \
  -B scenarios/02_rocket_two_axis_pod/gui/build \
  -DSCENARIO02_BUILD_GUI=ON -DBUILD_TESTING=ON
cmake --build scenarios/02_rocket_two_axis_pod/gui/build
ctest --test-dir scenarios/02_rocket_two_axis_pod/gui/build --output-on-failure

# 使用独立 Gazebo 分区进行雷达端到端冒烟测试，不启动 PX4
python3 scenarios/02_rocket_two_axis_pod/test_radar_integration.py \
  --metadata ../px4-gazebo-assets/worlds/geo_32_217857_119_105878/metadata.json

# 场景已经使用 --gui 启动后，联合验证 10 Hz 真值与 1 Hz 雷达航迹
GZ_PARTITION=px4_scenario_02_rocket_two_axis_pod GZ_IP=127.0.0.1 \
  python3 scenarios/02_rocket_two_axis_pod/test_live_radar_target.py --duration 5
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
