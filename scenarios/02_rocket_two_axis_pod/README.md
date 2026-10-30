# 场景 02：火箭穿越机与两轴吊仓

在资产仓的 `geo_32_217857_119_105878` world 中启动火箭穿越机与顶部两轴吊仓组合模型。所有飞机使用同一个 world，默认启动一架、不开启 Gazebo GUI，启动后保持待命，不自动解锁或飞行。

当前组合模型总长 `590 mm`、质量 `5.00 kg`。每次启动从资产仓原生模型生成运行副本，继承质量、惯量、动力配置，以及装配生成器派生的重心参考、控制分配和 IMU/GPS 位置参数。

## 启动和停止

```bash
# 默认一架
bash scenarios/02_rocket_two_axis_pod/start.sh

# 显式切换为 CSRT 图像跟踪
bash scenarios/02_rocket_two_axis_pod/start.sh --tracking-backend csrt

# 四架，打开 Gazebo GUI
bash scenarios/02_rocket_two_axis_pod/start.sh -n 4 --gui

# 需要恢复飞机之间的刚体碰撞和互相致损时显式启用
bash scenarios/02_rocket_two_axis_pod/start.sh -n 2 --enable-collision

# QGC 在远程机器时，显式指定 QGC 的 UDP 地址
bash scenarios/02_rocket_two_axis_pod/start.sh --gui \
  --target-qgc-address 192.168.1.100:14550

# 打印给局域网客户端使用的指定地址
bash scenarios/02_rocket_two_axis_pod/start.sh -n 2 \
  --advertise-host 192.168.1.56

bash scenarios/02_rocket_two_axis_pod/stop.sh
```

`-n` 支持 1～10，默认为 1。飞机在 world 中按 5 米间隔排列，首机平面位置为 ENU `(5.25,5) m`，后续飞机从该偏移按网格展开；格点避开当前地形三角面的对角共边，使尾底位于单一平面内。

吊仓单目标跟踪默认使用 `sim_truth`，用于优先验证后续导引算法；只有显式指定 `--tracking-backend csrt` 才使用 CSRT 图像跟踪。`sim_truth` 的自动锁定从视野中选择最靠近画面中心的飞机，手动框选只在选框内的飞机中选择；锁定后绑定该 Gazebo 实体，不因多目标交叉而自动换锁。可锁定候选包含本场景全部 `rocket_two_axis_pod_<id>` 普通飞机，以及运行时动态出现的 `xiaomotuo` 和 `xiaobianfu`，不将地形或标志物当作候选。

每机出生绝对 Z 取对应静态地形碰撞三角面的插值高度，加该机生成目录 `parameters.json` 中 `simulation.spawn_pose` 的 Z（当前0.215 m）；`prepare_world.py --altitude` 可显式覆盖这一竖直相对高度。初始yaw为0，roll/pitch将机体+Z对齐该三角面的向上单位法线，吊仓相对机体的安装保持继承，实体随后按正常刚体物理演化。当前双机默认坐标约为 `(5.25,5,-0.036526112) m`、`(10.25,5,-0.268718752) m`，初始RPY约为 `(−0.0884°,2.6588°,0°)`；对应地面高度为−0.251526112 m、−0.483718752 m，圆柱尾底到当地地面的最小法向间隙约4.77 mm。

地形采用当前world的米制Z_UP、平移tile、单平铺实例DAE三角面，缺失、不支持或格点未被地形覆盖时启动报错；世界中心雷达保持在 `(0,0)`，出生点和镜头不会位于雷达杆内部。`GZ_IP=127.0.0.1` 只限制 Gazebo Transport 使用回环接口，不影响 MAVLink、吊仓 UDP 和 RTSP 的局域网访问。

### WSL 图形环境

WSL 下启动脚本默认设置 `GALLIUM_DRIVER=d3d12` 和
`MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`，使 Gazebo GUI 与无窗口相机使用
NVIDIA GPU；已显式设置的环境变量优先。可用 `glxinfo -B` 检查渲染器，
应显示 `D3D12 (NVIDIA ...)` 和 `Accelerated: yes`；`llvmpipe` 表示 CPU 软件渲染。

中文界面和 SVG 图标需要安装字体与 Qt 组件：

```bash
sudo apt-get install fonts-noto-cjk libqt5svg5 mesa-utils
sudo usermod -aG render,video "$USER"
```

加入设备访问组后，重新登录终端再启动场景。字体安装后需重新打开 GUI。

未指定 `--advertise-host` 时，脚本读取系统主默认路由接口上的全局 IPv4；它不会使用 mihomo 等代理策略路由的 `198.18.0.0/15` 地址。若机器有多条默认路由，可显式传入 `--advertise-host 192.168.1.56`。

## 资产仓

资产仓默认位于 PX4 仓库同级的 `../px4-gazebo-assets`。首次启动时自动下载：

```text
https://github.com/desneo/px4-gazebo-assets.git
```

以后每次启动都会先获取远程 `origin/main`，并确认本地 `main` 已包含远程最新提交。本地提交与远程相同或领先时均可启动，也允许工作区存在未提交修改；脚本直接使用当前工作区生成运行模型，不会执行 `reset`、`checkout` 或覆盖本地修改。远程检查失败、分支不是 `main`，或本地提交落后/分叉于 `origin/main` 时，脚本会报告原因并停止。

可用环境变量覆盖位置和下载地址：

```bash
PX4_GAZEBO_ASSETS_ROOT=/path/to/px4-gazebo-assets \
PX4_GAZEBO_ASSETS_URL=https://example/model-repo.git \
  bash scenarios/02_rocket_two_axis_pod/start.sh
```

脚本会增量构建 PX4 SITL、两轴吊仓插件和本场景的碰撞监控插件；使用 `--gui` 时也会增量构建 world 的地图中心 GUI 插件，并启动靶机和雷达管理服务。

## 吊仓姿态与定位

吊仓沿用新CAD原始零位，镜头朝机体 +Z；外轴 yaw 绕水平 +X、范围±60°，内轴 pitch 绕随动 −Y、范围−80°～+30°，运动链为 `Rx(yaw) Ry(-pitch)`，协议 v7、`motion_head_joint_right_positive`。外轴穿过球心，球心不随外轴公转。机械 yaw 不是绕竖直轴的水平 pan；机体倾转后，两轴和可达域随机体变化。

水平安装时，yaw=0、pitch=−80°的前向光轴仍高于水平约10°。默认2 km来源距离、world中心海拔加100 m的靶机，其初始仰角约3°；近距、更高目标或载机倾转时，应依据实际机械可达域和罩壳遮挡检查图像覆盖。雷达检出不表示该方向已能由吊仓镜头锁定；当前入口不承诺任意方向跟踪。

定位箭头的Z随该机地形插值后的出生绝对Z设置。每架飞机西侧设置橙黄色、无碰撞的 `POD_LOCATION_MARKER_i` 定位箭头，箭头约在该机出生原点Z上方0.35 m处，指向顶部吊仓附近。可在实体树搜索 `rocket_two_axis_pod_i` 或 `POD_LOCATION_MARKER_i`，再使用“飞机定位与跟随”抵近。浏览相机近裁剪距离为0.005 m；近景缩放时将鼠标指向实体表面，避免以远处背景作为缩放参考，右键缓慢拖动微调。浏览相机和吊仓视频相机分别配置。厘米级部件可先将浏览相机灵敏度设为0.1，再按场景尺度调整；该服务同时影响缩放、平移和旋转，需在GUI服务就绪后、同一Gazebo分区执行，每次重启GUI后重新设置：

```bash
GZ_PARTITION=px4_scenario_02_rocket_two_axis_pod GZ_IP=127.0.0.1 \
  gz service -s /gui/camera/view_control/sensitivity \
  --reqtype gz.msgs.Double --reptype gz.msgs.Boolean \
  --timeout 3000 --req 'data: 0.1'
```

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

雷达航迹发送到“地面站地址（雷达航迹）”输入框指定的 UDP 地址。本地修改会持久保存，下次启动 GUI 时自动恢复；雷达停止时可以修改，运行或正在启停时锁定。雷达每次扫描发送一个包含零条或多条航迹的数据报，例如：

```json
{"type":"radar_tracks","source":"radar_center","schema_version":1,"timestamp_ms":1792108800000,"scan_sequence":12,"scan_rate_hz":1.0,"radar":{"latitude":32.21785738,"longitude":119.10587784,"altitude_msl_m":7.02},"tracks":[{"track_id":3,"state":"confirmed","classification":"small_target","confidence":0.99,"range_m":1996.8,"azimuth_deg":0.12,"elevation_deg":2.7,"radial_speed_m_s":-34.6,"latitude":32.21789,"longitude":119.12708,"altitude_msl_m":101.5}]}
```

经纬高、距离和速度均为带误差的雷达量测，不是真值。`altitude_msl_m` 使用 MSL 米，`radial_speed_m_s` 以远离雷达为正。雷达关闭时，靶机航迹和 QGC 虚拟 MAVLink 代理仍可独立运行。

## 靶机控制

使用 `--gui` 启动时，Gazebo 界面右侧显示“靶机控制”面板。面板的“启用靶机”下拉框包含“不启用”“小摩托”和“小蝙蝠”。选择机型后显示巡航速度、海拔高度、来源距离、来源方向以及启动/终止按钮。场景默认无 GUI，因此不启动靶机和雷达管理服务；无 GUI 的既有用法保持不变。

靶机使用资产仓模型的视觉部分，由 Gazebo 场景航迹插件按仿真时间直接控制位姿；它不启动独立 PX4、不运行传感器或气动插件，也不占用普通飞机的 MAVLink 端口。同一时间只允许一架靶机。重复点击“启动靶机”会提示先终止当前靶机；点击“终止靶机”会从 world 删除模型，并立即停止航迹和 QGC 状态发送。普通火箭机和吊仓仍使用真实 PX4 SITL，不受影响。

机型切换时自动设置经过模型飞行验证的默认巡航速度，并在界面显示有效范围：

- 小摩托：默认 `35 m/s`，允许 `25～65 m/s`；
- 小蝙蝠：默认 `18 m/s`，允许 `10～30 m/s`。

速度、距离或其他参数无效时，界面和后端都会提示原因并阻止启动。界面显示来源距离范围提示，允许 `100～10000 m`，默认 `2000 m`。海拔高度使用 MSL，默认值是 world 中心海拔加100米，即当前 world 约 `104.02 m`；输入高度还必须高于整段航线最高地形至少30米。

“从东向西”在中心正东方生成靶机，“从东北45度”在中心东北方向生成靶机。模型直接生成在配置的海拔高度，随后按配置巡航速度直线飞向 world 中心；穿过中心后继续飞行500米，并在终点保持静止，直到终止靶机。该模式用于场景目标和吊仓搜索测试，不模拟靶机自身的起飞、姿态动力学或飞控故障。

启用 GUI 靶机和雷达功能需要系统 Python 能导入 `gz.transport13` 和 `gz.msgs10`，不需要 `pymavlink`。靶机位姿按每个仿真步更新，管理服务以20 Hz接收位置状态。靶机不再发送独立 UDP 真值数据。

靶机同时通过轻量 MAVLink 代理出现在 QGroundControl 中。默认发送到 `127.0.0.1:14550`，使用独立 `SYSID=200`；可通过 `--target-qgc-address HOST:PORT` 和 `--target-sysid ID` 修改。代理只发送 `HEARTBEAT`、`GLOBAL_POSITION_INT`、`GPS_RAW_INT` 和 `VFR_HUD`，不监听或执行 QGC 控制指令，也不占用普通飞机的 `14280+i` 端口。启动脚本会拒绝与当前普通飞机 `SYSID 1～n` 重叠的靶机 SYSID。终止靶机后代理立即停止发送，QGC 会在心跳超时后将该靶机显示为断开或移除。

## 碰撞致损

场景默认关闭飞机之间的刚体接触和互相致损，避免两架飞机相遇后因接触约束物理耦合。每架飞机的所有碰撞体使用独立的 SDFormat `category_bitmask` / `collide_bitmask`；地形和建筑物保留默认位掩码，所以飞机与环境的原生碰撞不受影响。使用 `start.sh --enable-collision` 可启用飞机之间的原生刚体碰撞和互相致损。

飞机与地形、建筑物、靶机仍采用确定性的 `damage-on-contact`。致损服务直接调用对应 SITL 实例的 PX4 内置故障注入命令，将全部电机设为关闭；它不占用 MAVSDK UDP 端口，也不会改变外部控制程序的连接。

- PX4 与 PX4：默认不产生刚体接触，也不关闭电机；使用 `--enable-collision` 时恢复 Gazebo 原生接触检测，发生接触后两架飞机都关闭全部电机。
- PX4 与地面或建筑物：使用 Gazebo 原生碰撞检测；飞机确认离地1米后才启用该规则，避免出生点的地面接触触发故障。明确处于 `AUTO_LAND`、`AUTO_PRECLAND`，或处于 `AUTO_RTL` 且当前有效位置设定点为 `LAND` 时，正常降落触地被豁免。RTL 的爬升、返航和盘旋阶段不豁免，其他飞行中表面接触仍关闭全部电机。降落豁免不适用于已显式启用的飞机间接触或飞机与靶机的接触。
- PX4 与靶机：靶机继续采用轻量场景航迹，不参与刚体碰撞。**碰撞即损毁，不设置相对速度门槛，包括零相对速度接触。** 监控器从运行中的 PX4 模型读取各物理碰撞体的尺寸、中心和实时姿态，逐个与靶机的简化定向包围盒检查相交；支持俯仰、横滚、偏航和吊仓关节运动，不使用固定整机尺寸。圆柱、球等形状使用局部外接盒近似。当前9个CAD网格碰撞体的实际DAE顶点、实例节点变换、米制单位与mesh缩放在生成世界时读取，分别保存局部包围盒中心和半尺寸；运行时将局部中心经该collision的实时世界姿态变换，随载机和活动关节运动。物理mesh和原生接触传感器完整保留，不以这些检测盒替换。检测盒包含网格空腔及盒角，因此靶机交会判定仍是保守近似。相对速度 `|V_px4 - V_target|` 仅用于记录事件。

mesh检测配置以实际 `model/link/collision` 唯一键绑定。当前支持资产仓导出的具备有效法线的Z_UP、三角面、平铺实例节点DAE，以及节点平移、旋转、缩放和仿射矩阵；支持非单位mesh缩放与非零局部中心。缺失、空、非有限或退化网格、未知格式、其他up_axis、嵌套节点、submesh选择或重置中心等情况在生成世界前明确拒绝。插件配置缺失或重复同样报错，不静默忽略碰撞体。

PX4 与靶机碰撞后，PX4 关闭全部电机，靶机冻结在碰撞位置，状态变为“已碰撞”，立即停止航迹和虚拟 MAVLink 遥测，但不模拟自由落体。模型保留在 world 中，点击“终止靶机”后才清理。雷达若仍在运行，会按自身的航迹保持/丢失规则处理该目标。

当前地理 world 的地面和高程结构统一包含在 `tile_*::terrain::ground` 网格碰撞体中，因此事件中不区分“地面”和“建筑物”，两者执行相同致损策略。重复接触不会重复注入故障。

所有碰撞事件写入：

```text
/tmp/px4-scenarios/02_rocket_two_axis_pod/logs/collision-events.jsonl
```

每行是一个 JSON 对象，包含碰撞类型、仿真时间、飞机编号、相对速度、ENU/WGS84 位置、是否触发致损以及各 PX4 实例的故障注入结果。致损事件记录 `reason=collision_damage_policy`，直接降落豁免记录 `reason=normal_landing_mode`，RTL 最后降落阶段记录 `reason=rtl_landing_setpoint` 和当前设定点的有效标记、类型；关闭机间碰撞时收到的防御性 `px4_px4` 事件记录 `reason=aircraft_collision_disabled` 且不注入故障。

## 多机接口

第 `i` 架飞机使用：

- PX4/MAVSDK UDP：`14280 + i - 1`；
- 吊仓指令与遥测 UDP：`8770 + i - 1`；
- Aisheng JSON UDP：`9000 + i - 1`；
- 共享 RTSP 端口：`8555`；
- 视频路径：`/two_axis_pod_i`；
- Gazebo 吊仓话题：`/uav_i/pod/*`。

例如两架飞机：

```text
飞机1：udp://192.168.1.56:8770
       Aisheng JSON udp://192.168.1.56:9000
       rtsp://192.168.1.56:8555/two_axis_pod_1
飞机2：udp://192.168.1.56:8771
       Aisheng JSON udp://192.168.1.56:9001
       rtsp://192.168.1.56:8555/two_axis_pod_2
```

02 场景会在每个 PX4 SITL 实例启动完成后，自动为该实例启动独立的 `dyt_udp_bridge`。模块只编译进 SITL；它监听本机全部接口上的对应 Aisheng JSON UDP 端口，并将有效报文发布到该实例的 `dyt_aisheng_telemetry` uORB 主题。每架飞机启动前都会输出完整 PX4、吊仓网关和地面站命令，并将对应关系（包括 `dyt_udp`）保存到运行目录的 `interfaces.tsv`。RTSP 内部 RTP 端口由资产仓网关自动分配。

JSON `tracking_gate_w`、`tracking_gate_h` 分别映射为 uORB 的波门宽度和高度，保留 `uint16` 类型。输入接受 0～65535 的有限数值，并按四舍五入（0.5 进 1）转换为整数像素；各字段缺省为 0，负数、越界或非数值会拒绝整包。

吊仓网关、地面站与视频处理功能由资产仓提供，场景负责逐机启动网关并配置接口。打印的地面站命令调用资产仓 `run.sh`，由启动器选择并检查地面站 Python 环境；使用 `--decoder nvdec` 硬解，以及 `--no-manage-bridge` 连接场景已启动的网关，关闭地面站窗口不会停止场景网关。

最新资产仓的吊仓网关仅支持 NVIDIA 硬件编码，场景默认使用 `nvenc`；`--encoder` 保留用于显式配置，但当前只接受 `nvenc`。

## 目标惯性 LOS 遥测采集

`check_target_los.py` 验证正式 02 场景的两轴协议 v8、目标 NED 惯性视线角速度、机械 `gyro_*` 映射与艾晟19字段实际UDP。先启动场景、生成运动靶机，并完成镜头对准与锁定；工具不启动或停止 Gazebo/PX4，也不自动生成目标。使用资产仓已配置的 Python 环境，`--assets-root` 默认指向 PX4 仓同级的 `px4-gazebo-assets`：

```bash
../px4-gazebo-assets/.venv-linux/bin/python scenarios/02_rocket_two_axis_pod/check_target_los.py \
  --duration 120 --finish-file /tmp/scenario02-los.finish \
  --output /tmp/scenario02-los-evidence
# 另一个终端中创建finish文件，正常提前结束；开始前该文件应不存在
# touch /tmp/scenario02-los.finish
```

默认连接首机 UDP `127.0.0.1:8770`、`/uav_1/pod/telemetry` 和本场景 Gazebo 分区。生产 `UdpClient` 与 `TelemetryForwarder` 将新遥测转换并实际发送至临时回环 UDP 接收端；脚本核对19个字段、`gyro_pitch_rate/gyro_yaw_rate` 与 `pitch_los_rate/yaw_los_rate` 的独立来源和 ±20°/s 限幅，新增`los_omega_n/e/d`取独立`omega_ned_dps`，单位°/s且不限幅；原两角度导数及gyro仍限±20°/s。向量无效时发0/0/0，竖直时可保持向量有效。`gate_x/gate_y`独立取最终原始跟踪框中心，左上原点、右/下正，half-up四舍五入为JSON整数；失效发送0/0。接收端映射至已有int16字段，接受0..32767的整数字面量，旧9/13/16字段缺整对时默认0/0；残缺或非法坐标拒绝。`timestamp`为地面站发送线程在实际发送前生成的Unix整数毫秒时间戳，不采用仿真时间或遥测采样时间；采集工具独立校验其位于本机送入转发队列至收包的时间区间，并去掉该动态字段后比对其余18个测量字段。PX4将该扩展字段跳过，uORB自身`timestamp`仍为本机hrt微秒。并分别统计插件、网关、艾晟的真实时间Hz。显式加 `--px4-udp-port 9000` 可将实际收到的艾晟原始字节复制给首机 `dyt_udp_bridge`，用于配合 PX4 `listener dyt_aisheng_telemetry` 检查；默认不复制。

独立真值取自带仿真时间戳的 Gazebo `pose/info` 与 `dynamic_pose/info`，仅合并完全同刻的位置，不使用无时间戳的 trajectory 输出。从运行 SDF 各 link 的零位惯性点独立求名义重心，以模型世界位姿旋转该固定偏移，再由目标位置重建 NED 方位角和仰角的中心差分；不将插件给出的相对位置/速度当作独立角速度真值。当前 world 的 `heading_deg=0`；其它 heading 明确拒绝。三维向量另由同刻位置及独立位置差分计算`r×v/|r|²`，按自身`omega_valid`评分；不要求方位角有效。默认差分半窗 `0.04 s`、相对位置最大误差 `0.1 m`、角速度最大误差 `0.8°/s`，可通过 `--difference-window`、`--position-tolerance`、`--rate-tolerance` 显式设置。

采集窗口应包含至少 20 份可独立对照的有效运动目标 LOS，以及停止后 LOS 无效、外部值 `0.0/0.0` 的样本。界面仍沿用已有 track 状态。工具仅在显式提供 `--action-file /tmp/scenario02-los.actions.jsonl` 时执行逐行 JSON 吊舱验收动作，例如 `{"cmd":"aim","yaw":0,"pitch":-60,"timeout":20}` 通过现有手动速度接口反馈对准机械角；`{"cmd":"manual","yaw":0,"pitch":-10,"duration":2}` 连续发送有限时长的速度流后归零；`{"phase":"stop","cmd":"stop"}` 退出跟踪。`set_vision_mode`、`track`、`home` 和 `reset_axis` 沿用生产协议；未指定动作文件时只采集，不主动改变吊舱状态。动作文件应从空文件开始，尚未锁定时不能将采集结果解释为跟踪验收通过。

输出目录包含每秒更新的 `latest.json`、压缩原始记录 `samples.json.gz`、独立逐样本误差 `independent_scores.json` 和最终 `summary.json`；每 20 秒输出进度。工具正常结束或受控异常时先保存原始证据，再生成结果；只有最终 `summary.json` 明确为 `passed` 才表示本次检查通过，历史或中断采集不代替当前结果。

## 测试

当前v8单机组合飞机的目标惯性LOS、机械gyro、UDP→uORB、原生MAVSDK航线、落地、机械/光学接口及RTSP验证见[验证报告](docs/validation.md)；摘要在`docs/target_los_validation.json`，原始记录与源码快照保存在同级资产仓的装配模型测试目录。

```bash
python3 -m unittest scenarios/02_rocket_two_axis_pod/test_prepare_world.py
python3 -m unittest scenarios/02_rocket_two_axis_pod/test_target_manager.py
python3 -m unittest scenarios/02_rocket_two_axis_pod/test_radar_manager.py
python3 -m unittest scenarios/02_rocket_two_axis_pod/test_collision_manager.py
python3 -m unittest scenarios/02_rocket_two_axis_pod/test_mesh_collision_bounds.py
python3 -m unittest scenarios/02_rocket_two_axis_pod/test_gimbal_validation_math.py
bash -n scenarios/02_rocket_two_axis_pod/start.sh
bash -n scenarios/02_rocket_two_axis_pod/stop.sh

cmake -S scenarios/02_rocket_two_axis_pod/gui \
  -B scenarios/02_rocket_two_axis_pod/gui/build \
  -DSCENARIO02_BUILD_GUI=ON -DBUILD_TESTING=ON
cmake --build scenarios/02_rocket_two_axis_pod/gui/build
ctest --test-dir scenarios/02_rocket_two_axis_pod/gui/build --output-on-failure

# 独立 Gazebo 分区：零速碰撞、三维姿态、实际部件偏置与接触去重
python3 scenarios/02_rocket_two_axis_pod/test_collision_integration.py

# 独立 Gazebo 分区：当前9个CAD mesh、局部中心、节点变换及活动关节交会
python3 scenarios/02_rocket_two_axis_pod/test_mesh_collision_integration.py

# 使用独立 Gazebo 分区进行雷达端到端冒烟测试，不启动 PX4
python3 scenarios/02_rocket_two_axis_pod/test_radar_integration.py \
  --metadata ../px4-gazebo-assets/worlds/geo_32_217857_119_105878/metadata.json

# 场景已经使用 --gui 启动后，联合验证靶机目标与雷达航迹
GZ_PARTITION=px4_scenario_02_rocket_two_axis_pod GZ_IP=127.0.0.1 \
  python3 scenarios/02_rocket_two_axis_pod/test_live_radar_target.py --duration 5

# 双机场景：复用资产仓地面站客户端，验证双轴转动和逐机控制隔离；结束时回零
../px4-gazebo-assets/.venv-linux/bin/python \
  scenarios/02_rocket_two_axis_pod/test_gimbal_interfaces.py -n 2
```

吊仓接口测试需要资产仓已配置的地面站 Python 环境，执行期间不要同时人工控制吊仓。视频身份与帧表绑定可复用资产仓的 `scenarios/geographic_rocket_two_axis_pod/check_rtsp_binding.py`，分别指定各机 ID、UDP 端口和输出目录；该工具只读取遥测与视频。

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
