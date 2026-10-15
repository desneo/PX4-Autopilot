# 场景 02：火箭穿越机与两轴吊仓

在资产仓的 `geo_32_217857_119_105878` world 中启动火箭穿越机与顶部两轴吊仓组合模型。所有飞机使用同一个 world，默认启动一架、不开启 Gazebo GUI，启动后保持待命，不自动解锁或飞行。

## 启动和停止

```bash
# 默认一架
bash scenarios/02_rocket_two_axis_pod/start.sh

# 四架，打开 Gazebo GUI
bash scenarios/02_rocket_two_axis_pod/start.sh -n 4 --gui

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

脚本会增量构建 PX4 SITL 和资产仓的两轴吊仓插件；使用 `--gui` 时也会增量构建 world 的地图中心 GUI 插件。

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
bash -n scenarios/02_rocket_two_axis_pod/start.sh
bash -n scenarios/02_rocket_two_axis_pod/stop.sh
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
