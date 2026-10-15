# 场景 01：通用多机环境

在一个 Gazebo `default` world 中运行多架 `gz_x500`。默认启动 4 架，
飞机间距 5 米，默认不开启 GUI。Gazebo Transport 固定使用
`GZ_IP=127.0.0.1`，其发现和仿真消息不会进入局域网；MAVLink UDP 不受影响。

## 启动

```sh
# 默认 4 架
./scenarios/01_common/run_multi_plane.sh

# 指定架次，范围 1～10
./scenarios/01_common/run_multi_plane.sh -n 6

# 开启 Gazebo GUI
./scenarios/01_common/run_multi_plane.sh -n 4 --gui
```

脚本会在启动每架飞机前输出完整命令，日志保存在
`/tmp/px4-scenarios/01_common`。

## MAVSDK 测试

第 `i` 架飞机监听 SITL 主机的 UDP `14280+i`。MAVSDK 使用：

```text
udpout://SITL主机IP:14280+i
```

本机测试：

```sh
./scenarios/01_common/test_mavsdk_mission.sh -n 4
```

远端 Firefly（`192.168.1.100`）直接使用已有库，不重新编译 MAVSDK：

```text
/home/firefly/code/3rdparty/common/mavsdk/v3.17.4/arm64/ubuntu22.04
```

只需用 `build_mavsdk_mission.sh` 编译一次测试程序。SITL 位于
`192.168.1.56` 时，第 1 架飞机的测试命令为：

```sh
/home/firefly/mavsdk_mission udpout://192.168.1.56:14280
```

后续飞机依次使用 `14281`、`14282` 等端口。测试程序会上传航线、解锁、
执行航线、返航、降落并等待自动上锁，不使用 `mavsdk-grpc`。

## 停止

```sh
./scenarios/01_common/stop.sh
```

该脚本会停止系统中当前所有 PX4 SITL 和 Gazebo 进程。
