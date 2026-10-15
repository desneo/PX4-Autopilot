# PX4 SITL 测试场景

每个子目录包含一个独立场景及其启动、停止和测试工具。

- [`01_common`](01_common/README.md)：在同一个 Gazebo world 中运行多架
  `gz_x500`。

系统同时只运行一套 PX4 SITL。启动脚本检测到已有 PX4 SITL 或 Gazebo
进程时会拒绝启动；请先运行对应场景的 `stop.sh`。
