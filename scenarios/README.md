# PX4 SITL 测试场景

每个子目录包含一个独立场景及其启动、停止和测试工具。

- [`01_common`](01_common/README.md)：在同一个 Gazebo world 中运行多架
  `gz_x500`。
- [`02_rocket_two_axis_pod`](02_rocket_two_axis_pod/README.md)：在项目地理
  world 中运行火箭穿越机与两轴吊仓组合模型，支持多机独立吊仓接口。

系统同时只运行一套 PX4 SITL。启动脚本检测到已有 PX4 SITL 或 Gazebo
进程时会拒绝启动；请先运行对应场景的 `stop.sh`。
