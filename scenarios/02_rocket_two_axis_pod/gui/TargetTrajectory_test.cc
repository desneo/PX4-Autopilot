// 无 GUI Gazebo 集成测试：独立读取实际模型位姿，验证连续和有限航迹。
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include <gz/msgs/stringmsg.pb.h>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/transport/Node.hh>

namespace
{
void Require(bool condition, const std::string &message)
{
  if (!condition)
    throw std::runtime_error(message);
}

void Near(double actual, double expected, const std::string &message)
{
  Require(std::abs(actual - expected) < 1e-5,
      message + ": " + std::to_string(actual) + " / " +
      std::to_string(expected));
}

class Observation final : public gz::sim::System,
                          public gz::sim::ISystemPostUpdate
{
public:
  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override
  {
    const auto entity = ecm.EntityByComponents(
        gz::sim::components::Name("target"), gz::sim::components::Model());
    if (entity != gz::sim::kNullEntity)
      this->pose = gz::sim::worldPose(entity, ecm);
    const auto secondEntity = ecm.EntityByComponents(
        gz::sim::components::Name("target_2"), gz::sim::components::Model());
    if (secondEntity != gz::sim::kNullEntity)
      this->secondPose = gz::sim::worldPose(secondEntity, ecm);
    const auto nose = ecm.EntityByComponents(
        gz::sim::components::Name("nose_reference"), gz::sim::components::Visual());
    const auto tail = ecm.EntityByComponents(
        gz::sim::components::Name("tail_reference"), gz::sim::components::Visual());
    this->visualsPresent = nose != gz::sim::kNullEntity && tail != gz::sim::kNullEntity;
    if (this->visualsPresent)
    {
      this->nose = gz::sim::worldPose(nose, ecm).Pos();
      this->tail = gz::sim::worldPose(tail, ecm).Pos();
    }
    this->simTime = std::chrono::duration<double>(info.simTime).count();
  }

  gz::math::Pose3d pose;
  gz::math::Pose3d secondPose;
  gz::math::Vector3d nose;
  gz::math::Vector3d tail;
  bool visualsPresent{false};
  double simTime{0.0};
};

void NoseFollowsDisplacement(const Observation &observation,
                            const gz::math::Vector3d &displacement,
                            const std::string &message)
{
  Require(observation.visualsPresent, "未找到机头/机尾视觉参考点");
  const auto front = observation.nose - observation.tail;
  const double frontLength = front.Length();
  const double travelLength = displacement.Length();
  Require(frontLength > 0.0 && travelLength > 0.0, message + "：参考方向无效");
  Near(front.Dot(displacement) / (frontLength * travelLength), 1.0, message);
  Near(front.Cross(displacement).Length() / (frontLength * travelLength),
      0.0, message);
}

void SameVisualPoints(const Observation &observation,
                      const gz::math::Vector3d &nose,
                      const gz::math::Vector3d &tail,
                      const std::string &message)
{
  Near((observation.nose - nose).Length(), 0.0, message + "：机头移动");
  Near((observation.tail - tail).Length(), 0.0, message + "：机尾移动");
}

struct Status
{
  std::string model;
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double speed{0.0};
  int done{-1};
  double yaw{0.0};
};

class StatusObservation
{
public:
  void OnStatus(const gz::msgs::StringMsg &message)
  {
    std::istringstream stream(message.data());
    std::string tag;
    Status value;
    stream >> tag >> value.model >> value.x >> value.y >> value.z
           >> value.speed >> value.done >> value.yaw;
    std::string extra;
    if (!stream || tag != "POSE" || stream >> extra)
      return;
    std::lock_guard<std::mutex> guard(this->mutex);
    this->latest = value;
  }

  bool WaitFor(const std::function<bool(const Status &)> &predicate)
  {
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline)
    {
      {
        std::lock_guard<std::mutex> guard(this->mutex);
        if (predicate(this->latest))
          return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  }

private:
  std::mutex mutex;
  Status latest;
};

void Run()
{
  gz::sim::ServerConfig config;
  Require(config.SetSdfString(R"(
    <sdf version='1.9'>
      <world name='target_trajectory_test'>
        <physics name='test' type='ignored'>
          <max_step_size>0.1</max_step_size>
          <real_time_factor>100000</real_time_factor>
        </physics>
        <plugin filename='gz-sim-physics-system'
                name='gz::sim::systems::Physics'/>
        <plugin filename='TargetTrajectory'
                name='scenario02::TargetTrajectory'/>
        <model name='target'>
          <static>true</static>
          <link name='body'>
            <visual name='nose_reference'>
              <pose>1 0 0 0 0 0</pose>
              <geometry><sphere><radius>0.1</radius></sphere></geometry>
            </visual>
            <visual name='tail_reference'>
              <pose>-1 0 0 0 0 0</pose>
              <geometry><sphere><radius>0.1</radius></sphere></geometry>
            </visual>
          </link>
        </model>
        <model name='target_2'>
          <static>true</static>
          <link name='body_2'>
            <visual name='target_2_visual'>
              <geometry><sphere><radius>0.1</radius></sphere></geometry>
            </visual>
          </link>
        </model>
      </world>
    </sdf>)"), "测试 world 无法解析");
  gz::sim::Server server(config);
  server.SetUpdatePeriod(std::chrono::microseconds(1));
  auto observation = std::make_shared<Observation>();
  Require(server.AddSystem(observation).value_or(false), "无法添加位姿观测器");
  Require(server.RunOnce(true), "Gazebo 初始化失败");

  gz::transport::Node node;
  StatusObservation status;
  Require(node.Subscribe("/scenario/02/target/trajectory/status",
      &StatusObservation::OnStatus, &status), "无法订阅航迹状态");
  auto publisher = node.Advertise<gz::msgs::StringMsg>(
      "/scenario/02/target/trajectory/command");
  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::seconds(5);
  while (!publisher.HasConnections() &&
      std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  Require(publisher.HasConnections(), "航迹插件未订阅 command");
  auto send = [&](const std::string &data)
  {
    gz::msgs::StringMsg message;
    message.set_data(data);
    Require(publisher.Publish(message), "command 发布失败");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  };
  auto step = [&](std::uint64_t iterations, bool paused = false)
  {
    if (paused)
    {
      // Run 的迭代上限只计算未暂停的步；暂停检验必须逐次 RunOnce。
      for (std::uint64_t iteration = 0; iteration < iterations; ++iteration)
        Require(server.RunOnce(true), "Gazebo 暂停步进失败");
    }
    else
    {
      Require(server.Run(true, iterations, false), "Gazebo 步进失败");
    }
  };

  const double yaw = 0.713;
  const double vx = 35.0 * std::cos(yaw);
  const double vy = 35.0 * std::sin(yaw);
  const std::string continuous =
      "START_CONTINUOUS target 15000 -18000 312 35 0.713";
  send(continuous);
  step(1);
  const auto firstPose = observation->pose;
  const double firstTime = observation->simTime;
  Near(firstPose.Z(), 312.0, "连续航迹高度错误");
  Near(firstPose.Rot().Yaw(), yaw, "连续航迹航向错误");
  Require(firstPose.X() >= 15000.0 && firstPose.X() <= 15000.0 + vx * 0.1,
      "连续航迹未从指定起点开始");
  Require(firstPose.Y() >= -18000.0 && firstPose.Y() <= -18000.0 + vy * 0.1,
      "连续航迹未从指定起点开始");

  // 从地图覆盖外起飞，继续飞行约 14 km，超出有限航迹与地图范围。
  step(4000);
  double elapsed = observation->simTime - firstTime;
  Near(observation->pose.X(), firstPose.X() + vx * elapsed,
      "连续航迹 X 未按仿真时间推进");
  Near(observation->pose.Y(), firstPose.Y() + vy * elapsed,
      "连续航迹 Y 未按仿真时间推进");
  Near(observation->pose.Z(), 312.0, "连续航迹未保持高度");
  NoseFollowsDisplacement(*observation, observation->pose.Pos() - firstPose.Pos(),
      "连续航迹机头与实际位移方向不一致");
  Require(status.WaitFor([](const Status &value)
      { return value.model == "target" && value.done == 0 &&
          value.speed == 35.0 && value.x > 25000.0; }),
      "连续航迹七字段 POSE 状态错误或提前结束");

  // 重试同一 START 不能使已经飞远的靶机跳回起点。
  send(continuous);
  step(20);
  elapsed = observation->simTime - firstTime;
  Near(observation->pose.X(), firstPose.X() + vx * elapsed,
      "重复 START 重置了连续航迹");
  Near(observation->pose.Y(), firstPose.Y() + vy * elapsed,
      "重复 START 重置了连续航迹");

  const auto beforePause = observation->pose;
  const double pauseTime = observation->simTime;
  step(100, true);
  Near(observation->simTime, pauseTime, "暂停时仿真时间仍在推进");
  Near(observation->pose.X(), beforePause.X(), "暂停时靶机仍在移动");
  Near(observation->pose.Y(), beforePause.Y(), "暂停时靶机仍在移动");
  step(20);
  elapsed = observation->simTime - firstTime;
  Near(observation->pose.X(), firstPose.X() + vx * elapsed,
      "恢复时连续航迹跳变");
  Near(observation->pose.Y(), firstPose.Y() + vy * elapsed,
      "恢复时连续航迹跳变");

  send("START_CONTINUOUS target 0 0 0 -1 0");
  step(20);
  elapsed = observation->simTime - firstTime;
  Near(observation->pose.X(), firstPose.X() + vx * elapsed,
      "非法命令改变了连续航迹");

  send("STOP");
  step(1);
  const auto stopped = observation->pose;
  const auto stoppedNose = observation->nose;
  const auto stoppedTail = observation->tail;
  step(20);
  Near(observation->pose.X(), stopped.X(), "STOP 后靶机仍在移动");
  Near(observation->pose.Y(), stopped.Y(), "STOP 后靶机仍在移动");
  SameVisualPoints(*observation, stoppedNose, stoppedTail, "STOP 后未保持朝向");

  // 四个象限和正交方向均从实际位移推导朝向，不依赖 command 的自报 yaw。
  for (const double direction : {0.0, 1.5707963267948966,
      3.141592653589793, -1.5707963267948966, 2.3, -2.1, -0.713})
  {
    std::ostringstream command;
    command << std::setprecision(17)
            << "START_CONTINUOUS target 100 -200 321 35 " << direction;
    send(command.str());
    step(1);
    const auto before = observation->pose;
    step(10);
    NoseFollowsDisplacement(*observation, observation->pose.Pos() - before.Pos(),
        "多航向连续航迹机头与实际位移方向不一致");
  }

  // 私有 START 中故意给出与起终点方向不一致的 yaw，机头仍须沿真实航迹。
  send("START target 4 -8 123 10 0 10 -2.0");
  step(1);
  const auto finiteStart = observation->pose;
  step(4);
  const auto finiteDisplacement = observation->pose.Pos() - finiteStart.Pos();
  NoseFollowsDisplacement(*observation, finiteDisplacement,
      "有限航迹机头仍受不一致的输入 yaw 影响");
  step(20);
  Near(observation->pose.X(), 10.0, "有限航迹终点 X 错误");
  Near(observation->pose.Y(), 0.0, "有限航迹终点 Y 错误");
  Near(observation->pose.Z(), 123.0, "有限航迹高度错误");
  Require(status.WaitFor([](const Status &value)
      { return value.model == "target" && value.done == 1 &&
          value.speed == 0.0 && value.x == 10.0 && value.y == 0.0; }),
      "有限航迹未报告到达终点");
  NoseFollowsDisplacement(*observation, finiteDisplacement,
      "有限航迹到达终点时改变了朝向");
  const auto endpointNose = observation->nose;
  const auto endpointTail = observation->tail;
  step(20);
  Near(observation->pose.X(), 10.0, "有限航迹未停留在终点");
  Near(observation->pose.Y(), 0.0, "有限航迹未停留在终点");
  SameVisualPoints(*observation, endpointNose, endpointTail, "终点等待时未保持朝向");

  // STOP 后允许重新启动完全相同的连续配置。
  send("STOP");
  step(1);
  send(continuous);
  step(2);
  Require(observation->pose.X() >= 15000.0 &&
      observation->pose.X() < 15010.0,
      "STOP 后不能重新开始相同连续航迹");

  // 圆形编队共享同一相位，成员偏移随实时切向旋转。
  send("START_GROUP circle 2 100 200 300 0 0 10 0 50 1 20 5 target target_2");
  step(20);
  const auto circleLeader = observation->pose;
  const auto circleMember = observation->secondPose;
  Near(std::hypot(circleLeader.X() - 100.0, circleLeader.Y() - 250.0),
      50.0, "圆形航迹半径错误");
  Near(circleMember.Z() - circleLeader.Z(), 5.0, "圆形编队高度差错误");
  const gz::math::Vector3d circleOffset =
      circleMember.Pos() - circleLeader.Pos();
  Near(std::hypot(circleOffset.X(), circleOffset.Y()), 20.0,
      "圆形编队横向间距错误");
  const gz::math::Vector3d circleLeft(
      -std::sin(circleLeader.Rot().Yaw()),
      std::cos(circleLeader.Rot().Yaw()), 0.0);
  Near(circleOffset.Dot(circleLeft), 20.0,
      "圆形编队偏移未随实时航向旋转");

  // 单独冻结一架靶机时，其余成员继续沿圆周运行。
  send("FREEZE target_2");
  step(1);
  const auto frozenMember = observation->secondPose;
  const auto movingLeader = observation->pose;
  step(20);
  Near((observation->secondPose.Pos() - frozenMember.Pos()).Length(), 0.0,
      "FREEZE 后成员仍在移动");
  Require((observation->pose.Pos() - movingLeader.Pos()).Length() > 1.0,
      "冻结一个成员导致基准机停止");

  // 正弦轨迹按主航向推进，并对编队应用轨迹法向偏移。
  send("START_GROUP sine 2 0 0 80 200 0 10 0 20 100 15 -3 target target_2");
  step(25);
  const auto sineLeader = observation->pose;
  const auto sineMember = observation->secondPose;
  Require(sineLeader.X() > 14.0 && sineLeader.X() < 25.0,
      "正弦航迹未沿主方向推进");
  Require(sineLeader.Y() > 15.0,
      "正弦航迹未达到配置振幅附近");
  Near(sineMember.Z() - sineLeader.Z(), -3.0, "正弦编队高度差错误");
  const auto sineOffset = sineMember.Pos() - sineLeader.Pos();
  Near(std::hypot(sineOffset.X(), sineOffset.Y()), 15.0,
      "正弦编队横向间距错误");

  // 非法分组命令不能把运行中的正弦航迹跳到新起点。
  send("START_GROUP circle 2 999 999 999 0 0 10 0 0 1 10 5 target target_2");
  step(5);
  Require(observation->pose.X() < 100.0 && observation->pose.Y() < 100.0,
      "非法分组命令改变了当前航迹");

  // 分组直线使用同一进度并保持固定横向与高度偏移。
  send("START_GROUP line 2 -10 0 60 30 0 10 0 0 0 -12 4 target target_2");
  step(10);
  Near(observation->pose.Y(), 0.0, "分组直线基准机横向位置错误");
  Near(observation->secondPose.Y(), -12.0, "分组直线横向间距错误");
  Near(observation->secondPose.Z() - observation->pose.Z(), 4.0,
      "分组直线高度差错误");

  // 分组持续直飞可以单独停止成员，基准机继续运行。
  send("START_GROUP continuous 2 5 6 70 0 0 12 0.4 0 0 9 2 target target_2");
  step(1);
  const auto continuousMember = observation->secondPose;
  send("STOP target_2");
  step(1);
  const auto stoppedMember = observation->secondPose;
  const auto continuousLeader = observation->pose;
  step(20);
  Near((observation->secondPose.Pos() - stoppedMember.Pos()).Length(), 0.0,
      "STOP 单个成员后仍在移动");
  Require((observation->pose.Pos() - continuousLeader.Pos()).Length() > 1.0,
      "STOP 单个成员导致基准机停止");
  Require((continuousMember.Pos() - stoppedMember.Pos()).Length() < 2.0,
      "STOP 单个成员时发生异常跳变");

  send("STOP_ALL");
  step(1);
  const auto stoppedGroup = observation->pose;
  step(10);
  Near((observation->pose.Pos() - stoppedGroup.Pos()).Length(), 0.0,
      "STOP_ALL 后基准机仍在移动");
}
}  // namespace

int main()
{
  try
  {
    Run();
    std::cout << "连续航迹与有限航迹 Gazebo 集成测试通过\n";
    return EXIT_SUCCESS;
  }
  catch (const std::exception &error)
  {
    std::cerr << "航迹 Gazebo 集成测试失败：" << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
