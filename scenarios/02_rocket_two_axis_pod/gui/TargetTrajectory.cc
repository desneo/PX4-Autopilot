// 场景 02 靶机航迹控制器：按仿真时间直接推进视觉模型位姿。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>

namespace scenario02
{
class TargetTrajectory final : public gz::sim::System,
                               public gz::sim::ISystemConfigure,
                               public gz::sim::ISystemPreUpdate
{
  struct Command
  {
    enum class Action {None, Start, Stop};
    Action action{Action::None};
    std::string model;
    double startX{0.0};
    double startY{0.0};
    double z{0.0};
    double endX{0.0};
    double endY{0.0};
    double speed{0.0};
    double yaw{0.0};
    std::uint64_t generation{0};
  };

public:
  void Configure(const gz::sim::Entity &,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &,
                 gz::sim::EventManager &) override
  {
    this->commandTopic = _sdf->Get<std::string>(
        "command_topic", "/scenario/02/target/trajectory/command").first;
    this->statusTopic = _sdf->Get<std::string>(
        "status_topic", "/scenario/02/target/trajectory/status").first;
    this->statusPublisher =
        this->node.Advertise<gz::msgs::StringMsg>(this->statusTopic);
    this->node.Subscribe(this->commandTopic, &TargetTrajectory::OnCommand, this);
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override
  {
    Command command;
    {
      std::lock_guard<std::mutex> guard(this->mutex);
      command = this->pending;
    }
    if (command.generation != this->appliedGeneration)
    {
      this->appliedGeneration = command.generation;
      if (command.action == Command::Action::Stop)
      {
        this->active = false;
        this->entity = gz::sim::kNullEntity;
      }
      else if (command.action == Command::Action::Start)
      {
        this->current = command;
        this->progress = 0.0;
        this->active = true;
        this->done = false;
        this->entity = gz::sim::kNullEntity;
        this->lastPublish = _info.simTime - std::chrono::seconds(1);
      }
    }

    if (!this->active || _info.paused)
      return;

    if (this->entity == gz::sim::kNullEntity)
    {
      this->entity = _ecm.EntityByComponents(
          gz::sim::components::Name(this->current.model),
          gz::sim::components::Model());
      if (this->entity == gz::sim::kNullEntity)
        return;
    }

    const double dx = this->current.endX - this->current.startX;
    const double dy = this->current.endY - this->current.startY;
    const double length = std::hypot(dx, dy);
    if (!this->done && length > 0.0)
    {
      const double dt = std::max(
          0.0, std::chrono::duration<double>(_info.dt).count());
      this->progress = std::min(length, this->progress + this->current.speed * dt);
      this->done = this->progress >= length;
    }
    else if (length <= 0.0)
    {
      this->done = true;
    }

    const double fraction = length > 0.0 ? this->progress / length : 1.0;
    const double x = this->current.startX + fraction * dx;
    const double y = this->current.startY + fraction * dy;
    const gz::math::Pose3d pose(
        x, y, this->current.z, 0.0, 0.0, this->current.yaw);
    gz::sim::Model(this->entity).SetWorldPoseCmd(_ecm, pose);

    if (_info.simTime - this->lastPublish >= std::chrono::milliseconds(50))
    {
      gz::msgs::StringMsg status;
      std::ostringstream stream;
      stream << std::setprecision(12) << "POSE " << this->current.model << ' '
             << x << ' ' << y << ' ' << this->current.z << ' '
             << (this->done ? 0.0 : this->current.speed) << ' '
             << (this->done ? 1 : 0);
      status.set_data(stream.str());
      this->statusPublisher.Publish(status);
      this->lastPublish = _info.simTime;
    }
  }

private:
  void OnCommand(const gz::msgs::StringMsg &_message)
  {
    std::istringstream stream(_message.data());
    std::string action;
    stream >> action;
    Command command;
    if (action == "STOP")
    {
      command.action = Command::Action::Stop;
    }
    else if (action == "START")
    {
      command.action = Command::Action::Start;
      stream >> command.model >> command.startX >> command.startY >> command.z
             >> command.endX >> command.endY >> command.speed >> command.yaw;
      if (!stream || command.model.empty() || !std::isfinite(command.startX) ||
          !std::isfinite(command.startY) || !std::isfinite(command.z) ||
          !std::isfinite(command.endX) || !std::isfinite(command.endY) ||
          !std::isfinite(command.speed) || command.speed <= 0.0 ||
          !std::isfinite(command.yaw))
        return;
    }
    else
    {
      return;
    }

    std::lock_guard<std::mutex> guard(this->mutex);
    command.generation = ++this->generation;
    this->pending = command;
  }

  gz::transport::Node node;
  gz::transport::Node::Publisher statusPublisher;
  std::string commandTopic;
  std::string statusTopic;
  std::mutex mutex;
  Command pending;
  Command current;
  std::uint64_t generation{0};
  std::uint64_t appliedGeneration{0};
  gz::sim::Entity entity{gz::sim::kNullEntity};
  std::chrono::steady_clock::duration lastPublish{};
  double progress{0.0};
  bool active{false};
  bool done{false};
};
}  // namespace scenario02

GZ_ADD_PLUGIN(scenario02::TargetTrajectory,
              gz::sim::System,
              scenario02::TargetTrajectory::ISystemConfigure,
              scenario02::TargetTrajectory::ISystemPreUpdate)
