// 场景 02 靶机航迹控制器：按仿真时间推进一组视觉模型位姿。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

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
  enum class Trajectory {Line, Continuous, Circle, Sine};

  struct GroupConfig
  {
    Trajectory trajectory{Trajectory::Line};
    std::vector<std::string> models;
    double startX{0.0};
    double startY{0.0};
    double z{0.0};
    double endX{0.0};
    double endY{0.0};
    double speed{0.0};
    double yaw{0.0};
    double shape1{0.0};
    double shape2{0.0};
    double lateralSpacing{0.0};
    double altitudeStep{0.0};
  };

  struct Command
  {
    enum class Action {Start, StopAll, StopMember, FreezeMember};
    Action action{Action::StopAll};
    GroupConfig group;
    std::string model;
    std::string signature;
  };

  struct Member
  {
    std::string model;
    gz::sim::Entity entity{gz::sim::kNullEntity};
    double lateralOffset{0.0};
    double altitudeOffset{0.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double yaw{0.0};
    bool active{true};
    bool frozen{false};
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
    std::vector<Command> commands;
    {
      std::lock_guard<std::mutex> guard(this->mutex);
      commands.swap(this->pending);
    }
    for (const auto &command : commands)
      this->Apply(command, _info);

    if (!this->active || _info.paused)
      return;

    const double dt = std::max(
        0.0, std::chrono::duration<double>(_info.dt).count());
    this->Advance(dt);

    bool hasActiveMember = false;
    for (auto &member : this->members)
    {
      if (!member.active)
        continue;
      hasActiveMember = true;
      if (member.entity == gz::sim::kNullEntity)
      {
        member.entity = _ecm.EntityByComponents(
            gz::sim::components::Name(member.model),
            gz::sim::components::Model());
      }
      if (member.entity == gz::sim::kNullEntity)
        continue;

      if (!member.frozen)
      {
        const double normalX = -std::sin(this->referenceYaw);
        const double normalY = std::cos(this->referenceYaw);
        member.x = this->referenceX + member.lateralOffset * normalX;
        member.y = this->referenceY + member.lateralOffset * normalY;
        member.z = this->current.z + member.altitudeOffset;
        member.yaw = this->referenceYaw;
      }
      const gz::math::Pose3d pose(
          member.x, member.y, member.z, 0.0, 0.0, member.yaw);
      gz::sim::Model(member.entity).SetWorldPoseCmd(_ecm, pose);
    }
    if (!hasActiveMember)
      this->active = false;

    if (_info.simTime - this->lastPublish >= std::chrono::milliseconds(50))
    {
      for (const auto &member : this->members)
      {
        if (!member.active || member.entity == gz::sim::kNullEntity)
          continue;
        gz::msgs::StringMsg status;
        std::ostringstream stream;
        const bool memberDone = this->done || member.frozen;
        stream << std::setprecision(12) << "POSE " << member.model << ' '
               << member.x << ' ' << member.y << ' ' << member.z << ' '
               << (memberDone ? 0.0 : this->current.speed) << ' '
               << (memberDone ? 1 : 0) << ' ' << member.yaw;
        status.set_data(stream.str());
        this->statusPublisher.Publish(status);
      }
      this->lastPublish = _info.simTime;
    }
  }

private:
  static bool Finite(const GroupConfig &_group)
  {
    return std::isfinite(_group.startX) && std::isfinite(_group.startY) &&
        std::isfinite(_group.z) && std::isfinite(_group.endX) &&
        std::isfinite(_group.endY) && std::isfinite(_group.speed) &&
        std::isfinite(_group.yaw) && std::isfinite(_group.shape1) &&
        std::isfinite(_group.shape2) &&
        std::isfinite(_group.lateralSpacing) &&
        std::isfinite(_group.altitudeStep);
  }

  static bool Valid(const GroupConfig &_group)
  {
    if (_group.models.empty() || _group.models.size() > 4 ||
        !Finite(_group) || _group.speed <= 0.0)
      return false;
    std::unordered_set<std::string> names;
    for (const auto &model : _group.models)
    {
      if (model.empty() || !names.insert(model).second)
        return false;
    }
    if (_group.trajectory == Trajectory::Circle)
      return _group.shape1 > 0.0 && _group.shape2 != 0.0;
    if (_group.trajectory == Trajectory::Sine)
      return _group.shape1 >= 0.0 && _group.shape2 > 0.0 &&
          std::hypot(_group.endX - _group.startX,
                     _group.endY - _group.startY) > 0.0;
    return true;
  }

  static bool ParseTrajectory(const std::string &_value,
                              Trajectory &_trajectory)
  {
    if (_value == "line")
      _trajectory = Trajectory::Line;
    else if (_value == "continuous")
      _trajectory = Trajectory::Continuous;
    else if (_value == "circle")
      _trajectory = Trajectory::Circle;
    else if (_value == "sine")
      _trajectory = Trajectory::Sine;
    else
      return false;
    return true;
  }

  bool ParseStartGroup(std::istringstream &_stream, GroupConfig &_group)
  {
    std::string type;
    int count = 0;
    _stream >> type >> count >> _group.startX >> _group.startY >> _group.z
            >> _group.endX >> _group.endY >> _group.speed >> _group.yaw
            >> _group.shape1 >> _group.shape2 >> _group.lateralSpacing
            >> _group.altitudeStep;
    if (!_stream || count < 1 || count > 4 ||
        !ParseTrajectory(type, _group.trajectory))
      return false;
    _group.models.resize(static_cast<std::size_t>(count));
    for (auto &model : _group.models)
      _stream >> model;
    std::string extra;
    return _stream && !(_stream >> extra) && Valid(_group);
  }

  bool ParseLegacy(const std::string &_action,
                   std::istringstream &_stream,
                   GroupConfig &_group)
  {
    std::string model;
    _stream >> model >> _group.startX >> _group.startY >> _group.z;
    if (_action == "START")
    {
      _group.trajectory = Trajectory::Line;
      _stream >> _group.endX >> _group.endY;
    }
    else
    {
      _group.trajectory = Trajectory::Continuous;
    }
    _stream >> _group.speed >> _group.yaw;
    std::string extra;
    if (!_stream || _stream >> extra)
      return false;
    _group.models = {model};
    return Valid(_group);
  }

  void OnCommand(const gz::msgs::StringMsg &_message)
  {
    std::istringstream stream(_message.data());
    std::string action;
    stream >> action;
    Command command;
    if (action == "START" || action == "START_CONTINUOUS")
    {
      command.action = Command::Action::Start;
      if (!this->ParseLegacy(action, stream, command.group))
        return;
    }
    else if (action == "START_GROUP")
    {
      command.action = Command::Action::Start;
      if (!this->ParseStartGroup(stream, command.group))
        return;
    }
    else if (action == "FREEZE")
    {
      command.action = Command::Action::FreezeMember;
      std::string extra;
      if (!(stream >> command.model) || command.model.empty() || stream >> extra)
        return;
    }
    else if (action == "STOP")
    {
      std::string extra;
      if (stream >> command.model)
      {
        command.action = Command::Action::StopMember;
        if (command.model.empty() || stream >> extra)
          return;
      }
      else
      {
        command.action = Command::Action::StopAll;
      }
    }
    else if (action == "STOP_ALL")
    {
      std::string extra;
      if (stream >> extra)
        return;
      command.action = Command::Action::StopAll;
    }
    else
    {
      return;
    }

    command.signature = _message.data();
    std::lock_guard<std::mutex> guard(this->mutex);
    // 管理服务会重发启动命令，直到收到 POSE；同一组不能因此跳回起点。
    if (command.action == Command::Action::Start)
    {
      if (command.signature == this->lastStartSignature)
        return;
      this->lastStartSignature = command.signature;
    }
    else if (command.action == Command::Action::StopAll)
    {
      this->lastStartSignature.clear();
    }
    this->pending.push_back(std::move(command));
  }

  void Apply(const Command &_command, const gz::sim::UpdateInfo &_info)
  {
    if (_command.action == Command::Action::StopAll)
    {
      this->active = false;
      for (auto &member : this->members)
      {
        member.active = false;
        member.entity = gz::sim::kNullEntity;
      }
      return;
    }
    if (_command.action == Command::Action::StopMember ||
        _command.action == Command::Action::FreezeMember)
    {
      for (auto &member : this->members)
      {
        if (member.model != _command.model)
          continue;
        if (_command.action == Command::Action::StopMember)
        {
          member.active = false;
          member.entity = gz::sim::kNullEntity;
        }
        else
        {
          member.frozen = true;
        }
        break;
      }
      return;
    }

    this->current = _command.group;
    this->members.clear();
    this->members.reserve(this->current.models.size());
    for (std::size_t index = 0; index < this->current.models.size(); ++index)
    {
      Member member;
      member.model = this->current.models[index];
      member.lateralOffset = static_cast<double>(index) *
          this->current.lateralSpacing;
      member.altitudeOffset = static_cast<double>(index) *
          this->current.altitudeStep;
      this->members.push_back(std::move(member));
    }
    this->progress = 0.0;
    this->referenceX = this->current.startX;
    this->referenceY = this->current.startY;
    this->referenceYaw = this->InitialYaw();
    this->done = false;
    this->active = true;
    this->lastPublish = _info.simTime - std::chrono::seconds(1);
  }

  double InitialYaw() const
  {
    if (this->current.trajectory == Trajectory::Line ||
        this->current.trajectory == Trajectory::Sine)
    {
      const double dx = this->current.endX - this->current.startX;
      const double dy = this->current.endY - this->current.startY;
      if (std::hypot(dx, dy) > 0.0)
        return std::atan2(dy, dx);
    }
    return this->current.yaw;
  }

  void Advance(double _dt)
  {
    const double dx = this->current.endX - this->current.startX;
    const double dy = this->current.endY - this->current.startY;
    const double length = std::hypot(dx, dy);
    if (this->current.trajectory == Trajectory::Continuous)
    {
      this->progress += this->current.speed * _dt;
      this->referenceX = this->current.startX +
          this->progress * std::cos(this->current.yaw);
      this->referenceY = this->current.startY +
          this->progress * std::sin(this->current.yaw);
      this->referenceYaw = this->current.yaw;
      return;
    }
    if (this->current.trajectory == Trajectory::Circle)
    {
      this->progress += this->current.speed * _dt;
      const double radius = this->current.shape1;
      const double turn = this->current.shape2 > 0.0 ? 1.0 : -1.0;
      const double angle = turn * this->progress / radius;
      const double normalX = -std::sin(this->current.yaw);
      const double normalY = std::cos(this->current.yaw);
      const double centerX = this->current.startX + turn * radius * normalX;
      const double centerY = this->current.startY + turn * radius * normalY;
      const double radialX = this->current.startX - centerX;
      const double radialY = this->current.startY - centerY;
      this->referenceX = centerX + radialX * std::cos(angle) -
          radialY * std::sin(angle);
      this->referenceY = centerY + radialX * std::sin(angle) +
          radialY * std::cos(angle);
      this->referenceYaw = this->current.yaw + angle;
      return;
    }

    double advance = this->current.speed * _dt;
    if (this->current.trajectory == Trajectory::Sine)
    {
      constexpr double kPi = 3.14159265358979323846;
      const double wave = 2.0 * kPi / this->current.shape2;
      const double slope = this->current.shape1 * wave *
          std::cos(wave * this->progress);
      // progress 是主轴距离。按局部弧长换算，避免摆幅越大实际速度越快。
      advance /= std::hypot(1.0, slope);
    }
    if (!this->done && length > 0.0)
    {
      this->progress = std::min(
          length, this->progress + advance);
      this->done = this->progress >= length;
    }
    else if (length <= 0.0)
    {
      this->done = true;
    }
    const double tangentX = length > 0.0 ? dx / length :
        std::cos(this->current.yaw);
    const double tangentY = length > 0.0 ? dy / length :
        std::sin(this->current.yaw);
    if (this->current.trajectory == Trajectory::Sine)
    {
      const double normalX = -tangentY;
      const double normalY = tangentX;
      constexpr double kPi = 3.14159265358979323846;
      const double wave = 2.0 * kPi / this->current.shape2;
      const double offset = this->current.shape1 *
          std::sin(wave * this->progress);
      const double slope = this->current.shape1 * wave *
          std::cos(wave * this->progress);
      this->referenceX = this->current.startX + tangentX * this->progress +
          normalX * offset;
      this->referenceY = this->current.startY + tangentY * this->progress +
          normalY * offset;
      this->referenceYaw = std::atan2(
          tangentY + normalY * slope, tangentX + normalX * slope);
    }
    else
    {
      this->referenceX = this->current.startX + tangentX * this->progress;
      this->referenceY = this->current.startY + tangentY * this->progress;
      this->referenceYaw = std::atan2(tangentY, tangentX);
    }
  }

  gz::transport::Node node;
  gz::transport::Node::Publisher statusPublisher;
  std::string commandTopic;
  std::string statusTopic;
  std::mutex mutex;
  std::vector<Command> pending;
  std::string lastStartSignature;
  GroupConfig current;
  std::vector<Member> members;
  std::chrono::steady_clock::duration lastPublish{};
  double progress{0.0};
  double referenceX{0.0};
  double referenceY{0.0};
  double referenceYaw{0.0};
  bool active{false};
  bool done{false};
};
}  // namespace scenario02

GZ_ADD_PLUGIN(scenario02::TargetTrajectory,
              gz::sim::System,
              scenario02::TargetTrajectory::ISystemConfigure,
              scenario02::TargetTrajectory::ISystemPreUpdate)
