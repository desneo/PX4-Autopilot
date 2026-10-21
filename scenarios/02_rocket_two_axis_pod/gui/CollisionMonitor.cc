// 场景 02 碰撞监控器：原生 Gazebo 接触与轻量靶机包围盒检测。
#include "CollisionGeometry.hh"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>

namespace scenario02
{
class CollisionMonitor final : public gz::sim::System,
                               public gz::sim::ISystemConfigure,
                               public gz::sim::ISystemPostUpdate
{
  struct Snapshot
  {
    gz::sim::Entity entity{gz::sim::kNullEntity};
    std::string name;
    gz::math::Pose3d pose;
    gz::math::Vector3d velocity{0, 0, 0};
    bool isStatic{false};
    int aircraftId{0};
  };

  struct NativeContact
  {
    Snapshot first;
    Snapshot second;
    gz::math::Vector3d position{0, 0, 0};
  };

public:
  void Configure(const gz::sim::Entity &,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &,
                 gz::sim::EventManager &) override
  {
    this->eventTopic = _sdf->Get<std::string>(
        "event_topic", "/scenario/02/collision/event").first;
    this->aircraftPrefix = _sdf->Get<std::string>(
        "aircraft_prefix", "rocket_two_axis_pod_").first;
    this->airborneHeight = _sdf->Get<double>("airborne_height_m", 1.0).first;
    this->publisher = this->node.Advertise<gz::msgs::StringMsg>(this->eventTopic);
    if (_sdf->HasElement("mesh_collision"))
    {
      auto item = _sdf->FindElement("mesh_collision");
      while (item)
      {
        for (const auto *field : {"model", "link", "collision", "center", "half_extents"})
          if (!item->HasElement(field))
            throw std::runtime_error(std::string("mesh_collision missing ") + field);
        const auto model = item->Get<std::string>("model");
        const auto link = item->Get<std::string>("link");
        const auto collision = item->Get<std::string>("collision");
        LocalBounds bounds{item->Get<gz::math::Vector3d>("center"),
                           item->Get<gz::math::Vector3d>("half_extents")};
        if (model.empty() || link.empty() || collision.empty() ||
            !bounds.center.IsFinite() || !bounds.halfExtents.IsFinite() ||
            bounds.halfExtents.X() <= 0 || bounds.halfExtents.Y() <= 0 || bounds.halfExtents.Z() <= 0 ||
            !this->meshBounds.emplace(std::make_tuple(model, link, collision), bounds).second)
          throw std::runtime_error("invalid or duplicate mesh_collision bounds");
        item = item->GetNextElement("mesh_collision");
      }
    }
  }

  void PostUpdate(const gz::sim::UpdateInfo &_info,
                  const gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;

    const double dt = std::chrono::duration<double>(_info.dt).count();
    std::unordered_map<gz::sim::Entity, Snapshot> snapshots;
    std::map<std::string, Snapshot> byName;
    _ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
        [&](const gz::sim::Entity &_entity,
            const gz::sim::components::Model *,
            const gz::sim::components::Name *_name)
        {
          Snapshot snapshot;
          snapshot.entity = _entity;
          snapshot.name = _name->Data();
          snapshot.pose = gz::sim::worldPose(_entity, _ecm);
          snapshot.isStatic = gz::sim::Model(_entity).Static(_ecm);
          snapshot.aircraftId = this->AircraftId(snapshot.name);
          const auto previous = this->previousPose.find(snapshot.name);
          if (previous != this->previousPose.end() && dt > 0.0)
            snapshot.velocity =
                (snapshot.pose.Pos() - previous->second.Pos()) / dt;
          snapshots.emplace(_entity, snapshot);
          byName.emplace(snapshot.name, snapshot);
          return true;
        });

    std::map<std::string, NativeContact> nativeContacts;
    std::set<int> surfaceContactAircraft;
    _ecm.Each<gz::sim::components::ContactSensorData>(
        [&](const gz::sim::Entity &,
            const gz::sim::components::ContactSensorData *_data)
        {
          for (const auto &contact : _data->Data().contact())
          {
            const auto firstEntity = this->ContainingModel(
                static_cast<gz::sim::Entity>(contact.collision1().id()), _ecm);
            const auto secondEntity = this->ContainingModel(
                static_cast<gz::sim::Entity>(contact.collision2().id()), _ecm);
            if (firstEntity == gz::sim::kNullEntity ||
                secondEntity == gz::sim::kNullEntity ||
                firstEntity == secondEntity)
              continue;
            const auto first = snapshots.find(firstEntity);
            const auto second = snapshots.find(secondEntity);
            if (first == snapshots.end() || second == snapshots.end())
              continue;

            const bool aircraftPair = first->second.aircraftId > 0 &&
                second->second.aircraftId > 0;
            const bool firstSurface = first->second.aircraftId > 0 &&
                second->second.isStatic;
            const bool secondSurface = second->second.aircraftId > 0 &&
                first->second.isStatic;
            if (!aircraftPair && !firstSurface && !secondSurface)
              continue;

            NativeContact value{first->second, second->second,
                (first->second.pose.Pos() + second->second.pose.Pos()) * 0.5};
            if (contact.position_size() > 0)
            {
              value.position.Set(contact.position(0).x(),
                                 contact.position(0).y(),
                                 contact.position(0).z());
            }
            const std::string key = this->PairKey(
                value.first.name, value.second.name);
            nativeContacts.emplace(key, value);
            if (firstSurface)
              surfaceContactAircraft.insert(first->second.aircraftId);
            if (secondSurface)
              surfaceContactAircraft.insert(second->second.aircraftId);
          }
          return true;
        });

    for (const auto &[name, snapshot] : byName)
    {
      if (snapshot.aircraftId <= 0)
        continue;
      auto [initial, inserted] = this->initialHeight.emplace(
          snapshot.aircraftId, snapshot.pose.Pos().Z());
      if (!this->airborne[snapshot.aircraftId] &&
          surfaceContactAircraft.count(snapshot.aircraftId) == 0 &&
          snapshot.pose.Pos().Z() >= initial->second + this->airborneHeight)
        this->airborne[snapshot.aircraftId] = true;
    }

    std::set<std::string> currentNative;
    for (const auto &[key, contact] : nativeContacts)
    {
      currentNative.insert(key);
      if (this->activeNative.count(key) != 0)
        continue;
      if (contact.first.aircraftId > 0 && contact.second.aircraftId > 0)
      {
        this->Publish("px4_px4", _info, contact.position,
            {contact.first.aircraftId, contact.second.aircraftId},
            contact.first.name, contact.second.name,
            (contact.first.velocity - contact.second.velocity).Length(), true);
      }
      else
      {
        const Snapshot &aircraft = contact.first.aircraftId > 0 ?
            contact.first : contact.second;
        const Snapshot &surface = contact.first.aircraftId > 0 ?
            contact.second : contact.first;
        if (this->airborne[aircraft.aircraftId])
        {
          this->Publish("px4_surface", _info, contact.position,
              {aircraft.aircraftId}, aircraft.name, surface.name,
              aircraft.velocity.Length(), true);
        }
      }
    }
    this->activeNative = std::move(currentNative);

    this->CheckTargetIntersections(_info, byName, _ecm);
    this->previousPose.clear();
    for (const auto &[name, snapshot] : byName)
      this->previousPose.emplace(name, snapshot.pose);
  }

private:
  int AircraftId(const std::string &_name) const
  {
    if (_name.rfind(this->aircraftPrefix, 0) != 0)
      return 0;
    const std::string suffix = _name.substr(this->aircraftPrefix.size());
    if (suffix.empty() || !std::all_of(
        suffix.begin(), suffix.end(),
        [](unsigned char _character) { return std::isdigit(_character) != 0; }))
      return 0;
    try
    {
      const int value = std::stoi(suffix);
      return value > 0 ? value : 0;
    }
    catch (...)
    {
      return 0;
    }
  }

  gz::sim::Entity ContainingModel(
      gz::sim::Entity _entity,
      const gz::sim::EntityComponentManager &_ecm) const
  {
    while (_entity != gz::sim::kNullEntity)
    {
      if (_ecm.Component<gz::sim::components::Model>(_entity))
        return _entity;
      _entity = _ecm.ParentEntity(_entity);
    }
    return gz::sim::kNullEntity;
  }

  static std::string PairKey(std::string _first, std::string _second)
  {
    if (_second < _first)
      std::swap(_first, _second);
    return _first + "|" + _second;
  }

  std::optional<gz::math::Vector3d> TargetHalfExtents(
      const std::string &_name) const
  {
    if (_name == "xiaomotuo")
      return gz::math::Vector3d(1.75, 1.25, 0.40);
    if (_name == "xiaobianfu")
      return gz::math::Vector3d(0.35, 0.55, 0.12);
    return std::nullopt;
  }

  void CheckTargetIntersections(
      const gz::sim::UpdateInfo &_info,
      const std::map<std::string, Snapshot> &_models,
      const gz::sim::EntityComponentManager &_ecm)
  {
    const bool targetPresent = std::any_of(_models.begin(), _models.end(),
        [&](const auto &_item)
        { return this->TargetHalfExtents(_item.first).has_value(); });

    std::map<gz::sim::Entity, std::vector<OrientedBox>> aircraftBoxes;
    for (const auto &[name, snapshot] : _models)
      if (snapshot.aircraftId > 0)
        aircraftBoxes.emplace(snapshot.entity, std::vector<OrientedBox>{});
    _ecm.Each<gz::sim::components::Collision, gz::sim::components::Geometry>(
        [&](const gz::sim::Entity &_entity,
            const gz::sim::components::Collision *,
            const gz::sim::components::Geometry *_geometry)
        {
          const auto aircraft = aircraftBoxes.find(this->ContainingModel(_entity, _ecm));
          if (aircraft == aircraftBoxes.end())
            return true;
          LocalBounds bounds;
          if (_geometry->Data().Type() == sdf::GeometryType::MESH)
          {
            const auto *modelName = _ecm.Component<gz::sim::components::Name>(aircraft->first);
            const auto *linkName = _ecm.Component<gz::sim::components::Name>(_ecm.ParentEntity(_entity));
            const auto *collisionName = _ecm.Component<gz::sim::components::Name>(_entity);
            if (!modelName || !linkName || !collisionName)
              throw std::runtime_error("mesh collision missing model/link/collision identity");
            const auto found = this->meshBounds.find(std::make_tuple(
                modelName->Data(), linkName->Data(), collisionName->Data()));
            if (found == this->meshBounds.end())
              throw std::runtime_error("mesh collision has no validated startup bounds: " +
                  modelName->Data() + "::" + linkName->Data() + "::" + collisionName->Data());
            bounds = found->second;
          }
          else
          {
            const auto half = CollisionHalfExtents(_geometry->Data());
            if (!half)
              throw std::runtime_error("unsupported aircraft collision geometry");
            bounds = {gz::math::Vector3d::Zero, *half};
          }
          if (targetPresent)
            aircraft->second.push_back(WorldCollisionBox(gz::sim::worldPose(_entity, _ecm), bounds));
          return true;
        });

    if (!targetPresent)
    {
      this->activeTarget.clear();
      return;
    }
    std::set<std::string> current;
    for (const auto &[targetName, target] : _models)
    {
      const auto targetHalf = this->TargetHalfExtents(targetName);
      if (!targetHalf)
        continue;
      for (const auto &[aircraftName, aircraft] : _models)
      {
        if (aircraft.aircraftId <= 0)
          continue;
        const std::string key = this->PairKey(targetName, aircraftName);
        const auto &boxes = aircraftBoxes.at(aircraft.entity);
        if (std::none_of(boxes.begin(), boxes.end(), [&](const auto &_box)
            { return Intersects({target.pose, *targetHalf}, _box); }))
          continue;
        current.insert(key);
        if (this->activeTarget.count(key) != 0)
          continue;
        const double relativeSpeed =
            (target.velocity - aircraft.velocity).Length();
        this->Publish("px4_target", _info,
            (target.pose.Pos() + aircraft.pose.Pos()) * 0.5,
            {aircraft.aircraftId}, aircraft.name, target.name,
            relativeSpeed, true);
      }
    }
    this->activeTarget = std::move(current);
  }

  void Publish(const std::string &_type,
               const gz::sim::UpdateInfo &_info,
               const gz::math::Vector3d &_position,
               const std::vector<int> &_aircraftIds,
               const std::string &_first,
               const std::string &_second,
               double _relativeSpeed,
               bool _damageCandidate)
  {
    std::ostringstream stream;
    stream << std::setprecision(12)
           << "{\"schema_version\":1,\"collision_type\":\"" << _type
           << "\",\"simulation_time_s\":"
           << std::chrono::duration<double>(_info.simTime).count()
           << ",\"aircraft_ids\":[";
    for (std::size_t index = 0; index < _aircraftIds.size(); ++index)
    {
      if (index > 0)
        stream << ',';
      stream << _aircraftIds[index];
    }
    stream << "],\"entity_1\":\"" << _first
           << "\",\"entity_2\":\"" << _second
           << "\",\"relative_speed_m_s\":" << _relativeSpeed
           << ",\"damage_candidate\":"
           << (_damageCandidate ? "true" : "false")
           << ",\"position_enu_m\":{\"x\":" << _position.X()
           << ",\"y\":" << _position.Y()
           << ",\"z\":" << _position.Z() << "}}";
    gz::msgs::StringMsg message;
    message.set_data(stream.str());
    this->publisher.Publish(message);
  }

  gz::transport::Node node;
  gz::transport::Node::Publisher publisher;
  std::string eventTopic;
  std::string aircraftPrefix;
  double airborneHeight{1.0};
  std::map<std::string, gz::math::Pose3d> previousPose;
  std::map<int, double> initialHeight;
  std::map<int, bool> airborne;
  std::set<std::string> activeNative;
  std::set<std::string> activeTarget;
  std::map<std::tuple<std::string, std::string, std::string>, LocalBounds> meshBounds;
};
}  // namespace scenario02

GZ_ADD_PLUGIN(scenario02::CollisionMonitor,
              gz::sim::System,
              scenario02::CollisionMonitor::ISystemConfigure,
              scenario02::CollisionMonitor::ISystemPostUpdate)
