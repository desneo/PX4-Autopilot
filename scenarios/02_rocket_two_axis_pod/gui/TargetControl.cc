#include "TargetControl.hh"

#include <cmath>
#include <sstream>
#include <algorithm>
#include <QCoreApplication>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMetaObject>
#include <QSettings>
#include <gz/common/MouseEvent.hh>
#include <gz/common/KeyEvent.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <tinyxml2.h>

namespace scenario_02
{
namespace
{
constexpr double kMinFollowDistanceM = 0.25;
constexpr double kMaxFollowDistanceM = 500.0;
constexpr int kRadarResponseTimeoutMs = 5000;

bool IsValidStationAddress(const QString &_address)
{
  const QString address = _address.trimmed();
  const int separator = address.lastIndexOf(':');
  if (separator <= 0 || separator == address.size() - 1)
    return false;
  bool ok = false;
  const int port = address.mid(separator + 1).toInt(&ok);
  return ok && port >= 1 && port <= 65535;
}
}

TargetControl::TargetControl()
{
  QSettings settings(QSettings::IniFormat, QSettings::UserScope,
      QStringLiteral("PX4"), QStringLiteral("Scenario02TargetControl"));
  const QString savedAddress = settings.value(
      QStringLiteral("stationAddress"), this->stationAddress).toString().trimmed();
  if (IsValidStationAddress(savedAddress))
    this->stationAddress = savedAddress;

  this->radarResponseTimer.setSingleShot(true);
  this->radarResponseTimer.setInterval(kRadarResponseTimeoutMs);
  connect(&this->radarResponseTimer, &QTimer::timeout, this, [this]
  {
    if (!this->radarBusy)
      return;
    this->radarBusy = false;
    this->radarStatus = QStringLiteral(
        "雷达控制响应超时，请检查雷达服务或 WSL 网络后重试");
    emit this->RadarStateChanged();
  });
}

TargetControl::~TargetControl()
{
  this->radarResponseTimer.stop();
  this->SetNativeCameraControlBlocked(false);
}

void TargetControl::LoadConfig(const tinyxml2::XMLElement *_config)
{
  if (this->title.empty())
    this->title = "场景控制";

  if (_config)
  {
    if (const auto *value = _config->FirstChildElement("command_topic");
        value && value->GetText())
      this->commandTopic = QString::fromUtf8(value->GetText());
    if (const auto *value = _config->FirstChildElement("status_topic");
        value && value->GetText())
      this->statusTopic = QString::fromUtf8(value->GetText());
    if (const auto *value = _config->FirstChildElement("radar_command_topic");
        value && value->GetText())
      this->radarCommandTopic = QString::fromUtf8(value->GetText());
    if (const auto *value = _config->FirstChildElement("radar_status_topic");
        value && value->GetText())
      this->radarStatusTopic = QString::fromUtf8(value->GetText());
    if (const auto *value = _config->FirstChildElement("follow_service");
        value && value->GetText())
      this->followService = QString::fromUtf8(value->GetText());
    if (const auto *value = _config->FirstChildElement("move_to_pose_service");
        value && value->GetText())
      this->moveToPoseService = QString::fromUtf8(value->GetText());
    if (const auto *value = _config->FirstChildElement("track_topic");
        value && value->GetText())
      this->trackTopic = QString::fromUtf8(value->GetText());
    if (const auto *value = _config->FirstChildElement("aircraft_count");
        value && value->GetText())
    {
      bool ok = false;
      const int count = QString::fromUtf8(value->GetText()).toInt(&ok);
      if (ok && count > 0)
        this->aircraftCount = count;
    }
    if (const auto *value = _config->FirstChildElement("center_pose");
        value && value->GetText())
    {
      std::istringstream input(value->GetText());
      gz::math::Pose3d pose;
      if (input >> pose && pose.IsFinite())
        this->centerPose = pose;
    }
    if (const auto *value = _config->FirstChildElement("default_altitude_m");
        value && value->GetText())
    {
      bool ok = false;
      const double altitude = QString::fromUtf8(value->GetText()).toDouble(&ok);
      if (ok && std::isfinite(altitude))
        this->defaultAltitudeM = altitude;
    }
  }

  this->publisher = this->node.Advertise<gz::msgs::StringMsg>(
      this->commandTopic.toStdString());
  this->radarPublisher = this->node.Advertise<gz::msgs::StringMsg>(
      this->radarCommandTopic.toStdString());
  this->trackingPublisher = this->node.Advertise<gz::msgs::CameraTrack>(
      this->trackTopic.toStdString());
  const bool subscribed = this->node.Subscribe(
      this->statusTopic.toStdString(), &TargetControl::OnStatus, this);
  const bool radarSubscribed = this->node.Subscribe(
      this->radarStatusTopic.toStdString(), &TargetControl::OnRadarStatus, this);
  if (!this->publisher.Valid() || !subscribed)
    this->status = QStringLiteral("靶机控制通信初始化失败");
  if (!this->radarPublisher.Valid() || !radarSubscribed)
    this->radarStatus = QStringLiteral("雷达控制通信初始化失败");
  if (auto *window = gz::gui::App()->findChild<gz::gui::MainWindow *>())
    window->installEventFilter(this);
  emit this->StateChanged();
  emit this->RadarStateChanged();
  emit this->CameraStateChanged();
}

bool TargetControl::eventFilter(QObject *_object, QEvent *_event)
{
  if (this->following && !this->cameraBusy &&
      _event->type() == gz::gui::events::MousePressOnScene::kType)
  {
    const auto *press = static_cast<gz::gui::events::MousePressOnScene *>(
        _event);
    const auto &mouse = press->Mouse();
    this->lastMousePosition = mouse.Pos();
    this->followDragActive =
        mouse.Button() == gz::common::MouseEvent::MIDDLE ||
        (mouse.Buttons() & gz::common::MouseEvent::MIDDLE) ||
        ((mouse.Button() == gz::common::MouseEvent::LEFT ||
          (mouse.Buttons() & gz::common::MouseEvent::LEFT)) && mouse.Shift());
  }
  else if (this->following && !this->cameraBusy &&
      _event->type() == gz::gui::events::DragOnScene::kType)
  {
    const auto *drag = static_cast<gz::gui::events::DragOnScene *>(_event);
    const auto mouse = drag->Mouse();
    const bool orbitGesture =
        mouse.Button() == gz::common::MouseEvent::MIDDLE ||
        (mouse.Buttons() & gz::common::MouseEvent::MIDDLE) ||
        ((mouse.Button() == gz::common::MouseEvent::LEFT ||
          (mouse.Buttons() & gz::common::MouseEvent::LEFT)) && mouse.Shift());
    if (orbitGesture)
    {
      if (!this->followDragActive)
        this->lastMousePosition = mouse.PrevPos();
      this->OrbitFollow(mouse.Pos() - this->lastMousePosition);
      this->lastMousePosition = mouse.Pos();
      this->followDragActive = true;
    }
  }
  else if (this->following && !this->cameraBusy &&
      _event->type() == gz::gui::events::ScrollOnScene::kType)
  {
    const auto *scroll = static_cast<gz::gui::events::ScrollOnScene *>(_event);
    this->ZoomFollow(scroll->Mouse().Scroll().Y());
  }
  else if (_event->type() == gz::gui::events::KeyReleaseOnScene::kType)
  {
    const auto *release = static_cast<gz::gui::events::KeyReleaseOnScene *>(
        _event);
    if (this->following && release->Key().Key() == Qt::Key_Escape)
    {
      this->SetNativeCameraControlBlocked(false);
      this->following = false;
      this->followingEntity.clear();
      this->followingLabel.clear();
      this->followDragActive = false;
      this->cameraStatus = QStringLiteral("已停止跟随，当前为自由视角");
      emit this->CameraStateChanged();
    }
  }

  return QObject::eventFilter(_object, _event);
}

void TargetControl::ClearCameraTracking()
{
  if (auto *window = gz::gui::App()->findChild<gz::gui::MainWindow *>())
  {
    gz::common::KeyEvent key;
    key.SetKey(Qt::Key_Escape);
    key.SetType(gz::common::KeyEvent::RELEASE);
    gz::gui::events::KeyReleaseOnScene escape(key);
    QCoreApplication::sendEvent(window, &escape);
  }
}

void TargetControl::SetNativeCameraControlBlocked(bool _blocked)
{
  if (auto *window = gz::gui::App()->findChild<gz::gui::MainWindow *>())
  {
    gz::gui::events::BlockOrbit block(_blocked);
    QCoreApplication::sendEvent(window, &block);
  }
}

void TargetControl::OrbitFollow(const gz::math::Vector2i &_drag)
{
  if (_drag == gz::math::Vector2i::Zero)
    return;
  const double radius = std::max(
      this->followOffset.Length(), kMinFollowDistanceM);
  double azimuth = std::atan2(this->followOffset.Y(), this->followOffset.X());
  double elevation = std::atan2(this->followOffset.Z(),
      std::hypot(this->followOffset.X(), this->followOffset.Y()));
  azimuth -= static_cast<double>(_drag.X()) * 0.006;
  elevation = std::clamp(
      elevation + static_cast<double>(_drag.Y()) * 0.006,
      0.035, 1.483);
  const double horizontal = radius * std::cos(elevation);
  this->followOffset.Set(
      horizontal * std::cos(azimuth),
      horizontal * std::sin(azimuth),
      radius * std::sin(elevation));
  this->PublishFollow(this->followingEntity);
}

void TargetControl::ZoomFollow(int _scrollSteps)
{
  if (_scrollSteps == 0)
    return;
  double steps = static_cast<double>(_scrollSteps);
  if (std::abs(steps) > 10.0)
    steps /= 120.0;
  steps = std::clamp(steps, -4.0, 4.0);
  const double currentRadius = std::max(
      this->followOffset.Length(), kMinFollowDistanceM);
  // 当前 Gazebo 鼠标事件中，滚轮向前为负值、向后为正值。
  // 因此负值应缩短半径，正值应增大半径。
  const double newRadius = std::clamp(
      currentRadius * std::exp(0.18 * steps),
      kMinFollowDistanceM, kMaxFollowDistanceM);
  this->followOffset *= newRadius / currentRadius;
  this->PublishFollow(this->followingEntity);
}

void TargetControl::PublishFollow(const QString &_entity)
{
  if (!this->trackingPublisher.Valid())
    return;
  gz::msgs::CameraTrack message;
  message.set_track_mode(gz::msgs::CameraTrack::FOLLOW);
  message.mutable_follow_target()->set_name(_entity.toStdString());
  gz::msgs::Set(message.mutable_follow_offset(), this->followOffset);
  // gz-gui 8 的 FOLLOW 实现交叉使用这两个增益；都设为 1 可避免高速飞机
  // 下相机明显滞后，同时保证恢复交互视角时不会弹回旧位置。
  message.set_follow_pgain(1.0);
  message.set_track_pgain(1.0);
  this->trackingPublisher.Publish(message);
}

void TargetControl::FollowEntity(
    const QString &_entity, const QString &_label)
{
  if (this->cameraBusy || _entity.trimmed().isEmpty())
    return;

  gz::msgs::StringMsg request;
  request.set_data(_entity.trimmed().toStdString());
  this->pendingFollowingEntity = _entity.trimmed();
  this->pendingFollowingLabel = _label.trimmed();
  this->cameraBusy = true;
  this->cameraStatus = QStringLiteral("正在定位并跟随 %1…")
      .arg(this->pendingFollowingLabel);
  emit this->CameraStateChanged();

  if (!this->node.Request(this->followService.toStdString(), request,
      &TargetControl::OnFollowResponse, this))
  {
    this->cameraBusy = false;
    this->pendingFollowingEntity.clear();
    this->pendingFollowingLabel.clear();
    this->cameraStatus = QStringLiteral("相机跟随服务尚未就绪");
    emit this->CameraStateChanged();
  }
}

void TargetControl::OnFollowResponse(
    const gz::msgs::Boolean &_response, bool _result)
{
  const bool success = _result && _response.data();
  QMetaObject::invokeMethod(this, [this, success]
  {
    this->cameraBusy = false;
    if (success)
    {
      this->followOffset.Set(-12.0, 0.0, 6.0);
      this->PublishFollow(this->pendingFollowingEntity);
      this->SetNativeCameraControlBlocked(true);
      this->following = true;
      this->followingEntity = this->pendingFollowingEntity;
      this->followingLabel = this->pendingFollowingLabel;
      this->cameraStatus = QStringLiteral(
          "视野正在跟随 %1；滚轮前推靠近/后拉远离，中键拖动旋转")
          .arg(this->followingLabel);
    }
    else
    {
      this->cameraStatus = QStringLiteral("未找到所选飞机，请确认模型已经生成");
    }
    this->pendingFollowingEntity.clear();
    this->pendingFollowingLabel.clear();
    emit this->CameraStateChanged();
  }, Qt::QueuedConnection);
}

void TargetControl::StopFollowing()
{
  if (this->cameraBusy)
    return;
  this->SetNativeCameraControlBlocked(false);
  this->followDragActive = false;
  this->ClearCameraTracking();
  this->following = false;
  this->followingEntity.clear();
  this->followingLabel.clear();
  this->cameraStatus = QStringLiteral("已停止跟随，当前为自由视角");
  emit this->CameraStateChanged();
}

void TargetControl::ReturnToMapCenter()
{
  if (this->cameraBusy)
    return;

  this->SetNativeCameraControlBlocked(false);
  this->followDragActive = false;
  this->ClearCameraTracking();
  gz::msgs::GUICamera request;
  gz::msgs::Set(request.mutable_pose(), this->centerPose);
  this->cameraBusy = true;
  this->following = false;
  this->followingEntity.clear();
  this->followingLabel.clear();
  this->cameraStatus = QStringLiteral("正在回到地图中心…");
  emit this->CameraStateChanged();

  if (!this->node.Request(this->moveToPoseService.toStdString(), request,
      &TargetControl::OnMapCenterResponse, this))
  {
    this->cameraBusy = false;
    this->cameraStatus = QStringLiteral("地图中心视角服务尚未就绪");
    emit this->CameraStateChanged();
  }
}

void TargetControl::OnMapCenterResponse(
    const gz::msgs::Boolean &_response, bool _result)
{
  const bool success = _result && _response.data();
  QMetaObject::invokeMethod(this, [this, success]
  {
    this->cameraBusy = false;
    this->cameraStatus = success ? QStringLiteral("已回到地图中心") :
        QStringLiteral("视角尚未就绪，请稍后重试");
    emit this->CameraStateChanged();
  }, Qt::QueuedConnection);
}

void TargetControl::PublishRadar(const QString &_json)
{
  if (!this->radarPublisher.Valid())
  {
    this->radarResponseTimer.stop();
    this->radarStatus = QStringLiteral("雷达控制服务尚未就绪");
    this->radarBusy = false;
    emit this->RadarStateChanged();
    return;
  }
  gz::msgs::StringMsg message;
  message.set_data(_json.toStdString());
  if (!this->radarPublisher.Publish(message))
  {
    this->radarResponseTimer.stop();
    this->radarStatus = QStringLiteral("发送雷达控制命令失败");
    this->radarBusy = false;
    emit this->RadarStateChanged();
  }
}

void TargetControl::SetStationAddress(const QString &_address)
{
  const QString address = _address.trimmed();
  if (!IsValidStationAddress(address) || address == this->stationAddress)
    return;
  this->stationAddress = address;
  QSettings settings(QSettings::IniFormat, QSettings::UserScope,
      QStringLiteral("PX4"), QStringLiteral("Scenario02TargetControl"));
  settings.setValue(QStringLiteral("stationAddress"), this->stationAddress);
  settings.sync();
  emit this->StationAddressChanged();
}

void TargetControl::Publish(const QString &_json)
{
  if (!this->publisher.Valid())
  {
    this->status = QStringLiteral("靶机控制服务尚未就绪");
    this->busy = false;
    emit this->StateChanged();
    return;
  }
  gz::msgs::StringMsg message;
  message.set_data(_json.toStdString());
  if (!this->publisher.Publish(message))
  {
    this->status = QStringLiteral("发送靶机控制命令失败");
    this->busy = false;
    emit this->StateChanged();
  }
}

void TargetControl::StartTarget(
    const QString &_model,
    double _speed,
    double _altitude,
    double _distance,
    const QString &_direction)
{
  if (this->busy)
    return;

  QJsonObject command{
      {QStringLiteral("command"), QStringLiteral("start")},
      {QStringLiteral("model"), _model},
      {QStringLiteral("speed"), _speed},
      {QStringLiteral("altitude"), _altitude},
      {QStringLiteral("distance"), _distance},
      {QStringLiteral("direction"), _direction},
  };
  this->busy = true;
  this->status = QStringLiteral("正在启动靶机…");
  emit this->StateChanged();
  this->Publish(QString::fromUtf8(
      QJsonDocument(command).toJson(QJsonDocument::Compact)));
}

void TargetControl::StopTarget()
{
  if (this->busy)
    return;

  QJsonObject command{
      {QStringLiteral("command"), QStringLiteral("stop")},
  };
  this->busy = true;
  this->status = QStringLiteral("正在终止靶机…");
  emit this->StateChanged();
  this->Publish(QString::fromUtf8(
      QJsonDocument(command).toJson(QJsonDocument::Compact)));
}

void TargetControl::StartRadar(
    double _rateHz, const QString &_stationAddress)
{
  if (this->radarBusy)
    return;
  this->SetStationAddress(_stationAddress);
  QJsonObject command{
      {QStringLiteral("command"), QStringLiteral("start")},
      {QStringLiteral("rate_hz"), _rateHz},
      {QStringLiteral("station_address"), _stationAddress},
  };
  this->radarBusy = true;
  this->radarStatus = QStringLiteral("正在启动雷达…");
  this->radarResponseTimer.start();
  emit this->RadarStateChanged();
  this->PublishRadar(QString::fromUtf8(
      QJsonDocument(command).toJson(QJsonDocument::Compact)));
}

void TargetControl::StopRadar()
{
  if (this->radarBusy)
    return;
  QJsonObject command{
      {QStringLiteral("command"), QStringLiteral("stop")},
  };
  this->radarBusy = true;
  this->radarStatus = QStringLiteral("正在停止雷达…");
  this->radarResponseTimer.start();
  emit this->RadarStateChanged();
  this->PublishRadar(QString::fromUtf8(
      QJsonDocument(command).toJson(QJsonDocument::Compact)));
}

void TargetControl::OnStatus(const gz::msgs::StringMsg &_message)
{
  const auto document = QJsonDocument::fromJson(
      QByteArray::fromStdString(_message.data()));
  if (!document.isObject())
    return;
  const auto object = document.object();
  const QString state = object.value(QStringLiteral("state")).toString();
  const QString message = object.value(QStringLiteral("message")).toString();
  const QString model = object.value(QStringLiteral("model")).toString();
  QMetaObject::invokeMethod(this, [this, state, message, model]
  {
    this->busy = state == QStringLiteral("starting") ||
        state == QStringLiteral("stopping");
    this->running = state == QStringLiteral("starting") ||
        state == QStringLiteral("running") ||
        state == QStringLiteral("collided") ||
        state == QStringLiteral("stopping");
    const QString previousTargetModel = this->targetModel;
    this->targetModel = this->running ? model : QString{};
    this->status = message.isEmpty() ? QStringLiteral("等待操作") : message;
    emit this->StateChanged();
    if (!this->running && !previousTargetModel.isEmpty() &&
        this->followingEntity == previousTargetModel)
      this->StopFollowing();
  }, Qt::QueuedConnection);
}

void TargetControl::OnRadarStatus(const gz::msgs::StringMsg &_message)
{
  const auto document = QJsonDocument::fromJson(
      QByteArray::fromStdString(_message.data()));
  if (!document.isObject())
    return;
  const auto object = document.object();
  const QString state = object.value(QStringLiteral("state")).toString();
  const QString message = object.value(QStringLiteral("message")).toString();
  const int trackCount = object.value(QStringLiteral("track_count")).toInt();
  QMetaObject::invokeMethod(this, [this, state, message, trackCount]
  {
    this->radarResponseTimer.stop();
    this->radarBusy = state == QStringLiteral("starting") ||
        state == QStringLiteral("stopping");
    this->radarRunning = state == QStringLiteral("starting") ||
        state == QStringLiteral("running") ||
        state == QStringLiteral("stopping");
    this->radarStatus = message.isEmpty() ?
        QStringLiteral("等待操作") : message;
    this->radarTrackCount = trackCount;
    emit this->RadarStateChanged();
  }, Qt::QueuedConnection);
}
}

GZ_ADD_PLUGIN(scenario_02::TargetControl, gz::gui::Plugin)
