#include "TargetControl.hh"

#include <cmath>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMetaObject>
#include <gz/plugin/Register.hh>
#include <tinyxml2.h>

namespace scenario_02
{
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
  const bool subscribed = this->node.Subscribe(
      this->statusTopic.toStdString(), &TargetControl::OnStatus, this);
  const bool radarSubscribed = this->node.Subscribe(
      this->radarStatusTopic.toStdString(), &TargetControl::OnRadarStatus, this);
  if (!this->publisher.Valid() || !subscribed)
    this->status = QStringLiteral("靶机控制通信初始化失败");
  if (!this->radarPublisher.Valid() || !radarSubscribed)
    this->radarStatus = QStringLiteral("雷达控制通信初始化失败");
  emit this->StateChanged();
  emit this->RadarStateChanged();
}

void TargetControl::PublishRadar(const QString &_json)
{
  if (!this->radarPublisher.Valid())
  {
    this->radarStatus = QStringLiteral("雷达控制服务尚未就绪");
    this->radarBusy = false;
    emit this->RadarStateChanged();
    return;
  }
  gz::msgs::StringMsg message;
  message.set_data(_json.toStdString());
  if (!this->radarPublisher.Publish(message))
  {
    this->radarStatus = QStringLiteral("发送雷达控制命令失败");
    this->radarBusy = false;
    emit this->RadarStateChanged();
  }
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
    const QString &_direction,
    const QString &_stationAddress)
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
      {QStringLiteral("station_address"), _stationAddress},
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
  QJsonObject command{
      {QStringLiteral("command"), QStringLiteral("start")},
      {QStringLiteral("rate_hz"), _rateHz},
      {QStringLiteral("station_address"), _stationAddress},
  };
  this->radarBusy = true;
  this->radarStatus = QStringLiteral("正在启动雷达…");
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
  QMetaObject::invokeMethod(this, [this, state, message]
  {
    this->busy = state == QStringLiteral("starting") ||
        state == QStringLiteral("stopping");
    this->running = state == QStringLiteral("starting") ||
        state == QStringLiteral("running") ||
        state == QStringLiteral("collided") ||
        state == QStringLiteral("stopping");
    this->status = message.isEmpty() ? QStringLiteral("等待操作") : message;
    emit this->StateChanged();
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
