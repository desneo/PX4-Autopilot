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
    this->title = "靶机控制";

  if (_config)
  {
    if (const auto *value = _config->FirstChildElement("command_topic");
        value && value->GetText())
      this->commandTopic = QString::fromUtf8(value->GetText());
    if (const auto *value = _config->FirstChildElement("status_topic");
        value && value->GetText())
      this->statusTopic = QString::fromUtf8(value->GetText());
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
  const bool subscribed = this->node.Subscribe(
      this->statusTopic.toStdString(), &TargetControl::OnStatus, this);
  if (!this->publisher.Valid() || !subscribed)
    this->status = QStringLiteral("靶机控制通信初始化失败");
  emit this->StateChanged();
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
}

GZ_ADD_PLUGIN(scenario_02::TargetControl, gz::gui::Plugin)
