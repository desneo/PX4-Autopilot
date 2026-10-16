#pragma once

#include <QString>
#include <gz/gui/Plugin.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>

namespace scenario_02
{
class TargetControl final : public gz::gui::Plugin
{
  Q_OBJECT
  Q_PROPERTY(bool busy READ Busy NOTIFY StateChanged)
  Q_PROPERTY(bool running READ Running NOTIFY StateChanged)
  Q_PROPERTY(QString status READ Status NOTIFY StateChanged)
  Q_PROPERTY(double defaultAltitudeM READ DefaultAltitudeM NOTIFY StateChanged)

  public: TargetControl() = default;
  public: ~TargetControl() override = default;

  public: Q_INVOKABLE void StartTarget(
      const QString &_model,
      double _speed,
      double _altitude,
      double _distance,
      const QString &_direction,
      const QString &_stationAddress);
  public: Q_INVOKABLE void StopTarget();

  public: bool Busy() const { return this->busy; }
  public: bool Running() const { return this->running; }
  public: QString Status() const { return this->status; }
  public: double DefaultAltitudeM() const { return this->defaultAltitudeM; }

  signals: void StateChanged();

  protected: void LoadConfig(const tinyxml2::XMLElement *_config) override;

  private: void OnStatus(const gz::msgs::StringMsg &_message);
  private: void Publish(const QString &_json);

  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher publisher;
  private: QString commandTopic{QStringLiteral("/scenario/02/target/command")};
  private: QString statusTopic{QStringLiteral("/scenario/02/target/status")};
  private: bool busy{false};
  private: bool running{false};
  private: QString status{QStringLiteral("等待操作")};
  private: double defaultAltitudeM{100.0};
};
}
