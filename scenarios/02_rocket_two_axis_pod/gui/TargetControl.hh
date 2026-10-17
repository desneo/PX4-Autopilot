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
  Q_PROPERTY(bool radarBusy READ RadarBusy NOTIFY RadarStateChanged)
  Q_PROPERTY(bool radarRunning READ RadarRunning NOTIFY RadarStateChanged)
  Q_PROPERTY(QString radarStatus READ RadarStatus NOTIFY RadarStateChanged)
  Q_PROPERTY(int radarTrackCount READ RadarTrackCount NOTIFY RadarStateChanged)

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
  public: Q_INVOKABLE void StartRadar(
      double _rateHz, const QString &_stationAddress);
  public: Q_INVOKABLE void StopRadar();

  public: bool Busy() const { return this->busy; }
  public: bool Running() const { return this->running; }
  public: QString Status() const { return this->status; }
  public: double DefaultAltitudeM() const { return this->defaultAltitudeM; }
  public: bool RadarBusy() const { return this->radarBusy; }
  public: bool RadarRunning() const { return this->radarRunning; }
  public: QString RadarStatus() const { return this->radarStatus; }
  public: int RadarTrackCount() const { return this->radarTrackCount; }

  signals: void StateChanged();
  signals: void RadarStateChanged();

  protected: void LoadConfig(const tinyxml2::XMLElement *_config) override;

  private: void OnStatus(const gz::msgs::StringMsg &_message);
  private: void OnRadarStatus(const gz::msgs::StringMsg &_message);
  private: void Publish(const QString &_json);
  private: void PublishRadar(const QString &_json);

  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher publisher;
  private: gz::transport::Node::Publisher radarPublisher;
  private: QString commandTopic{QStringLiteral("/scenario/02/target/command")};
  private: QString statusTopic{QStringLiteral("/scenario/02/target/status")};
  private: QString radarCommandTopic{
      QStringLiteral("/scenario/02/radar/command")};
  private: QString radarStatusTopic{
      QStringLiteral("/scenario/02/radar/status")};
  private: bool busy{false};
  private: bool running{false};
  private: QString status{QStringLiteral("等待操作")};
  private: double defaultAltitudeM{100.0};
  private: bool radarBusy{false};
  private: bool radarRunning{false};
  private: QString radarStatus{QStringLiteral("等待操作")};
  private: int radarTrackCount{0};
};
}
