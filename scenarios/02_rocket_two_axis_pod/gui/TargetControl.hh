#pragma once

#include <QString>
#include <gz/gui/Plugin.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/cameratrack.pb.h>
#include <gz/msgs/gui_camera.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>

class TargetControlTest;

namespace scenario_02
{
class TargetControl final : public gz::gui::Plugin
{
  Q_OBJECT
  friend class ::TargetControlTest;
  Q_PROPERTY(bool busy READ Busy NOTIFY StateChanged)
  Q_PROPERTY(bool running READ Running NOTIFY StateChanged)
  Q_PROPERTY(QString status READ Status NOTIFY StateChanged)
  Q_PROPERTY(double defaultAltitudeM READ DefaultAltitudeM NOTIFY StateChanged)
  Q_PROPERTY(bool radarBusy READ RadarBusy NOTIFY RadarStateChanged)
  Q_PROPERTY(bool radarRunning READ RadarRunning NOTIFY RadarStateChanged)
  Q_PROPERTY(QString radarStatus READ RadarStatus NOTIFY RadarStateChanged)
  Q_PROPERTY(int radarTrackCount READ RadarTrackCount NOTIFY RadarStateChanged)
  Q_PROPERTY(int aircraftCount READ AircraftCount NOTIFY CameraStateChanged)
  Q_PROPERTY(QString targetModel READ TargetModel NOTIFY StateChanged)
  Q_PROPERTY(bool cameraBusy READ CameraBusy NOTIFY CameraStateChanged)
  Q_PROPERTY(bool following READ Following NOTIFY CameraStateChanged)
  Q_PROPERTY(QString followingEntity READ FollowingEntity NOTIFY CameraStateChanged)
  Q_PROPERTY(QString followingLabel READ FollowingLabel NOTIFY CameraStateChanged)
  Q_PROPERTY(QString cameraStatus READ CameraStatus NOTIFY CameraStateChanged)

  public: TargetControl();
  public: ~TargetControl() override;

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
  public: Q_INVOKABLE void FollowEntity(
      const QString &_entity, const QString &_label);
  public: Q_INVOKABLE void StopFollowing();
  public: Q_INVOKABLE void ReturnToMapCenter();

  public: bool Busy() const { return this->busy; }
  public: bool Running() const { return this->running; }
  public: QString Status() const { return this->status; }
  public: double DefaultAltitudeM() const { return this->defaultAltitudeM; }
  public: bool RadarBusy() const { return this->radarBusy; }
  public: bool RadarRunning() const { return this->radarRunning; }
  public: QString RadarStatus() const { return this->radarStatus; }
  public: int RadarTrackCount() const { return this->radarTrackCount; }
  public: int AircraftCount() const { return this->aircraftCount; }
  public: QString TargetModel() const { return this->targetModel; }
  public: bool CameraBusy() const { return this->cameraBusy; }
  public: bool Following() const { return this->following; }
  public: QString FollowingEntity() const { return this->followingEntity; }
  public: QString FollowingLabel() const { return this->followingLabel; }
  public: QString CameraStatus() const { return this->cameraStatus; }

  signals: void StateChanged();
  signals: void RadarStateChanged();
  signals: void CameraStateChanged();

  protected: void LoadConfig(const tinyxml2::XMLElement *_config) override;
  protected: bool eventFilter(QObject *_object, QEvent *_event) override;

  private: void OnStatus(const gz::msgs::StringMsg &_message);
  private: void OnRadarStatus(const gz::msgs::StringMsg &_message);
  private: void OnFollowResponse(
      const gz::msgs::Boolean &_response, bool _result);
  private: void OnMapCenterResponse(
      const gz::msgs::Boolean &_response, bool _result);
  private: void ClearCameraTracking();
  private: void SetNativeCameraControlBlocked(bool _blocked);
  private: void OrbitFollow(const gz::math::Vector2i &_drag);
  private: void ZoomFollow(int _scrollSteps);
  private: void PublishFollow(const QString &_entity);
  private: void Publish(const QString &_json);
  private: void PublishRadar(const QString &_json);

  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher publisher;
  private: gz::transport::Node::Publisher radarPublisher;
  private: gz::transport::Node::Publisher trackingPublisher;
  private: QString commandTopic{QStringLiteral("/scenario/02/target/command")};
  private: QString statusTopic{QStringLiteral("/scenario/02/target/status")};
  private: QString radarCommandTopic{
      QStringLiteral("/scenario/02/radar/command")};
  private: QString radarStatusTopic{
      QStringLiteral("/scenario/02/radar/status")};
  private: QString followService{QStringLiteral("/gui/follow")};
  private: QString moveToPoseService{QStringLiteral("/gui/move_to/pose")};
  private: QString trackTopic{QStringLiteral("/gui/track")};
  private: bool busy{false};
  private: bool running{false};
  private: QString status{QStringLiteral("等待操作")};
  private: double defaultAltitudeM{100.0};
  private: bool radarBusy{false};
  private: bool radarRunning{false};
  private: QString radarStatus{QStringLiteral("等待操作")};
  private: int radarTrackCount{0};
  private: int aircraftCount{1};
  private: QString targetModel;
  private: bool cameraBusy{false};
  private: bool following{false};
  private: QString followingEntity;
  private: QString followingLabel;
  private: QString pendingFollowingEntity;
  private: QString pendingFollowingLabel;
  private: QString cameraStatus{QStringLiteral("请选择需要跟随的飞机")};
  private: gz::math::Vector3d followOffset{-12.0, 0.0, 6.0};
  private: gz::math::Vector2i lastMousePosition;
  private: bool followDragActive{false};
  private: gz::math::Pose3d centerPose{
      0, 0, 200, 0, 1.5707963267948966, 1.5707963267948966};
};
}
