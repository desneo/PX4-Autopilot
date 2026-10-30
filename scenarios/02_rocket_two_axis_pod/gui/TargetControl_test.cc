#include "TargetControl.hh"

#include <QJsonDocument>
#include <QJsonObject>
#include <QQuickItem>
#include <QSettings>
#include <QtTest/QtTest>
#include <cmath>
#include <functional>
#include <gz/common/MouseEvent.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/cameratrack.pb.h>
#include <gz/msgs/gui_camera.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>
#include <tinyxml2.h>

class TargetControlTest : public QObject
{
  Q_OBJECT

  private slots: void PublishesStartAndReceivesStatus();
};

void TargetControlTest::PublishesStartAndReceivesStatus()
{
  QSettings settings(QSettings::IniFormat, QSettings::UserScope,
      QStringLiteral("PX4"), QStringLiteral("Scenario02TargetControl"));
  settings.clear();
  settings.sync();

  tinyxml2::XMLDocument config;
  QVERIFY(config.Parse(R"(
    <plugin filename="TargetControl">
      <command_topic>/test/target/command</command_topic>
      <status_topic>/test/target/status</status_topic>
      <radar_command_topic>/test/radar/command</radar_command_topic>
      <radar_status_topic>/test/radar/status</radar_status_topic>
      <follow_service>/test/gui/follow</follow_service>
      <track_topic>/test/gui/track</track_topic>
      <move_to_pose_service>/test/gui/move_to/pose</move_to_pose_service>
      <center_pose>0 0 200 0 1.5707963267948966 1.5707963267948966</center_pose>
      <aircraft_count>2</aircraft_count>
      <default_altitude_m>104.019969</default_altitude_m>
    </plugin>)") == tinyxml2::XML_SUCCESS);

  gz::transport::Node node;
  QString command;
  QString radarCommand;
  QString followedEntity;
  gz::msgs::CameraTrack trackMessage;
  bool returnedToCenter = false;
  QVERIFY(node.Subscribe<gz::msgs::StringMsg>("/test/target/command",
      std::function<void(const gz::msgs::StringMsg &)>([&command](
          const gz::msgs::StringMsg &_message)
      {
        command = QString::fromStdString(_message.data());
      })));
  QVERIFY(node.Subscribe<gz::msgs::StringMsg>("/test/radar/command",
      std::function<void(const gz::msgs::StringMsg &)>([&radarCommand](
          const gz::msgs::StringMsg &_message)
      {
        radarCommand = QString::fromStdString(_message.data());
      })));
  QVERIFY(node.Subscribe<gz::msgs::CameraTrack>("/test/gui/track",
      std::function<void(const gz::msgs::CameraTrack &)>([&trackMessage](
          const gz::msgs::CameraTrack &_message)
      {
        trackMessage = _message;
      })));
  std::function<bool(const gz::msgs::StringMsg &, gz::msgs::Boolean &)>
      followHandler = [&followedEntity](
      const gz::msgs::StringMsg &_request, gz::msgs::Boolean &_response)
  {
    followedEntity = QString::fromStdString(_request.data());
    _response.set_data(true);
    return true;
  };
  QVERIFY(node.Advertise("/test/gui/follow", followHandler));
  std::function<bool(const gz::msgs::GUICamera &, gz::msgs::Boolean &)>
      centerHandler = [&returnedToCenter](
      const gz::msgs::GUICamera &_request, gz::msgs::Boolean &_response)
  {
    returnedToCenter = _request.has_pose() &&
        qFuzzyCompare(_request.pose().position().z(), 200.0);
    _response.set_data(returnedToCenter);
    return true;
  };
  QVERIFY(node.Advertise("/test/gui/move_to/pose", centerHandler));

  scenario_02::TargetControl plugin;
  plugin.Load(config.RootElement());
  QVERIFY(plugin.PluginItem());
  QCOMPARE(plugin.DefaultAltitudeM(), 104.019969);
  QCOMPARE(plugin.AircraftCount(), 2);
  QCOMPARE(plugin.StationAddress(), QStringLiteral("192.168.1.41:7790"));
  plugin.SetStationAddress(QStringLiteral("192.168.1.100:7790"));
  scenario_02::TargetControl restoredSettings;
  QCOMPARE(restoredSettings.StationAddress(),
      QStringLiteral("192.168.1.100:7790"));
  QVERIFY(plugin.PluginItem()->findChild<QQuickItem *>("returnToMapCenterButton"));
  QVERIFY(plugin.PluginItem()->findChild<QQuickItem *>("fixedFollowTitleBar"));
  auto *aircraftCombo = plugin.PluginItem()->findChild<QQuickItem *>(
      "aircraftFollowCombo");
  QVERIFY(aircraftCombo);
  QTRY_COMPARE_WITH_TIMEOUT(aircraftCombo->property("count").toInt(), 3, 1000);
  QVERIFY(plugin.PluginItem()->findChild<QQuickItem *>("stopFollowingButton"));
  QVERIFY(plugin.PluginItem()->findChild<QQuickItem *>("fixedTargetTitleBar"));
  QVERIFY(plugin.PluginItem()->findChild<QQuickItem *>("fixedRadarTitleBar"));
  auto *collapseButton = plugin.PluginItem()->findChild<QQuickItem *>(
      "collapseTargetPanelButton");
  QVERIFY(collapseButton);
  auto *radarCollapseButton = plugin.PluginItem()->findChild<QQuickItem *>(
      "collapseRadarPanelButton");
  QVERIFY(radarCollapseButton);
  auto *targetModel = plugin.PluginItem()->findChild<QQuickItem *>(
      "targetModelCombo");
  auto *speedRange = plugin.PluginItem()->findChild<QQuickItem *>(
      "speedRangeLabel");
  auto *distanceRange = plugin.PluginItem()->findChild<QQuickItem *>(
      "distanceRangeLabel");
  QVERIFY(targetModel);
  QVERIFY(speedRange);
  QVERIFY(distanceRange);
  QCOMPARE(distanceRange->property("text").toString(),
      QStringLiteral("100～10000 m"));
  QVERIFY(plugin.PluginItem()->findChild<QQuickItem *>("startTargetButton"));
  QVERIFY(plugin.PluginItem()->findChild<QQuickItem *>("stopTargetButton"));
  QVERIFY(plugin.PluginItem()->findChild<QQuickItem *>("startRadarButton"));
  QVERIFY(plugin.PluginItem()->findChild<QQuickItem *>("stopRadarButton"));
  auto *radarRateRange = plugin.PluginItem()->findChild<QQuickItem *>(
      "radarRateRangeLabel");
  auto *stationField = plugin.PluginItem()->findChild<QQuickItem *>(
      "stationAddressField");
  QVERIFY(radarRateRange);
  QVERIFY(stationField);
  QCOMPARE(radarRateRange->property("text").toString(),
      QStringLiteral("1.0～2.0 Hz"));

  plugin.FollowEntity("rocket_two_axis_pod_2", "飞机 2 · PX4");
  QTRY_COMPARE_WITH_TIMEOUT(
      followedEntity, QStringLiteral("rocket_two_axis_pod_2"), 2000);
  QTRY_VERIFY_WITH_TIMEOUT(plugin.Following(), 2000);
  QTRY_COMPARE_WITH_TIMEOUT(trackMessage.track_mode(),
      gz::msgs::CameraTrack::FOLLOW, 2000);
  QCOMPARE(QString::fromStdString(trackMessage.follow_target().name()),
      QStringLiteral("rocket_two_axis_pod_2"));
  QCOMPARE(trackMessage.follow_pgain(), 1.0);
  QCOMPARE(trackMessage.track_pgain(), 1.0);
  QCOMPARE(trackMessage.follow_offset().x(), -12.0);
  QCOMPARE(trackMessage.follow_offset().z(), 6.0);
  QCOMPARE(plugin.FollowingEntity(), QStringLiteral("rocket_two_axis_pod_2"));
  QCOMPARE(plugin.FollowingLabel(), QStringLiteral("飞机 2 · PX4"));

  const double initialRadius = std::hypot(
      std::hypot(trackMessage.follow_offset().x(),
                 trackMessage.follow_offset().y()),
      trackMessage.follow_offset().z());
  gz::common::MouseEvent scrollMouse;
  scrollMouse.SetType(gz::common::MouseEvent::SCROLL);
  scrollMouse.SetScroll(0, -1);
  gz::gui::events::ScrollOnScene scrollEvent(scrollMouse);
  QVERIFY(!plugin.eventFilter(nullptr, &scrollEvent));
  QTRY_VERIFY_WITH_TIMEOUT(std::hypot(
      std::hypot(trackMessage.follow_offset().x(),
                 trackMessage.follow_offset().y()),
      trackMessage.follow_offset().z()) < initialRadius, 2000);
  QCOMPARE(trackMessage.track_mode(), gz::msgs::CameraTrack::FOLLOW);
  QCOMPARE(QString::fromStdString(trackMessage.follow_target().name()),
      QStringLiteral("rocket_two_axis_pod_2"));

  for (int index = 0; index < 30; ++index)
    QVERIFY(!plugin.eventFilter(nullptr, &scrollEvent));
  QTRY_VERIFY_WITH_TIMEOUT(std::hypot(
      std::hypot(trackMessage.follow_offset().x(),
                 trackMessage.follow_offset().y()),
      trackMessage.follow_offset().z()) < 0.5, 2000);
  const double closestRadius = std::hypot(
      std::hypot(trackMessage.follow_offset().x(),
                 trackMessage.follow_offset().y()),
      trackMessage.follow_offset().z());
  QVERIFY(closestRadius >= 0.249);

  gz::common::MouseEvent pressMouse;
  pressMouse.SetType(gz::common::MouseEvent::PRESS);
  pressMouse.SetButton(gz::common::MouseEvent::MIDDLE);
  pressMouse.SetButtons(gz::common::MouseEvent::MIDDLE);
  pressMouse.SetPos(100, 100);
  gz::gui::events::MousePressOnScene pressEvent(pressMouse);
  QVERIFY(!plugin.eventFilter(nullptr, &pressEvent));
  gz::common::MouseEvent dragMouse;
  dragMouse.SetType(gz::common::MouseEvent::MOVE);
  dragMouse.SetButtons(gz::common::MouseEvent::MIDDLE);
  dragMouse.SetPrevPos(100, 100);
  dragMouse.SetPos(140, 120);
  gz::gui::events::DragOnScene dragEvent(dragMouse);
  const double beforeOrbitY = trackMessage.follow_offset().y();
  QVERIFY(!plugin.eventFilter(nullptr, &dragEvent));
  QTRY_VERIFY_WITH_TIMEOUT(
      std::abs(trackMessage.follow_offset().y() - beforeOrbitY) > 0.01, 2000);
  QCOMPARE(trackMessage.track_mode(), gz::msgs::CameraTrack::FOLLOW);
  QCOMPARE(QString::fromStdString(trackMessage.follow_target().name()),
      QStringLiteral("rocket_two_axis_pod_2"));

  plugin.StopFollowing();
  QVERIFY(!plugin.Following());
  plugin.ReturnToMapCenter();
  QTRY_VERIFY_WITH_TIMEOUT(returnedToCenter, 2000);
  QTRY_VERIFY_WITH_TIMEOUT(!plugin.CameraBusy(), 2000);

  targetModel->setProperty("currentIndex", 1);
  QTRY_COMPARE(speedRange->property("text").toString(),
      QStringLiteral("25～65 m/s"));
  targetModel->setProperty("currentIndex", 2);
  QTRY_COMPARE(speedRange->property("text").toString(),
      QStringLiteral("10～30 m/s"));

  QVERIFY(plugin.PluginItem()->property("targetCollapsed").toBool());
  QVERIFY(QMetaObject::invokeMethod(collapseButton, "clicked"));
  QTRY_VERIFY_WITH_TIMEOUT(
      !plugin.PluginItem()->property("targetCollapsed").toBool(), 1000);
  QVERIFY(QMetaObject::invokeMethod(collapseButton, "clicked"));
  QTRY_VERIFY_WITH_TIMEOUT(
      plugin.PluginItem()->property("targetCollapsed").toBool(), 1000);
  QVERIFY(plugin.PluginItem()->property("radarCollapsed").toBool());
  QVERIFY(QMetaObject::invokeMethod(radarCollapseButton, "clicked"));
  QTRY_VERIFY_WITH_TIMEOUT(
      !plugin.PluginItem()->property("radarCollapsed").toBool(), 1000);

  plugin.StartTarget(
      "xiaomotuo", 35, 104.019969, 2000, "east_west");
  QTRY_VERIFY_WITH_TIMEOUT(!command.isEmpty(), 2000);
  const auto object = QJsonDocument::fromJson(command.toUtf8()).object();
  QCOMPARE(object.value("command").toString(), QString("start"));
  QCOMPARE(object.value("model").toString(), QString("xiaomotuo"));
  QCOMPARE(object.value("speed").toDouble(), 35.0);
  QVERIFY(!object.contains(QStringLiteral("station_address")));

  plugin.StartRadar(1.5, "192.168.1.41:7790");
  QTRY_VERIFY_WITH_TIMEOUT(!radarCommand.isEmpty(), 2000);
  const auto radarObject = QJsonDocument::fromJson(
      radarCommand.toUtf8()).object();
  QCOMPARE(radarObject.value("command").toString(), QString("start"));
  QCOMPARE(radarObject.value("rate_hz").toDouble(), 1.5);
  QCOMPARE(radarObject.value("station_address").toString(),
      QString("192.168.1.41:7790"));

  auto statusPublisher = node.Advertise<gz::msgs::StringMsg>(
      "/test/target/status");
  QVERIFY(statusPublisher.Valid());
  gz::msgs::StringMsg status;
  status.set_data(
      R"({"state":"running","message":"靶机飞行中","model":"xiaomotuo"})");
  for (int attempt = 0; attempt < 20 && !plugin.Running(); ++attempt)
  {
    statusPublisher.Publish(status);
    QTest::qWait(50);
  }
  QTRY_VERIFY_WITH_TIMEOUT(plugin.Running(), 2000);
  QCOMPARE(plugin.Status(), QStringLiteral("靶机飞行中"));
  QCOMPARE(plugin.TargetModel(), QStringLiteral("xiaomotuo"));
  QTRY_COMPARE_WITH_TIMEOUT(aircraftCombo->property("count").toInt(), 4, 1000);

  status.set_data(R"({"state":"collided","message":"靶机已碰撞"})");
  statusPublisher.Publish(status);
  QTRY_COMPARE_WITH_TIMEOUT(
      plugin.Status(), QStringLiteral("靶机已碰撞"), 2000);
  QVERIFY(plugin.Running());

  auto radarStatusPublisher = node.Advertise<gz::msgs::StringMsg>(
      "/test/radar/status");
  QVERIFY(radarStatusPublisher.Valid());
  status.set_data(
      R"({"state":"running","message":"雷达运行中，当前航迹 2 条","track_count":2})");
  for (int attempt = 0; attempt < 20 && !plugin.RadarRunning(); ++attempt)
  {
    radarStatusPublisher.Publish(status);
    QTest::qWait(50);
  }
  QTRY_VERIFY_WITH_TIMEOUT(plugin.RadarRunning(), 2000);
  QCOMPARE(plugin.RadarTrackCount(), 2);
  QVERIFY(!stationField->property("enabled").toBool());

  plugin.radarResponseTimer.setInterval(20);
  plugin.StopRadar();
  QTRY_VERIFY_WITH_TIMEOUT(!plugin.RadarBusy(), 1000);
  QVERIFY(plugin.RadarStatus().contains(QStringLiteral("响应超时")));

  status.set_data(
      R"({"state":"idle","message":"雷达已停止","track_count":0})");
  radarStatusPublisher.Publish(status);
  QTRY_VERIFY_WITH_TIMEOUT(!plugin.RadarRunning(), 2000);
  QTRY_VERIFY_WITH_TIMEOUT(stationField->property("enabled").toBool(), 2000);
  settings.clear();
  settings.sync();
}

int main(int argc, char **argv)
{
  gz::gui::Application app(argc, argv, gz::gui::WindowType::kDialog);
  TargetControlTest test;
  return QTest::qExec(&test, argc, argv);
}

#include "TargetControl_test.moc"
