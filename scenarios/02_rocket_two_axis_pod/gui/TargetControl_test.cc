#include "TargetControl.hh"

#include <QJsonDocument>
#include <QJsonObject>
#include <QQuickItem>
#include <QtTest/QtTest>
#include <functional>
#include <gz/gui/Application.hh>
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
  tinyxml2::XMLDocument config;
  QVERIFY(config.Parse(R"(
    <plugin filename="TargetControl">
      <command_topic>/test/target/command</command_topic>
      <status_topic>/test/target/status</status_topic>
      <default_altitude_m>104.019969</default_altitude_m>
    </plugin>)") == tinyxml2::XML_SUCCESS);

  gz::transport::Node node;
  QString command;
  QVERIFY(node.Subscribe<gz::msgs::StringMsg>("/test/target/command",
      std::function<void(const gz::msgs::StringMsg &)>([&command](
          const gz::msgs::StringMsg &_message)
      {
        command = QString::fromStdString(_message.data());
      })));

  scenario_02::TargetControl plugin;
  plugin.Load(config.RootElement());
  QVERIFY(plugin.PluginItem());
  QCOMPARE(plugin.DefaultAltitudeM(), 104.019969);
  QVERIFY(plugin.PluginItem()->findChild<QQuickItem *>("fixedTargetTitleBar"));
  auto *collapseButton = plugin.PluginItem()->findChild<QQuickItem *>(
      "collapseTargetPanelButton");
  QVERIFY(collapseButton);
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

  targetModel->setProperty("currentIndex", 1);
  QTRY_COMPARE(speedRange->property("text").toString(),
      QStringLiteral("25～65 m/s"));
  targetModel->setProperty("currentIndex", 2);
  QTRY_COMPARE(speedRange->property("text").toString(),
      QStringLiteral("10～30 m/s"));

  auto *card = plugin.CardItem();
  QVERIFY(card);
  QVERIFY(QMetaObject::invokeMethod(collapseButton, "clicked"));
  QTRY_VERIFY_WITH_TIMEOUT(
      plugin.PluginItem()->property("collapsed").toBool(), 1000);
  QCOMPARE(qRound(card->height()), 50);
  QVERIFY(QMetaObject::invokeMethod(collapseButton, "clicked"));
  QTRY_VERIFY_WITH_TIMEOUT(
      !plugin.PluginItem()->property("collapsed").toBool(), 1000);
  QCOMPARE(qRound(card->height()), 540);

  plugin.StartTarget(
      "xiaomotuo", 35, 104.019969, 2000, "east_west",
      "192.168.1.41:7788");
  QTRY_VERIFY_WITH_TIMEOUT(!command.isEmpty(), 2000);
  const auto object = QJsonDocument::fromJson(command.toUtf8()).object();
  QCOMPARE(object.value("command").toString(), QString("start"));
  QCOMPARE(object.value("model").toString(), QString("xiaomotuo"));
  QCOMPARE(object.value("speed").toDouble(), 35.0);

  auto statusPublisher = node.Advertise<gz::msgs::StringMsg>(
      "/test/target/status");
  QVERIFY(statusPublisher.Valid());
  gz::msgs::StringMsg status;
  status.set_data(R"({"state":"running","message":"靶机飞行中"})");
  for (int attempt = 0; attempt < 20 && !plugin.Running(); ++attempt)
  {
    statusPublisher.Publish(status);
    QTest::qWait(50);
  }
  QTRY_VERIFY_WITH_TIMEOUT(plugin.Running(), 2000);
  QCOMPARE(plugin.Status(), QStringLiteral("靶机飞行中"));

  status.set_data(R"({"state":"collided","message":"靶机已碰撞"})");
  statusPublisher.Publish(status);
  QTRY_COMPARE_WITH_TIMEOUT(
      plugin.Status(), QStringLiteral("靶机已碰撞"), 2000);
  QVERIFY(plugin.Running());
}

int main(int argc, char **argv)
{
  gz::gui::Application app(argc, argv, gz::gui::WindowType::kDialog);
  TargetControlTest test;
  return QTest::qExec(&test, argc, argv);
}

#include "TargetControl_test.moc"
