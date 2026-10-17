import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

Item {
  id: root
  anchors.fill: parent
  Layout.minimumWidth: 350
  Layout.minimumHeight: 700
  property bool radarCollapsed: false
  property bool targetCollapsed: false

  function selectedModel() {
    return targetModel.currentIndex === 1 ? "xiaomotuo" :
           targetModel.currentIndex === 2 ? "xiaobianfu" : ""
  }
  function speedMinimum() { return targetModel.currentIndex === 1 ? 25 : 10 }
  function speedMaximum() { return targetModel.currentIndex === 1 ? 65 : 30 }
  function stationValid(value) {
    var separator = value.lastIndexOf(":")
    if (separator <= 0 || separator === value.length - 1)
      return false
    var port = Number(value.substring(separator + 1))
    return isFinite(port) && port >= 1 && port <= 65535
  }
  function validateRadar() {
    var rate = Number(radarRateField.text)
    if (!isFinite(rate) || rate < 1.0 || rate > 2.0)
      return "雷达航迹频率必须在 1.0～2.0 Hz"
    if (!root.stationValid(stationField.text.trim()))
      return "地面站地址格式应为 主机:端口"
    return ""
  }
  function validateTarget() {
    if (targetModel.currentIndex === 0)
      return "请先选择小摩托或小蝙蝠"
    var speed = Number(speedField.text)
    if (!isFinite(speed) || speed < root.speedMinimum() || speed > root.speedMaximum())
      return "巡航速度必须在 " + root.speedMinimum() + "～" + root.speedMaximum() + " m/s"
    var altitude = Number(altitudeField.text)
    if (!isFinite(altitude))
      return "海拔高度必须是有效数值"
    var distance = Number(distanceField.text)
    if (!isFinite(distance) || distance < 100 || distance > 10000)
      return "来源距离必须在 100～10000 米"
    if (!root.stationValid(stationField.text.trim()))
      return "请在雷达控制中填写有效的地面站地址"
    return ""
  }

  Flickable {
    anchors.fill: parent
    clip: true
    contentWidth: width
    contentHeight: panels.height
    boundsBehavior: Flickable.StopAtBounds

    Column {
      id: panels
      width: parent.width
      spacing: 4

      Rectangle {
        id: radarPanel
        width: parent.width
        height: root.radarCollapsed ? 50 : 310
        color: "#e8ffffff"

        Rectangle {
          id: radarTitleBar
          objectName: "fixedRadarTitleBar"
          anchors.left: parent.left
          anchors.right: parent.right
          anchors.top: parent.top
          height: 50
          color: "#03a9f4"
          Label {
            anchors.left: parent.left
            anchors.leftMargin: 12
            anchors.verticalCenter: parent.verticalCenter
            text: "雷达控制"
            color: "white"
            font.pixelSize: 16
          }
          ToolButton {
            id: radarCollapseButton
            objectName: "collapseRadarPanelButton"
            anchors.right: parent.right
            anchors.top: parent.top
            anchors.bottom: parent.bottom
            width: 50
            text: root.radarCollapsed ? "\u25BE" : "\u25B4"
            contentItem: Text {
              text: radarCollapseButton.text
              color: "white"
              horizontalAlignment: Text.AlignHCenter
              verticalAlignment: Text.AlignVCenter
            }
            background: Item {}
            onClicked: root.radarCollapsed = !root.radarCollapsed
            ToolTip.visible: hovered
            ToolTip.text: root.radarCollapsed ? "展开雷达控制" : "折叠雷达控制"
          }
        }

        ColumnLayout {
          anchors.left: parent.left
          anchors.right: parent.right
          anchors.top: radarTitleBar.bottom
          anchors.bottom: parent.bottom
          anchors.margins: 10
          spacing: 6
          visible: !root.radarCollapsed

          GridLayout {
            columns: 2
            columnSpacing: 8
            rowSpacing: 6
            Layout.fillWidth: true
            Label { text: "航迹频率（Hz）"; color: "#202124" }
            TextField {
              id: radarRateField
              objectName: "radarRateField"
              Layout.fillWidth: true
              text: "1.0"
              selectByMouse: true
              enabled: !TargetControl.radarRunning && !TargetControl.radarBusy
              validator: DoubleValidator { bottom: 1.0; top: 2.0; decimals: 2 }
            }
            Label { text: "频率范围"; color: "#202124" }
            Label {
              objectName: "radarRateRangeLabel"
              text: "1.0～2.0 Hz"
              color: "#607d8b"
            }
            Label { text: "地面站地址\n（雷达/靶机真值）"; color: "#202124" }
            TextField {
              id: stationField
              objectName: "stationAddressField"
              Layout.fillWidth: true
              text: "192.168.1.41:7790"
              selectByMouse: true
              enabled: !TargetControl.radarRunning && !TargetControl.running
            }
          }
          RowLayout {
            Layout.fillWidth: true
            spacing: 8
            Button {
              objectName: "startRadarButton"
              Layout.fillWidth: true
              text: "启动雷达"
              enabled: !TargetControl.radarBusy
              onClicked: {
                var error = root.validateRadar()
                radarError.text = error
                if (error.length === 0)
                  TargetControl.StartRadar(Number(radarRateField.text), stationField.text.trim())
              }
            }
            Button {
              objectName: "stopRadarButton"
              Layout.fillWidth: true
              text: "停止雷达"
              enabled: TargetControl.radarRunning && !TargetControl.radarBusy
              onClicked: TargetControl.StopRadar()
            }
          }
          Label {
            id: radarError
            objectName: "radarValidationMessage"
            Layout.fillWidth: true
            visible: text.length > 0
            color: "#d32f2f"
            wrapMode: Text.WordWrap
            font.pixelSize: 12
          }
          Label {
            objectName: "radarStatusLabel"
            Layout.fillWidth: true
            text: TargetControl.radarStatus
            color: "#202124"
            wrapMode: Text.WordWrap
            font.pixelSize: 12
          }
          Item { Layout.fillHeight: true }
        }
      }

      Rectangle {
        id: targetPanel
        width: parent.width
        height: root.targetCollapsed ? 50 : (targetModel.currentIndex > 0 ? 500 : 175)
        color: "#e8ffffff"

        Rectangle {
          id: targetTitleBar
          objectName: "fixedTargetTitleBar"
          anchors.left: parent.left
          anchors.right: parent.right
          anchors.top: parent.top
          height: 50
          color: "#03a9f4"
          Label {
            anchors.left: parent.left
            anchors.leftMargin: 12
            anchors.verticalCenter: parent.verticalCenter
            text: "靶机控制"
            color: "white"
            font.pixelSize: 16
          }
          ToolButton {
            id: targetCollapseButton
            objectName: "collapseTargetPanelButton"
            anchors.right: parent.right
            anchors.top: parent.top
            anchors.bottom: parent.bottom
            width: 50
            text: root.targetCollapsed ? "\u25BE" : "\u25B4"
            contentItem: Text {
              text: targetCollapseButton.text
              color: "white"
              horizontalAlignment: Text.AlignHCenter
              verticalAlignment: Text.AlignVCenter
            }
            background: Item {}
            onClicked: root.targetCollapsed = !root.targetCollapsed
            ToolTip.visible: hovered
            ToolTip.text: root.targetCollapsed ? "展开靶机控制" : "折叠靶机控制"
          }
        }

        ColumnLayout {
          anchors.left: parent.left
          anchors.right: parent.right
          anchors.top: targetTitleBar.bottom
          anchors.bottom: parent.bottom
          anchors.margins: 10
          spacing: 6
          visible: !root.targetCollapsed

          Label { text: "启用靶机"; color: "#202124"; font.bold: true }
          ComboBox {
            id: targetModel
            objectName: "targetModelCombo"
            Layout.fillWidth: true
            model: ["不启用", "小摩托", "小蝙蝠"]
            enabled: !TargetControl.busy && !TargetControl.running
            onCurrentIndexChanged: {
              if (currentIndex === 1) speedField.text = "35"
              else if (currentIndex === 2) speedField.text = "18"
              targetError.text = ""
            }
          }
          GridLayout {
            visible: targetModel.currentIndex > 0
            columns: 2
            columnSpacing: 8
            rowSpacing: 5
            Layout.fillWidth: true
            Label { text: "巡航速度（m/s）"; color: "#202124" }
            TextField {
              id: speedField; objectName: "speedField"; Layout.fillWidth: true
              text: "35"; selectByMouse: true
              validator: DoubleValidator { bottom: 0; top: 1000; decimals: 2 }
            }
            Label { text: "速度范围"; color: "#202124" }
            Label {
              objectName: "speedRangeLabel"
              text: root.speedMinimum() + "～" + root.speedMaximum() + " m/s"
              color: "#607d8b"
            }
            Label { text: "海拔高度（m）"; color: "#202124" }
            TextField {
              id: altitudeField; objectName: "altitudeField"; Layout.fillWidth: true
              text: TargetControl.defaultAltitudeM.toFixed(2); selectByMouse: true
              validator: DoubleValidator { bottom: -500; top: 10000; decimals: 3 }
            }
            Label { text: "来源距离（m）"; color: "#202124" }
            TextField {
              id: distanceField; objectName: "distanceField"; Layout.fillWidth: true
              text: "2000"; selectByMouse: true
              validator: IntValidator { bottom: 100; top: 10000 }
            }
            Label { text: "距离范围"; color: "#202124" }
            Label { objectName: "distanceRangeLabel"; text: "100～10000 m"; color: "#607d8b" }
            Label { text: "来源方向"; color: "#202124" }
            ComboBox {
              id: directionBox; objectName: "directionCombo"; Layout.fillWidth: true
              model: ["从东向西", "从东北45度"]
            }
          }
          RowLayout {
            visible: targetModel.currentIndex > 0
            Layout.fillWidth: true
            spacing: 8
            Button {
              objectName: "startTargetButton"
              Layout.fillWidth: true
              text: "启动靶机"
              enabled: !TargetControl.busy
              onClicked: {
                var error = root.validateTarget()
                targetError.text = error
                if (error.length === 0)
                  TargetControl.StartTarget(
                    root.selectedModel(), Number(speedField.text), Number(altitudeField.text),
                    Number(distanceField.text),
                    directionBox.currentIndex === 0 ? "east_west" : "northeast_45",
                    stationField.text.trim())
              }
            }
            Button {
              objectName: "stopTargetButton"
              Layout.fillWidth: true
              text: "终止靶机"
              enabled: TargetControl.running && !TargetControl.busy
              onClicked: TargetControl.StopTarget()
            }
          }
          Label {
            id: targetError
            objectName: "validationMessage"
            Layout.fillWidth: true
            visible: text.length > 0
            color: "#d32f2f"
            wrapMode: Text.WordWrap
            font.pixelSize: 12
          }
          Label {
            objectName: "targetStatusLabel"
            Layout.fillWidth: true
            visible: targetModel.currentIndex > 0
            text: TargetControl.status
            color: "#202124"
            wrapMode: Text.WordWrap
            font.pixelSize: 12
          }
          Item { Layout.fillHeight: true }
        }
      }
    }
    ScrollBar.vertical: ScrollBar {}
  }
}
