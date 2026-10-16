import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

Item {
  id: root
  anchors.fill: parent
  Layout.minimumWidth: 350
  Layout.minimumHeight: collapsed ? 50 : 540
  property bool collapsed: false

  function setCollapsed(value) {
    collapsed = value
    if (parent && parent.card)
      parent.card().height = collapsed ? 50 : 540
  }

  function selectedModel() {
    return targetModel.currentIndex === 1 ? "xiaomotuo" :
           targetModel.currentIndex === 2 ? "xiaobianfu" : ""
  }

  function speedMinimum() {
    return targetModel.currentIndex === 1 ? 25 : 10
  }

  function speedMaximum() {
    return targetModel.currentIndex === 1 ? 65 : 30
  }

  function stationValid(value) {
    var separator = value.lastIndexOf(":")
    if (separator <= 0 || separator === value.length - 1)
      return false
    var port = Number(value.substring(separator + 1))
    return isFinite(port) && port >= 1 && port <= 65535
  }

  function validateForm() {
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
      return "地面站地址格式应为 主机:端口"
    return ""
  }

  Rectangle {
    id: fixedTitleBar
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
      id: collapseButton
      objectName: "collapseTargetPanelButton"
      anchors.right: parent.right
      anchors.top: parent.top
      anchors.bottom: parent.bottom
      width: 50
      text: root.collapsed ? "\u25BE" : "\u25B4"
      contentItem: Text {
        text: collapseButton.text
        color: "white"
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
      }
      background: Item {}
      onClicked: root.setCollapsed(!root.collapsed)
      ToolTip.visible: hovered
      ToolTip.text: root.collapsed ? "展开靶机控制" : "折叠靶机控制"
    }
  }

  ColumnLayout {
    anchors.left: parent.left
    anchors.right: parent.right
    anchors.top: fixedTitleBar.bottom
    anchors.bottom: parent.bottom
    anchors.margins: 8
    spacing: 6
    visible: !root.collapsed

  Label {
    text: "启用靶机"
    font.bold: true
  }

  ComboBox {
    id: targetModel
    objectName: "targetModelCombo"
    Layout.fillWidth: true
    model: ["不启用", "小摩托", "小蝙蝠"]
    enabled: !TargetControl.busy && !TargetControl.running
    onCurrentIndexChanged: {
      if (currentIndex === 1)
        speedField.text = "35"
      else if (currentIndex === 2)
        speedField.text = "18"
      localError.text = ""
    }
  }

  GridLayout {
    visible: targetModel.currentIndex > 0
    columns: 2
    columnSpacing: 8
    rowSpacing: 5
    Layout.fillWidth: true

    Label { text: "巡航速度（m/s）" }
    TextField {
      id: speedField
      objectName: "speedField"
      Layout.fillWidth: true
      text: "35"
      selectByMouse: true
      validator: DoubleValidator { bottom: 0; top: 1000; decimals: 2 }
    }

    Label { text: "速度范围" }
    Label {
      objectName: "speedRangeLabel"
      text: root.speedMinimum() + "～" + root.speedMaximum() + " m/s"
      color: "#607d8b"
    }

    Label { text: "海拔高度（m）" }
    TextField {
      id: altitudeField
      objectName: "altitudeField"
      Layout.fillWidth: true
      text: TargetControl.defaultAltitudeM.toFixed(2)
      selectByMouse: true
      validator: DoubleValidator { bottom: -500; top: 10000; decimals: 3 }
    }

    Label { text: "来源距离（m）" }
    TextField {
      id: distanceField
      objectName: "distanceField"
      Layout.fillWidth: true
      text: "2000"
      selectByMouse: true
      validator: IntValidator { bottom: 100; top: 10000 }
    }

    Label { text: "距离范围" }
    Label {
      objectName: "distanceRangeLabel"
      text: "100～10000 m"
      color: "#607d8b"
    }

    Label { text: "来源方向" }
    ComboBox {
      id: directionBox
      objectName: "directionCombo"
      Layout.fillWidth: true
      model: ["从东向西", "从东北45度"]
    }

    Label { text: "地面站地址" }
    TextField {
      id: stationField
      objectName: "stationAddressField"
      Layout.fillWidth: true
      text: "192.168.1.41:7788"
      selectByMouse: true
    }
  }

  RowLayout {
    visible: targetModel.currentIndex > 0
    Layout.fillWidth: true
    spacing: 8

    Button {
      id: startButton
      objectName: "startTargetButton"
      Layout.fillWidth: true
      text: "启动靶机"
      enabled: !TargetControl.busy
      onClicked: {
        var error = root.validateForm()
        localError.text = error
        if (error.length !== 0)
          return
        TargetControl.StartTarget(
          root.selectedModel(), Number(speedField.text), Number(altitudeField.text),
          Number(distanceField.text),
          directionBox.currentIndex === 0 ? "east_west" : "northeast_45",
          stationField.text.trim())
      }
    }

    Button {
      id: stopButton
      objectName: "stopTargetButton"
      Layout.fillWidth: true
      text: "终止靶机"
      enabled: TargetControl.running && !TargetControl.busy
      onClicked: TargetControl.StopTarget()
    }
  }

  Label {
    id: localError
    objectName: "validationMessage"
    Layout.fillWidth: true
    visible: text.length > 0
    color: "#d32f2f"
    wrapMode: Text.WordWrap
    font.pixelSize: 12
  }

  Rectangle {
    visible: targetModel.currentIndex > 0
    Layout.fillWidth: true
    height: 1
    color: "#b0bec5"
  }

  Label {
    objectName: "targetStatusLabel"
    Layout.fillWidth: true
    visible: targetModel.currentIndex > 0
    text: TargetControl.status
    wrapMode: Text.WordWrap
    font.pixelSize: 12
  }

    Item { Layout.fillHeight: true }
  }
}
