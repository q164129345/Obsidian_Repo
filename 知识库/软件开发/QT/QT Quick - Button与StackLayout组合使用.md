# 一、前言
---
这个是安富莱为H7-Tool实现的一款上位机软件，我正在使用Qt实现类似的窗口布局。
![[Pasted image 20240826190104.png]]
通过Qt的Controls控件Button与Layout控件StackLayout组合起来，实现类似的效果。
![[Pasted image 20240826190119.png]]
# 二、QML代码
---
```jsx
import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Window 2.15

ApplicationWindow {
    visible: true
    width: 400
    height: 300
    title: qsTr("Vertical Buttons and StackLayout Example")

    RowLayout {
        anchors.fill: parent

        ColumnLayout {
            spacing: 10
            Layout.alignment: Qt.AlignTop
            Layout.fillHeight: true

            Button {
                text: qsTr("首页")
                onClicked: stackLayout.currentIndex = 0
            }

            Button {
                text: qsTr("联系")
                onClicked: stackLayout.currentIndex = 1
            }

            Button {
                text: qsTr("关于")
                onClicked: stackLayout.currentIndex = 2
            }
        }

        StackLayout {
            id: stackLayout
            Layout.fillWidth: true
            Layout.fillHeight: true
            currentIndex: 0

            Rectangle {
                color: "#00B000"
                Text {
                    text: qsTr("首页")
                    anchors.centerIn: parent
                    font.pixelSize: 24
                }
            }

            Rectangle {
                color: "steelblue"
                Text {
                    text: qsTr("联系")
                    anchors.centerIn: parent
                    font.pixelSize: 24
                }
            }

            Rectangle {
                color: "lightgrey"
                Text {
                    text: qsTr("关于")
                    anchors.centerIn: parent
                    font.pixelSize: 24
                }
            }
        }
    }
}
```

# 三、细节补充
---
## 3.1、增加TabBar与TabButton
![[Pasted image 20240826190144.png]]