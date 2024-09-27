# 一、前言
---
Qt Quick控件非常丰富，学习一下TabBar与TabButton。
![[Pasted image 20240822211752.png]]
根据官方文档的介绍，TabBar通常跟TabBotton一起使用，用于控制StackLayout切换Index。
![[Pasted image 20240822211812.png]]
程序的运行效果如下：
![[录屏 07-01-2024 03_34_20 PM.webm]]

对应的QML代码：
```jsx
import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Window

ApplicationWindow {
    visible: true
    width: 600
    height: 500
    title: qsTr("TabBar and StackLayout Example")

    ColumnLayout {
        anchors.fill: parent
        spacing: 0

        TabBar {
            id: tabBar
            currentIndex: stackLayout.currentIndex
            width: parent.width // 等于父类ColumnLayout的宽度，相当于ApplicationWindow的宽度
            TabButton {
                text: qsTr("首页")
                onClicked: stackLayout.currentIndex = 0
            }
            TabButton {
                text: qsTr("联系")
                onClicked: stackLayout.currentIndex = 1
            }
            TabButton {
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

# 二、细节补充
---
## 2.1、尝试将TabButton改为Button
![[Pasted image 20240822211850.png]]
如下图所示，程序虽然能跑起来，但是，按钮却不见了。官方的描述：
<aside> 💡 TabBar is populated with TabButton controls

</aside>


![[Pasted image 20240822211907.png]]
## 2.2、TabBar + TabButton + StackLayout还不如Button + StackLayout美观
### Button + StackLayout
![[Pasted image 20240822211956.png]]
### TabBar + TabButton + StackLayout
![[Pasted image 20240822212008.png]]