# 一、前言
---
使用定位和大小调整属性。
程序效果如下：
![[录屏 06-28-2024 05_18_35 PM.webm]]

# 二、QML代码
---
```jsx
import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

ApplicationWindow {
    visible: true
    width: 640
    height: 480
    title: "Positioning and Sizing Example"

    Rectangle {
        anchors.fill: parent
        color: "lightgray"

        Rectangle {
            width: parent.width * 0.5
            height: 100
            color: "red"
            anchors.top: parent.top // 顶部 对齐 父类的顶部
            anchors.horizontalCenter: parent.horizontalCenter // 水平方向的中心 对齐 父类的水平方向的中心
        }

        Rectangle {
            width: 100
            height: parent.height * 0.5
            color: "green"
            anchors.left: parent.left // 左侧 对齐 父类的左侧
            anchors.verticalCenter: parent.verticalCenter // 垂直方向的中心 对齐 父类的垂直方向的中心
        }

        Rectangle {
            width: parent.width * 0.3
            height: parent.height * 0.3
            color: "blue"
            anchors.right: parent.right // 右侧 对齐 父类的右侧
            anchors.bottom: parent.bottom // 底部 对齐 父类的底部
        }
    }
}
```