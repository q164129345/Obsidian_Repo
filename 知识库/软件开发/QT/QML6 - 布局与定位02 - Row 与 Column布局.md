# 一、前言
---
程序运行的效果如下：
![[录屏 06-28-2024 12_00_15 PM.webm]]

# 二、QML代码
---
```jsx
import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Controls 2.15

ApplicationWindow {
    visible: true
    width: 640
    height: 480
    title: "Row and Column Layout Example"

    Row {
        spacing: 10
        anchors.centerIn: parent

        Rectangle {
            width: 100
            height: 100
            color: "red"
        }

        Rectangle {
            width: 100
            height: 100
            color: "green"
        }

        Rectangle {
            width: 100
            height: 100
            color: "blue"
        }
    }

    Column {
        spacing: 10
        anchors.centerIn: parent

        Rectangle {
            width: 100
            height: 100
            color: "red"
        }

        Rectangle {
            width: 100
            height: 100
            color: "green"
        }

        Rectangle {
            width: 100
            height: 100
            color: "blue"
        }
    }
}
```