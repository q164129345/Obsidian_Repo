# 一、前言
---
使用Grid布局元素，创建网格布局。
程序的效果：
![[录屏 06-28-2024 02_21_08 PM.webm]]

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
    title: "Grid Layout Example"

    Grid {
        columns: 3 // 一排的数量
        spacing: 10 // 间隔
        anchors.centerIn: parent // 使用锚点，基于主窗口的中间

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

        Rectangle {
            width: 100
            height: 100
            color: "yellow"
        }

        Rectangle {
            width: 100
            height: 100
            color: "purple"
        }

        Rectangle {
            width: 100
            height: 100
            color: "orange"
        }
    }
}
```

尝试将columns从3改为4，看看效果：
![[录屏 06-28-2024 02_30_22 PM.webm]]





