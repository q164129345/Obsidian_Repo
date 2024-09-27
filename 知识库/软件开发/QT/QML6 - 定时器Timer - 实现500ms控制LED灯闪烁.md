# 一、前言
---
QML原来也有定时器：
![[Pasted image 20240826191738.png]]
代码效果：
![[录屏 07-25-2024 02_07_27 PM.webm]]
# 二、代码
---
## 2.1、Main.qml
```cpp
import QtQuick
import QtQuick.Controls

ApplicationWindow {
    visible: true
    width: 300
    height: 300
    title: "LED Blinker"

    Rectangle {
        id: led
        width: 35
        height: 35
        radius: 25
        color: "green"
        anchors.centerIn: parent

        property bool isGreen: true // 确保属性声明正确

        Timer {
            id: blinkTimer
            interval: 500 // 500ms
            running: true
            repeat: true
            onTriggered: {
                // Toggle the LED color between green and red using the isGreen property
                if (led.isGreen) {
                    led.color = "red";
                } else {
                    led.color = "green";
                }
                led.isGreen = !led.isGreen; // Toggle the state
                console.log("LED color:", led.color);
            }
        }
    }
}

```