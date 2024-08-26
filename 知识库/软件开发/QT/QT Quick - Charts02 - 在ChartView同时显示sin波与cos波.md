# 一、前言
---
上一章节实现sin曲线的显示。本章节在坐标轴上增加多一条线cos曲线。
![[录屏2024-08-03 21.01.52_720p.mov]]
# 二、代码
---
## 2.1、CMakeLists.txt
CMakeLists.txt添加Charts库
![[Pasted image 20240826192305.png]]
## 2.2、main.cpp
![[Pasted image 20240826192316.png]]
## 2.3、Main.qml
```jsx
import QtQuick 2.15  // 引入 QtQuick 模块，提供基本的 QML 类型
import QtQuick.Controls 2.15  // 引入 QtQuick.Controls 模块，提供控件类型
import QtCharts 2.15  // 引入 QtCharts 模块，提供绘制图表的类型
import QtQuick.Window 2.15  // 引入 QtQuick.Window 模块，提供窗口类型

ApplicationWindow {
    visible: true  // 窗口可见
    width: 640  // 窗口宽度
    height: 480  // 窗口高度
    title: "Sine and Cosine Wave Plot"  // 设置窗口标题

    ChartView {
        id: chartView
        anchors.fill: parent  // 填满整个窗口
        antialiasing: true  // 启用抗锯齿以获得更好的绘图质量

        ValueAxis {
            id: axisX
            min: 0  // X 轴最小值
            max: 100  // X 轴最大值
            tickCount: 11  // X 轴刻度数
        }
        ValueAxis {
            id: axisY
            min: -1  // Y 轴最小值
            max: 1  // Y 轴最大值
            tickCount: 5  // Y 轴刻度数
        }

        LineSeries {
            id: sineSeries
            name: "Sine Wave"  // 设置线系列名称为“正弦波”
            useOpenGL: true  // 启用 OpenGL 以提高性能
            axisX: axisX  // 绑定 X 轴
            axisY: axisY  // 绑定 Y 轴
        }

        LineSeries {
            id: cosineSeries
            name: "Cosine Wave"  // 设置线系列名称为“余弦波”
            useOpenGL: true  // 启用 OpenGL 以提高性能
            axisX: axisX  // 绑定 X 轴
            axisY: axisY  // 绑定 Y 轴
        }

        Timer {
            interval: 50  // 定时器触发间隔为 50 毫秒
            running: true  // 定时器开始运行
            repeat: true  // 定时器重复触发
            property real phase: 0  // 添加一个相位属性
            onTriggered: {  // 定时器触发时执行的操作
                var sampleCount = 100  // 采样点数
                var frequency = 0.5  // 频率
                sineSeries.clear()  // 清空正弦波数据点
                cosineSeries.clear()  // 清空余弦波数据点
                for (var i = 0; i < sampleCount; ++i) {
                    var sineY = Math.sin(frequency * (i + phase) * Math.PI / 25)  // 计算正弦波 y 坐标
                    var cosineY = Math.cos(frequency * (i + phase) * Math.PI / 25)  // 计算余弦波 y 坐标
                    sineSeries.append(i, sineY)  // 将正弦波点添加到线系列中
                    cosineSeries.append(i, cosineY)  // 将余弦波点添加到线系列中
                }
                phase += 5  // 增加相位
                if (phase >= sampleCount)  // 如果相位超过采样点数，重置相位
                    phase -= sampleCount
            }
        }
    }
}
```