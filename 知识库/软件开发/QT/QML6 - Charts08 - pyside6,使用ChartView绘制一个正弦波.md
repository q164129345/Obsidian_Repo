# 导言
---
使用pyside6开发GUI程序也是一个不错的选择。qml无论是在C++环境还是python环境，代码都一样。

# 一、main.py
----
```python
# This Python file uses the following encoding: utf-8
import sys
from pathlib import Path

#from PySide6.QtGui import QGuiApplication
from PySide6.QtQml import QQmlApplicationEngine
from PySide6.QtWidgets import QApplication


if __name__ == "__main__":
    app = QApplication(sys.argv) # 使用QApplication,否则程序崩溃
    engine = QQmlApplicationEngine()
    qml_file = Path(__file__).resolve().parent / "main.qml"
    engine.load(qml_file)
    if not engine.rootObjects():
        sys.exit(-1)
    sys.exit(app.exec())

    
```

# 二、main.qml
---
```java
import QtQuick 2.15  // 引入 QtQuick 模块，提供基本的 QML 类型
import QtQuick.Controls 2.15  // 引入 QtQuick.Controls 模块，提供控件类型
import QtCharts 2.15  // 引入 QtCharts 模块，提供绘制图表的类型
import QtQuick.Window 2.15  // 引入 QtQuick.Window 模块，提供窗口类型

ApplicationWindow {
    visible: true
    width: 640
    height: 480
    title: "Sine Wave Plot"  // 设置窗口标题

    ChartView {
        id: chartView
        anchors.fill: parent  // 填满整个窗口
        antialiasing: true  // 启用抗锯齿以获得更好的绘图质量

        LineSeries {
            id: lineSeries
            //useOpenGL: true  // 启用 OpenGL 以提高性能
            axisX: ValuesAxis {  // 定义 X 轴
                id: axisX
                min: 0  // X 轴最小值
                max: 100  // X 轴最大值
                tickCount: 11  // X 轴刻度数
            }
            axisY: ValuesAxis {  // 定义 Y 轴
                id: axisY
                min: -1  // Y 轴最小值
                max: 1  // Y 轴最大值
                tickCount: 5  // Y 轴刻度数
            }
        }

        Timer {
            interval: 50  // 定时器触发间隔为 50 毫秒
            running: true  // 定时器开始运行
            repeat: true  // 定时器重复触发
            property real phase: 0  // 添加一个相位属性
            onTriggered: {  // 定时器触发时执行的操作
                var sampleCount = 100  // 采样点数
                var frequency = 0.5  // 频率
                lineSeries.clear()  // 清空当前数据点
                for (var i = 0; i < sampleCount; ++i) {
                    var y = Math.sin(frequency * (i + phase) * Math.PI / 25)  // 计算正弦波 y 坐标
                    lineSeries.append(i, y)  // 将计算的点添加到线系列中
                }
                phase += 5  // 增加相位
                if (phase >= sampleCount)  // 如果相位超过采样点数，重置相位
                    phase -= sampleCount
            }
        }
    }
}

```
