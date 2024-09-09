# 一、前言
---
Qt版本：
![[Pasted image 20240826191952.png]]
程序效果：
![[录屏 07-26-2024 02_49_52 PM.webm]]

# 二、程序
---
## 2.1、CMakeLists.txt
添加QtCharts模块。
![[Pasted image 20240826192030.png]]
## 2.2、main.cpp
```cpp
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv); // qml使用QtCharts之后，不能使用QGuiApplication,否则程序崩溃

    QQmlApplicationEngine engine;
    const QUrl url(QStringLiteral("qrc:/Make_Line/Main.qml"));
    QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
        &app, [url](QObject *obj, const QUrl &objUrl) {
            if (!obj && url == objUrl)
                QCoreApplication::exit(-1);
        }, Qt::QueuedConnection);
    engine.load(url);

    return app.exec();
}

```

## 2.3、Main.qml
```cpp
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
            useOpenGL: true  // 启用 OpenGL 以提高性能
            axisX: ValueAxis {  // 定义 X 轴
                id: axisX
                min: 0  // X 轴最小值
                max: 100  // X 轴最大值
                tickCount: 11  // X 轴刻度数
            }
            axisY: ValueAxis {  // 定义 Y 轴
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

### 详细说明

- `ApplicationWindow`：创建一个应用程序窗口。
- `ChartView`：用于显示图表的容器。
- `LineSeries`：定义线条系列，用于绘制正弦波。
- `ValueAxis`：定义图表的 X 轴和 Y 轴。
- `Timer`：定时器，用于周期性地更新正弦波的相位，实现动画效果。

# 三、细节补充
---
## 3.1、ValueAxis与ValuesAxis
如下图所示，Qt Creator并不认识ValueAxis。但是，程序也能正常编译运行。
![[Pasted image 20240826192058.png]]
有意思的是，官方文档能找到ValueAxis。网络找资料与问ChatGPT都找不到原因。
![[Pasted image 20240826192112.png]]
如下图所示，将ValueAxis改为ValuesAxis之后，QtCreator可以认识它了。编译、运行正常。
![[Pasted image 20240826192126.png]]
奇怪的是，在官方文档找不到ValuesAxis的信息。我怀疑是Qt的bug。
![[Pasted image 20240826192137.png]]

## 3.2 macos使用useOpenGL: true后，无法显示曲线
```java
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
```
使用macos时，需要备注掉useOpenGL，或者useOpenGL:false关闭这个功能，原因如下：
`macOS 在逐步过渡到 Apple 自己的图形框架 Metal，OpenGL 已经不再是 macOS 上的首选图形 API，OpenGL 的支持变得不太可靠。尽管 Qt 依然支持 OpenGL，但是在 macOS 上，某些版本的 OpenGL 可能无法正常工作或表现得不够稳定。`




