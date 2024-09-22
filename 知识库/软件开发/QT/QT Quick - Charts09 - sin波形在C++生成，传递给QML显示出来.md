# 导言
---
[[QT Quick - Charts06 - 让X轴随着时间移动，递增]]。之前，sin波形是在main.qml的定时器里生成。今天，尝试把这一块功能搬到C++里实现。
![[录屏2024-09-22 10.35.45.mov]]

# 一、main.cpp
---
```c
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QApplication>
#include <QTimer>
#include <QQmlContext>
#include <cmath>
#include <QDebug>

class SineWaveGenerator : public QObject {
    Q_OBJECT
public:
    SineWaveGenerator(QObject *parent = nullptr) : QObject(parent), phase(0), timeElapsed(0) {
        connect(&timer, &QTimer::timeout, this, &SineWaveGenerator::generatePoint);
        timer.start(50); // 每50毫秒触发一次
    }

signals:
    void newPoint(qreal x, qreal y);

private slots:
    void generatePoint() {
        qreal frequency = 0.5;
        qreal y = std::sin(frequency * (timeElapsed + phase) * M_PI / 25);
        emit newPoint(timeElapsed, y);
        qDebug() << "Generated point:" << timeElapsed << y; // 调试信息

        phase += 1;
        if (phase >= 1000)
            phase -= 1000;

        timeElapsed += 0.05; // 每次触发增加0.05秒
    }

private:
    QTimer timer;
    qreal phase;  // 用于计算当前的相位
    qreal timeElapsed; // 用于计算当前的时间
};

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    QQmlApplicationEngine engine;
    SineWaveGenerator generator;
    engine.rootContext()->setContextProperty("sineWaveGenerator", &generator);

    QObject::connect(
        &engine,
        &QQmlApplicationEngine::objectCreationFailed,
        &app,
        []() { QCoreApplication::exit(-1); },
        Qt::QueuedConnection);
    engine.loadFromModule("sin_Line", "Main");

    return app.exec();
}

#include "main.moc"

```

# 二、main.qml
```java
import QtQuick 2.15  // 引入 QtQuick 模块，提供基本的 QML 类型
import QtQuick.Controls 2.15  // 引入 QtQuick.Controls 模块，提供控件类型
import QtCharts 2.15  // 引入 QtCharts 模块，提供绘制图表的类型
import QtQuick.Window 2.15  // 引入 QtQuick.Window 模块，提供窗口类型

ApplicationWindow {
    visible: true
    width: 640
    height: 480
    title: "Sine Wave Plot with Crosshair"

    function handleNewPoint(x, y) {
        console.log("Received point:", x, y) // 调试信息
        lineSeries.append(x, y)
        chartView.pointsData.push({x: x, y: y})

        if (x < 10) {
            axisX.min = 0
            axisX.max = 10
        } else {
            axisX.min = x - 10
            axisX.max = x
        }
    }

    ChartView {
        id: chartView
        anchors.fill: parent
        antialiasing: true

        property var pointsData: []

        LineSeries {
            id: lineSeries
            useOpenGL: false
            axisX: ValuesAxis {
                id: axisX
                min: 0
                max: 10
                tickCount: 11
                labelFormat: "%.0f"
            }
            axisY: ValuesAxis {
                id: axisY
                min: -1
                max: 1
                tickCount: 5
            }
        }

        Connections {
            target: sineWaveGenerator
            onNewPoint: {
                // console.log("Signal received in Connections element"); // 调试信息
                handleNewPoint(x, y);
            }
        }

        // 鼠标区域，用于捕获鼠标的位置
        MouseArea {
            anchors.fill: parent
            hoverEnabled: true
            onPositionChanged: {
                // 获取鼠标位置对应的曲线数据点
                var mappedValue = chartView.mapToValue(Qt.point(mouseX, mouseY), lineSeries)
                var xVal = Math.round(mappedValue.x)  // 四舍五入取整 x 值
                var yVal = Math.sin(xVal * Math.PI / 50)  // 计算正弦波 y 值

                // 设定一个捕捉阈值，当鼠标接近曲线点时才显示坐标
                var threshold = 50  // 像素为单位
                var closestPoint = null
                var closestDistance = Number.MAX_VALUE
                var pointPos = null;  // 提前声明 pointPos 变量

                for (var i = 0; i < chartView.pointsData.length; ++i) {
                    var point = chartView.pointsData[i]
                    pointPos = chartView.mapToPosition(Qt.point(point.x, point.y), lineSeries)  // 计算点的位置

                    var distance = Math.sqrt(Math.pow(mouseX - pointPos.x, 2) +
                                             Math.pow(mouseY - pointPos.y, 2))
                    if (distance < threshold && distance < closestDistance) {
                        closestPoint = point
                        closestDistance = distance
                    }
                }

                // 如果找到了最近的点且距离在阈值内，显示十字光标和提示框
                if (closestPoint !== null) {
                    pointPos = chartView.mapToPosition(Qt.point(closestPoint.x, closestPoint.y), lineSeries)
                    crosshairCanvas.crosshairX = pointPos.x
                    crosshairCanvas.crosshairY = pointPos.y
                    crosshairCanvas.requestPaint()  // 触发 Canvas 重绘
                    crosshairCanvas.visible = true

                    tooltip.x = crosshairCanvas.crosshairX + 10
                    tooltip.y = crosshairCanvas.crosshairY - 30
                    // toFiexed()限制坐标的浮点数位置，toFixed(2)的意思是只显示小数点后两位
                    tooltip.text = "x: " + closestPoint.x.toFixed(2) + "\n" + "y: " + closestPoint.y.toFixed(2) // 显示最近点的坐标
                    tooltip.visible = true
                } else {
                    // 如果没有找到匹配点，隐藏十字光标和提示框
                    crosshairCanvas.visible = false
                    tooltip.visible = false
                }
            }
            onExited: {
                // 鼠标离开图表区域时，隐藏十字光标和提示框
                crosshairCanvas.visible = false
                tooltip.visible = false
            }
        }

        // 画十字光标的 Canvas
        Canvas {
            id: crosshairCanvas
            anchors.fill: parent
            visible: false
            property real crosshairX: 0
            property real crosshairY: 0
            z: 1  // 设置较高的层级，以确保绘制在图表上方

            onPaint: {
                var ctx = getContext("2d")
                ctx.clearRect(0, 0, width, height)

                ctx.strokeStyle = "purple"  // 设置线条颜色为紫色
                ctx.lineWidth = 1

                // 绘制竖线
                ctx.beginPath()
                ctx.moveTo(crosshairX, 0)
                ctx.lineTo(crosshairX, height)
                ctx.stroke()

                // 绘制横线
                ctx.beginPath()
                ctx.moveTo(0, crosshairY)
                ctx.lineTo(width, crosshairY)
                ctx.stroke()
            }
        }

        // 鼠标位置显示数据提示框
        ToolTip {
            id: tooltip
            visible: false
            background: Rectangle {
                color: "black"
                border.color: "white"
                radius: 5
                opacity: 0.8
            }
            contentItem: Text {
                font.bold: true
                font.pointSize: 12
                color: "white"
                text: tooltip.text
            }
        }
    }
}

```

# 三、CMakeLists.txt
```cmake
cmake_minimum_required(VERSION 3.16)

project(sin_Line VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 6.5 REQUIRED COMPONENTS Quick Charts)

qt_standard_project_setup(REQUIRES 6.5)

qt_add_executable(appsin_Line
    main.cpp
)

qt_add_qml_module(appsin_Line
    URI sin_Line
    VERSION 1.0
    QML_FILES
        Main.qml
)

# Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
# If you are developing for iOS or macOS you should consider setting an
# explicit, fixed bundle identifier manually though.
set_target_properties(appsin_Line PROPERTIES
#    MACOSX_BUNDLE_GUI_IDENTIFIER com.example.appsin_Line
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

target_link_libraries(appsin_Line
    PRIVATE Qt6::Quick Qt6::Charts
)

include(GNUInstallDirs)
install(TARGETS appsin_Line
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

```

# 四、细节补充
---
## 4.1、

