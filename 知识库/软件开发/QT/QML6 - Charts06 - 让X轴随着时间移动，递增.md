# 导言
---
效果如下所示：
![[录屏 09-09-2024 09_31_46_PM.webm]]

# 一、QML代码
---
在上一章节的基础上修改main.qml代码。
```java
import QtQuick 2.15
import QtQuick.Controls 2.15
import QtCharts 2.15
import QtQuick.Window 2.15

ApplicationWindow {
    visible: true
    width: 640
    height: 480
    title: "Sine Wave Plot with Crosshair"

    ChartView {
        id: chartView
        anchors.fill: parent
        antialiasing: true

        // 数组保存曲线的所有点
        property var pointsData: []

        LineSeries {
            id: lineSeries
            useOpenGL: true
            axisX: ValueAxis {
                id: axisX
                min: 0
                max: 10  // 初始显示10秒
                tickCount: 11
                labelFormat: "%.0f"  // 设置标签格式为整数
            }
            axisY: ValueAxis {
                id: axisY
                min: -1
                max: 1
                tickCount: 5
            }
        }

        Timer {
            interval: 50  // 每50毫秒触发一次
            running: true
            repeat: true
            property real phase: 0
            property real timeElapsed: 0  // 记录经过的时间
            onTriggered: {
                var frequency = 0.5
                var y = Math.sin(frequency * (timeElapsed + phase) * Math.PI / 25)
                lineSeries.append(timeElapsed, y)  // X轴表示时间
                chartView.pointsData.push({x: timeElapsed, y: y})  // 保存每个点的数据

                phase += 1
                if (phase >= 1000)
                    phase -= 1000

                // 更新经过的时间
                timeElapsed += 0.05  // 每次触发增加0.05秒

                // 更新X轴的最小值和最大值
                if (timeElapsed < 10) {
                    axisX.min = 0
                    axisX.max = 10
                } else {
                    axisX.min = timeElapsed - 10
                    axisX.max = timeElapsed
                }
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


