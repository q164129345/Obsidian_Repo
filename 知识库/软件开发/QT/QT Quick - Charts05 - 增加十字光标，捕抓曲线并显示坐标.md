# 导言
---
在[[QT Quick - Charts01 - 使用ChartView绘制一个正弦波]]，将数据可视化之后。接下来进一步优化代码，将曲线上的坐标用鼠标捕抓并显示出来，实时查看坐标的数值。
![[录屏 09-05-2024 05:17:58 PM.webm]]

# 一、QML
---
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
            axisX: ValuesAxis {
                id: axisX
                min: 0
                max: 100
                tickCount: 11
            }
            axisY: ValuesAxis {
                id: axisY
                min: -1
                max: 1
                tickCount: 5
            }
        }

        Timer {
            interval: 50 // 增加此值，会将曲线的更新频率变慢
            running: true
            repeat: true
            property real phase: 0
            onTriggered: {
                var sampleCount = 1000 // 增加采样的点数，更好捕抓
                var frequency = 0.5
                lineSeries.clear()
                chartView.pointsData = []  // 清空保存的数据
                for (var i = 0; i < sampleCount; ++i) {
                    var y = Math.sin(frequency * (i + phase) * Math.PI / 25)
                    lineSeries.append(i, y)
                    chartView.pointsData.push({x: i, y: y})  // 保存每个点的数据
                }
                phase += 1 // 减小相位增量，减慢曲线移动速度（减少之后，曲线变得更加丝滑）
                if (phase >= sampleCount)
                    phase -= sampleCount
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
                var threshold = 50  // 像素为单位（增加捕抓阀值，便于捕抓到曲线上的点）
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
                    tooltip.text = "x: " + closestPoint.x.toFixed(2) + "\n" + "y: " + closestPoint.y.toFixed(2)
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

# 二、调整参数，优化捕抓
---
## 方法 1: 减少相位增量`phase`
通过减小 `Timer` 中 `phase` 的增量，可以让曲线移动得更慢。比如，将 `phase` 的增量从 `5` 调整为更小的值，如 `1`。
```java
Timer {
    interval: 50
    running: true
    repeat: true
    property real phase: 0
    onTriggered: {
        var sampleCount = 100
        var frequency = 0.5
        lineSeries.clear()
        chartView.pointsData = []  // 清空保存的数据
        for (var i = 0; i < sampleCount; ++i) {
            var y = Math.sin(frequency * (i + phase) * Math.PI / 25)
            lineSeries.append(i, y)
            chartView.pointsData.push({x: i, y: y})  // 保存每个点的数据
        }
        phase += 1  // 减小相位增量，减慢曲线移动速度
        if (phase >= sampleCount)
            phase -= sampleCount
    }
}
```
## 方法 2: 增大 `interval`
增大 `Timer` 的 `interval` 值，可以减缓曲线更新的频率。比如，将 `interval` 从 `50` 毫秒增加到 `100` 或 `200` 毫秒。
```java
Timer {
    interval: 50
    running: true
    repeat: true
    property real phase: 0
    onTriggered: {
        var sampleCount = 100
        var frequency = 0.5
        lineSeries.clear()
        chartView.pointsData = []  // 清空保存的数据
        for (var i = 0; i < sampleCount; ++i) {
            var y = Math.sin(frequency * (i + phase) * Math.PI / 25)
            lineSeries.append(i, y)
            chartView.pointsData.push({x: i, y: y})  // 保存每个点的数据
        }
        phase += 1  // 减小相位增量，减慢曲线移动速度
        if (phase >= sampleCount)
            phase -= sampleCount
    }
}
```
## 方法 3: 增加数据点数`sampleCount`
通过增加采样点数，可以使曲线在同样的 `x` 范围内更加平滑，便于捕捉。将 `sampleCount` 从 `100` 增加到 `200` 或更高。
```java
Timer {
    interval: 50
    running: true
    repeat: true
    property real phase: 0
    onTriggered: {
        var sampleCount = 200  // 增加采样点数，减缓曲线的变化
        var frequency = 0.5
        lineSeries.clear()
        chartView.pointsData = []
        for (var i = 0; i < sampleCount; ++i) {
            var y = Math.sin(frequency * (i + phase) * Math.PI / 25)
            lineSeries.append(i, y)
            chartView.pointsData.push({x: i, y: y})
        }
        phase += 1
        if (phase >= sampleCount)
            phase -= sampleCount
    }
}
```
## 方法 4: 鼠标捕捉点改进
你还可以增加捕捉点的阈值，允许鼠标更容易捕捉到曲线上的点。例如，将 `threshold` 从 `10` 增加到 `15` 或 `20`，使鼠标捕捉更加灵敏。
```java
var threshold = 20  // 增大捕捉阈值，便于捕捉到曲线上的点
```
## 总结
- 减小 `phase` 增量、增大 `Timer` 间隔，或者增加采样点数，都会使曲线移动得更加平滑和缓慢。
- 增大捕捉点的阈值可以使鼠标更容易捕捉到曲线上的点，提升交互体验。










