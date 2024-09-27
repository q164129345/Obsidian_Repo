![[录屏2024-09-12 07.35.47_720p.mov]]
效果如上。

## 一、QML代码
----
```java
import QtQuick 2.15  // 导入 QtQuick 模块，提供基本的 QML 组件
import QtQuick.Controls 2.15  // 导入 QtQuick 控件模块，提供 UI 控件
import QtCharts 2.15  // 导入 QtCharts 模块，提供图表功能
import QtQuick.Window 2.15  // 导入 QtQuick 窗口模块，提供窗口功能

ApplicationWindow {  // 创建应用程序窗口
    visible: true  // 窗口可见
    width: 640  // 窗口宽度
    height: 480  // 窗口高度
    title: "Sine Wave Plot with Crosshair"  // 窗口标题

    ChartView {  // 创建图表视图
        id: chartView  // 设置图表视图的 ID
        anchors.fill: parent  // 填充父组件
        legend.visible: true  // 显示图例
        antialiasing: true  // 启用抗锯齿

        // 数组保存曲线的所有点
        property var pointsData: []  // 定义一个空数组，用于存储点数据

        ValueAxis {  // 创建 X 轴
            id: axisX  // 设置 X 轴的 ID
            min: 0  // X 轴最小值
            max: 10  // X 轴最大值
            titleText: "Time (s)"  // X 轴标题
        }

        ValueAxis {  // 创建 Y 轴
            id: axisY  // 设置 Y 轴的 ID
            min: -1  // Y 轴最小值
            max: 1  // Y 轴最大值
            titleText: "Value"  // Y 轴标题
        }

        LineSeries {  // 创建正弦曲线系列
            id: lineSeries  // 设置正弦曲线的 ID
            name: "Sine"  // 曲线名称
            useOpenGL: false  // 不使用 OpenGL
            axisX: axisX  // 绑定 X 轴
            axisY: axisY  // 绑定 Y 轴
        }

        LineSeries {  // 创建余弦曲线系列
            id: cosLineSeries  // 设置余弦曲线的 ID
            name: "Cosine"  // 曲线名称
            useOpenGL: false  // 不使用 OpenGL
            axisX: axisX  // 绑定 X 轴
            axisY: axisY  // 绑定 Y 轴
        }

        Timer {  // 创建定时器
            interval: 50  // 每 50 毫秒触发一次
            running: true  // 定时器运行
            repeat: true  // 定时器重复
            property real phase: 0  // 定义相位属性
            property real timeElapsed: 0  // 记录经过的时间
            onTriggered: {  // 定时器触发时执行
                var frequency = 0.5  // 定义频率
                var y = Math.sin(frequency * (timeElapsed + phase) * Math.PI / 25)  // 计算正弦值
                var yCos = Math.cos(frequency * (timeElapsed + phase) * Math.PI / 25)  // 计算余弦值
                lineSeries.append(timeElapsed, y)  // 将正弦值添加到曲线
                cosLineSeries.append(timeElapsed, yCos)  // 将余弦值添加到曲线
                chartView.pointsData.push({x: timeElapsed, y: y, yCos: yCos})  // 保存每个点的数据

                phase += 1  // 增加相位
                if (phase >= 1000)  // 如果相位超过 1000
                    phase -= 1000  // 重置相位

                // 更新经过的时间
                timeElapsed += 0.05  // 每次触发增加 0.05 秒

                // 更新 X 轴的最小值和最大值
                if (timeElapsed < 10) {  // 如果经过的时间小于 10
                    axisX.min = 0  // 设置 X 轴最小值为 0
                    axisX.max = 10  // 设置 X 轴最大值为 10
                } else {  // 否则
                    axisX.min = timeElapsed - 10  // 设置 X 轴最小值为当前时间减去 10
                    axisX.max = timeElapsed  // 设置 X 轴最大值为当前时间
                }
            }
        }

        // 鼠标域，用于捕获鼠标的位置
        MouseArea {  // 创建鼠标区域
            anchors.fill: parent  // 填充父组件
            hoverEnabled: true  // 启用鼠标悬停
            onPositionChanged: {  // 鼠标位置改变时执行
                // 获取鼠标位置对应的曲线数据点
                var mappedValueSin = chartView.mapToValue(Qt.point(mouseX, mouseY), lineSeries)  // 将鼠标位置映射到正弦曲线
                var mappedValueCos = chartView.mapToValue(Qt.point(mouseX, mouseY), cosLineSeries)  // 将鼠标位置映射到余弦曲线
                var xValSin = Math.round(mappedValueSin.x)  // 四舍五入取整 x 值
                var xValCos = Math.round(mappedValueCos.x)  // 四舍五入取整 x 值
                var yValSin = Math.sin(xValSin * Math.PI / 50)  // 计算正弦波 y 值
                var yValCos = Math.cos(xValCos * Math.PI / 50)  // 计算余弦波 y 值

                // 设定一个捕捉阈值，当鼠标接近曲线点才显示坐标
                var threshold = 50  // 像素为单位
                var closestPoint = null  // 最近点初始化为 null
                var closestDistance = Number.MAX_VALUE  // 最近距离初始化为最大值
                var pointPos = null;  // 提前声明 pointPos 变量

                for (var i = 0; i < chartView.pointsData.length; ++i) {  // 遍历所有点
                    var point = chartView.pointsData[i]  // 获取当前点
                    var pointPosSin = chartView.mapToPosition(Qt.point(point.x, point.y), lineSeries)  // 计算正弦点的位置
                    var pointPosCos = chartView.mapToPosition(Qt.point(point.x, point.yCos), cosLineSeries)  // 计算余弦点的位置

                    var distanceSin = Math.sqrt(Math.pow(mouseX - pointPosSin.x, 2) +  // 计算鼠标到正弦点的距离
                                                Math.pow(mouseY - pointPosSin.y, 2))
                    var distanceCos = Math.sqrt(Math.pow(mouseX - pointPosCos.x, 2) +  // 计算鼠标到余弦点的距离
                                                Math.pow(mouseY - pointPosCos.y, 2))

                    if (distanceSin < threshold && distanceSin < closestDistance) {  // 如果正弦点在阈值内且距离最近
                        closestPoint = {x: point.x, y: point.y}  // 更新最近点
                        closestDistance = distanceSin  // 更新最近距离
                        pointPos = pointPosSin  // 更新点位置
                    }

                    if (distanceCos < threshold && distanceCos < closestDistance) {  // 如果余弦点在阈值内且距离最近
                        closestPoint = {x: point.x, y: point.yCos}  // 更新最近点
                        closestDistance = distanceCos  // 更新最近距离
                        pointPos = pointPosCos  // 更新点位置
                    }
                }

                // 如果找到了最近的点且距离在阈值内，显示十字光标和提示框
                if (closestPoint !== null) {  // 如果找到最近点
                    crosshairCanvas.crosshairX = pointPos.x  // 设置十字光标 X 坐标
                    crosshairCanvas.crosshairY = pointPos.y  // 设置十字光标 Y 坐标
                    crosshairCanvas.requestPaint()  // 触发 Canvas 重绘
                    crosshairCanvas.visible = true  // 显示十字光标

                    tooltip.x = crosshairCanvas.crosshairX + 10  // 设置提示框 X 坐标
                    tooltip.y = crosshairCanvas.crosshairY - 30  // 设置提示框 Y 坐标
                    tooltip.text = "x: " + closestPoint.x.toFixed(2) + "\n" + "y: " + closestPoint.y.toFixed(2)  // 设置提示框文本
                    tooltip.visible = true  // 显示提示框
                } else {  // 如果没有找到匹配点
                    crosshairCanvas.visible = false  // 隐藏十字光标
                    tooltip.visible = false  // 隐藏提示框
                }
            }
            onExited: {  // 鼠标离开时执行
                // 鼠标离开图表区域时，隐藏十字光标和提示框
                crosshairCanvas.visible = false  // 隐藏十字光标
                tooltip.visible = false  // 隐藏提示框
            }
        }

        // 画十字光标的 Canvas
        Canvas {  // 创建画布
            id: crosshairCanvas  // 设置画布的 ID
            anchors.fill: parent  // 填充父组件
            visible: false  // 初始隐藏
            property real crosshairX: 0  // 十字光标 X 坐标
            property real crosshairY: 0  // 十字光标 Y 坐标
            z: 1  // 设置较高的层级，以确保绘制在图表上方

            onPaint: {  // 绘制时执行
                var ctx = getContext("2d")  // 获取 2D 上下文
                ctx.clearRect(0, 0, width, height)  // 清空画布

                ctx.strokeStyle = "purple"  // 设置线条颜色为紫色
                ctx.lineWidth = 1  // 设置线条宽度

                // 绘制竖线
                ctx.beginPath()  // 开始路径
                ctx.moveTo(crosshairX, 0)  // 移动到十字光标 X 坐标
                ctx.lineTo(crosshairX, height)  // 绘制到画布底部
                ctx.stroke()  // 绘制路径

                // 绘制横线
                ctx.beginPath()  // 开始路径
                ctx.moveTo(0, crosshairY)  // 移动到十字光标 Y 坐标
                ctx.lineTo(width, crosshairY)  // 绘制到画布右侧
                ctx.stroke()  // 绘制路径
            }
        }

        // 鼠标位置显示数据提示框
        ToolTip {  // 创建提示框
            id: tooltip  // 设置提示框的 ID
            visible: false  // 初始隐藏
            background: Rectangle {  // 提示框背景
                color: "black"  // 背景颜色为黑色
                border.color: "white"  // 边框颜色为白色
                radius: 5  // 边框圆角半径
                opacity: 0.8  // 透明度
            }
            contentItem: Text {  // 提示框内容
                font.bold: true  // 字体加粗
                font.pointSize: 12  // 字体大小
                color: "white"  // 字体颜色为白色
                text: tooltip.text  // 显示提示框文本
            }
        }
    }
}

```



