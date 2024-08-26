# 一、前言
---
从官方文档看到，ChartView的属性与方法都挺多的，信号只有两个。
![[Pasted image 20240826192413.png]]
![[Pasted image 20240826192432.png]]

实验例子：
```jsx
import QtQuick 2.15
import QtQuick.Controls 2.15
import QtCharts 2.15

ApplicationWindow {
    visible: true
    width: 1000
    height: 800
    title: qsTr("电机速度可视化")

    ChartView {
        id: chartView
        anchors.fill: parent
        antialiasing: true
        animationDuration: 500  // 动画持续时间为500毫秒
        animationOptions: ChartView.SeriesAnimations  // 设置动画选项

        ValuesAxis {
            id: axisX
            titleText: "时间 (s)"
            min: 0
            max: 19
            tickCount: 20
        }

        ValuesAxis {
            id: axisY
            titleText: "电机速度 (RPM)"
            min: 0
            max: 5000
            tickCount: 10
        }

        LineSeries {
            id: lineSeries
            name: "电机速度"
            axisX: axisX
            axisY: axisY
            color: "blue"
        }
    }

    Timer {
        interval: 500
        running: true
        repeat: true
        onTriggered: {
            let xValue = lineSeries.count
            let yValue = Math.random() * 5000
            lineSeries.append(xValue, yValue)

            // 动态调整X轴的最大值
            if (xValue > axisX.max) {
                axisX.max = xValue + 10  // 新数据点添加时将最大值递增一些
            }
        }
    }
}

```

# 二、属性
---
## 2.1、animationDuration（动画持续时间）
用于设置图表动画的持续时间（以毫秒为单位）。它控制图表中动画的时长。animationDuration 仅仅设置了动画持续时间，而动画的类型和触发条件也需要配置。如果没有显式指定动画类型或触发条件，可能看不到明显的变化。
## 2.2、animationOptions（动画选项）
设置图表的动画类型，要跟animationDuration配合使用，动画的类型如下：
1. **ChartView.NoAnimation ：**图表中的动画被禁用。这是默认值。
2. **ChartView.GridAxisAnimations** ：图表中的网格轴动画已启用。
3. **ChartView.SeriesAnimations ：**图表中的系列动画已启用。
4. **ChartView.AllAnimations ：**图表中的所有动画类型均已启用。

### 2.2.1、使用animationDuration与animationOptions的效果
![[录屏 07-29-2024 10_04_44 AM.webm]]

### 2.2.2、不使用animationDuration与animationOptions的效果
![[录屏 07-29-2024 10_06_01 AM.webm]]
对比上面两个视频，可以了解到animationDuration与animationOptions组合使用的效果。

## 2.3、backgroundColor与backgroundRoundness
backgroundColor：图表背景的颜色。默认情况下，背景颜色由图表主题指定。
backgroundRoundness：图表背景角上的圆的直径。
![[Pasted image 20240826192626.png]]
程序的效果如下：
![[Pasted image 20240826192638.png]]
## 2.4、**dropShadowEnabled**
**dropShadowEnabled设为true时，ChartView边缘的视觉效果要好看一点点，有一点点3D的感觉。**
![[Pasted image 20240826192657.png]]
## 2.5、count
如下图所示，lineSeries.count在每一次Timer回调之后自动地++1。
![[Pasted image 20240826192716.png]]
![[Pasted image 20240826192736.png]]


## 2.6、legend
如下图所示，通过修改legned可以让图例发生变化。
**常用属性**
• alignment：设置图例的位置。可以是Qt.AlignTop、Qt.AlignBottom、Qt.AlignLeft或Qt.AlignRight。
• backgroundVisible：控制图例的背景是否可见，布尔值。
• borderColor：设置图例边框的颜色。
• color：设置图例文本的颜色。
• font：设置图例文本的字体。
• labelColor：设置图例标签的颜色。
• markerShape：设置图例标记的形状，可以是Legend.MarkerShapeDefault、Legend.MarkerShapeCircle或Legend.MarkerShapeRectangle。
• reverseMarkers：控制是否反转图例标记的顺序，布尔值。
• showToolTips：控制是否显示图例工具提示，布尔值。
• visible：控制图例的可见性，布尔值。
![[Pasted image 20240826192802.png]]
## 2.7、theme
回到最开始的代码，theme可以很方便地改变图表的样式。官方文档：
![[Pasted image 20240826192820.png]]
```jsx
import QtQuick 2.15  // 导入 Qt Quick 2.15 模块，用于基本的 QML 类型
import QtQuick.Controls 2.15  // 导入 Qt Quick Controls 2.15 模块，用于应用程序控件
import QtCharts 2.15  // 导入 Qt Charts 2.15 模块，用于创建图表
import QtQml 2.15  // 导入 Qt QML 2.15 模块，用于 QML 中的高级功能

ApplicationWindow {
    visible: true  // 设置窗口为可见
    width: 1000  // 设置窗口宽度为 1000 像素
    height: 800  // 设置窗口高度为 800 像素
    title: qsTr("电机速度可视化")  // 设置窗口标题为 "电机速度可视化"

    ChartView {
        id: chartView  // 设置 ChartView 的 ID 为 chartView
        anchors.fill: parent  // 将 ChartView 的大小设置为填满父窗口
        antialiasing: true  // 启用抗锯齿以提高图表的绘制质量
        animationDuration: 500  // 设置动画持续时间为 500 毫秒
        animationOptions: ChartView.SeriesAnimations  // 设置动画选项为系列动画
        dropShadowEnabled: true  // 启用阴影效果
        theme: ChartView.ChartThemeBlueCerulean  // 设置图表主题为蓝色天青主题

        ValueAxis {
            id: axisX  // 设置 X 轴的 ID 为 axisX
            titleText: "时间 (s)"  // 设置 X 轴的标题文本为 "时间 (s)"
            min: 0  // 设置 X 轴的最小值为 0
            max: 19  // 设置 X 轴的最大值为 19
            tickCount: 20  // 设置 X 轴的刻度数为 20
        }

        ValueAxis {
            id: axisY  // 设置 Y 轴的 ID 为 axisY
            titleText: "电机速度 (RPM)"  // 设置 Y 轴的标题文本为 "电机速度 (RPM)"
            min: 0  // 设置 Y 轴的最小值为 0
            max: 5000  // 设置 Y 轴的最大值为 5000
            tickCount: 10  // 设置 Y 轴的刻度数为 10
        }

        LineSeries {
            id: lineSeries1  // 设置第一个折线系列的 ID 为 lineSeries1
            name: "电机速度1"  // 设置第一个折线系列的名称为 "电机速度1"
            axisX: axisX  // 将第一个折线系列的 X 轴设置为 axisX
            axisY: axisY  // 将第一个折线系列的 Y 轴设置为 axisY
            color: "blue"  // 设置第一个折线系列的颜色为蓝色
        }

        LineSeries {
            id: lineSeries2  // 设置第二个折线系列的 ID 为 lineSeries2
            name: "电机速度2"  // 设置第二个折线系列的名称为 "电机速度2"
            axisX: axisX  // 将第二个折线系列的 X 轴设置为 axisX
            axisY: axisY  // 将第二个折线系列的 Y 轴设置为 axisY
            color: "red"  // 设置第二个折线系列的颜色为红色
        }
    }

    Timer {
        interval: 500  // 设置计时器的间隔为 500 毫秒
        running: true  // 设置计时器为运行状态
        repeat: true  // 设置计时器为重复触发
        onTriggered: {
            // 获取第一个折线系列的数据点数量作为 x 轴值
            let xValue = lineSeries1.count
            // 生成 0 到 5000 之间的随机 y 值作为第一个折线系列的新数据点
            let yValue1 = Math.random() * 5000
            // 生成 0 到 5000 之间的随机 y 值作为第二个折线系列的新数据点
            let yValue2 = Math.random() * 5000
            // 将新数据点添加到第一个折线系列
            lineSeries1.append(xValue, yValue1)
            // 将新数据点添加到第二个折线系列
            lineSeries2.append(xValue, yValue2)

            // 动态调整 X 轴的最大值
            if (xValue > axisX.max) {
                // 如果新数据点的 x 值超过当前 X 轴的最大值，将 X 轴的最大值增加 10
                axisX.max = xValue + 10
            }
        }
    }
}
```

### 2.7.1、ChartView.ChartThemeBlueCerulean（蓝色天清主题）
![[Pasted image 20240826192845.png]]

### 2.7.2、ChartView.ChartThemeBrownSand：棕色沙地主题
![[Pasted image 20240826192857.png]]

### 2.7.3、ChartView.ChartThemeBlueNcs：蓝色NCS主题
![[Pasted image 20240826192913.png]]

### 2.7.4、ChartView.ChartThemeHighContrast：高对比度主题
![[Pasted image 20240826192925.png]]

### 2.7.5、ChartView.ChartThemeBlueIcy：冰蓝主题
![[Pasted image 20240826192938.png]]

### 2.7.6、ChartView.ChartThemeQt：Qt主题
![[Pasted image 20240826192949.png]]
## 2.8、title、titleColor、titleFont
![[Pasted image 20240826193002.png]]
如上所示，可以为图表增加一个主题，设置主题的名字，字体大小，颜色等。

# 三、方法
---
## 3.1、scrollLeft()、**scrollRight()、scrollUp()、scrollDown()**
```jsx
import QtQuick 2.15
import QtQuick.Controls 2.15
import QtCharts 2.15
import QtQml 2.15

ApplicationWindow {
    visible: true  // 设置窗口为可见
    width: 1000  // 设置窗口宽度为 1000 像素
    height: 800  // 设置窗口高度为 800 像素
    title: qsTr("电机速度可视化")  // 设置窗口标题为 "电机速度可视化"

    ChartView {
        id: chartView  // 设置 ChartView 的 ID 为 chartView
        anchors.fill: parent  // 将 ChartView 的大小设置为填满父窗口
        antialiasing: true  // 启用抗锯齿以提高图表的绘制质量
        animationDuration: 500  // 设置动画持续时间为 500 毫秒
        animationOptions: ChartView.SeriesAnimations  // 设置动画选项为系列动画
        dropShadowEnabled: true  // 启用阴影效果
        theme: ChartView.ChartThemeBrownSand  // 设置图表主题为蓝色天青主题
        title: "电机速度实时监控"  // 设置图表标题
        titleColor: "darkblue"  // 设置图表标题颜色为深蓝色
        titleFont.pixelSize: 24  // 设置图表标题字体大小为 24 像素
        titleFont.bold: true  // 设置图表标题字体为粗体
        plotAreaColor: "yellow"  // 设置绘图区域背景颜色为黄色

        ValuesAxis {
            id: axisX  // 设置 X 轴的 ID 为 axisX
            titleText: "时间 (s)"  // 设置 X 轴的标题文本为 "时间 (s)"
            min: 0  // 设置 X 轴的最小值为 0
            max: 19  // 设置 X 轴的最大值为 19
            tickCount: 20  // 设置 X 轴的刻度数为 20
        }

        ValuesAxis {
            id: axisY  // 设置 Y 轴的 ID 为 axisY
            titleText: "电机速度 (RPM)"  // 设置 Y 轴的标题文本为 "电机速度 (RPM)"
            min: 0  // 设置 Y 轴的最小值为 0
            max: 5000  // 设置 Y 轴的最大值为 5000
            tickCount: 10  // 设置 Y 轴的刻度数为 10
        }

        LineSeries {
            id: lineSeries1  // 设置第一个折线系列的 ID 为 lineSeries1
            name: "电机速度1"  // 设置第一个折线系列的名称为 "电机速度1"
            axisX: axisX  // 将第一个折线系列的 X 轴设置为 axisX
            axisY: axisY  // 将第一个折线系列的 Y 轴设置为 axisY
            color: "blue"  // 设置第一个折线系列的颜色为蓝色
        }

        LineSeries {
            id: lineSeries2  // 设置第二个折线系列的 ID 为 lineSeries2
            name: "电机速度2"  // 设置第二个折线系列的名称为 "电机速度2"
            axisX: axisX  // 将第二个折线系列的 X 轴设置为 axisX
            axisY: axisY  // 将第二个折线系列的 Y 轴设置为 axisY
            color: "red"  // 设置第二个折线系列的颜色为红色
        }
    }

    Column {
        spacing: 10
        anchors {
            bottom: parent.bottom
            horizontalCenter: parent.horizontalCenter
            bottomMargin: 20
        }

        Button {
            text: "Scroll Up"
            onClicked: chartView.scrollUp(50)  // 向上滚动 50 像素
        }

        Button {
            text: "Scroll Down"
            onClicked: chartView.scrollDown(50)  // 向下滚动 50 像素
        }

        Button {
            text: "Scroll Left"
            onClicked: chartView.scrollLeft(50)  // 向左滚动 50 像素
        }

        Button {
            text: "Scroll Right"
            onClicked: chartView.scrollRight(50)  // 向右滚动 50 像素
        }
    }

    Timer {
        interval: 500  // 设置计时器的间隔为 500 毫秒
        running: true  // 设置计时器为运行状态
        repeat: true  // 设置计时器为重复触发
        onTriggered: {
            let xValue = lineSeries1.count;
            let yValue1 = Math.random() * 5000;
            let yValue2 = Math.random() * 5000;
            lineSeries1.append(xValue, yValue1);
            lineSeries2.append(xValue, yValue2);

            if (xValue > axisX.max) {
                axisX.max = xValue + 10;
            }
        }
    }
}

```

代码运行效果：
![[录屏2024-08-06 06.45.14_480p.mov]]

## 3.1、zoomIn()、zoomOut()、zoomReset()
```jsx
import QtQuick 2.15
import QtQuick.Controls 2.15
import QtCharts 2.15
import QtQml 2.15

ApplicationWindow {
    visible: true  // 设置窗口为可见
    width: 1000  // 设置窗口宽度为 1000 像素
    height: 800  // 设置窗口高度为 800 像素
    title: qsTr("电机速度可视化")  // 设置窗口标题为 "电机速度可视化"

    ChartView {
        id: chartView  // 设置 ChartView 的 ID 为 chartView
        anchors.fill: parent  // 将 ChartView 的大小设置为填满父窗口
        antialiasing: true  // 启用抗锯齿以提高图表的绘制质量
        animationDuration: 500  // 设置动画持续时间为 500 毫秒
        animationOptions: ChartView.SeriesAnimations  // 设置动画选项为系列动画
        dropShadowEnabled: true  // 启用阴影效果
        theme: ChartView.ChartThemeBrownSand  // 设置图表主题
        title: "电机速度实时监控"  // 设置图表标题
        titleColor: "darkblue"  // 设置图表标题颜色为深蓝色
        titleFont.pixelSize: 24  // 设置图表标题字体大小为 24 像素
        titleFont.bold: true  // 设置图表标题字体为粗体
        plotAreaColor: "yellow"  // 设置绘图区域背景颜色为黄色

        ValuesAxis {
            id: axisX  // 设置 X 轴的 ID 为 axisX
            titleText: "时间 (s)"  // 设置 X 轴的标题文本为 "时间 (s)"
            min: 0  // 设置 X 轴的最小值为 0
            max: 19  // 设置 X 轴的最大值为 19
            tickCount: 20  // 设置 X 轴的刻度数为 20
        }

        ValuesAxis {
            id: axisY  // 设置 Y 轴的 ID 为 axisY
            titleText: "电机速度 (RPM)"  // 设置 Y 轴的标题文本为 "电机速度 (RPM)"
            min: 0  // 设置 Y 轴的最小值为 0
            max: 5000  // 设置 Y 轴的最大值为 5000
            tickCount: 10  // 设置 Y 轴的刻度数为 10
        }

        LineSeries {
            id: lineSeries1  // 设置第一个折线系列的 ID 为 lineSeries1
            name: "电机速度1"  // 设置第一个折线系列的名称为 "电机速度1"
            axisX: axisX  // 将第一个折线系列的 X 轴设置为 axisX
            axisY: axisY  // 将第一个折线系列的 Y 轴设置为 axisY
            color: "blue"  // 设置第一个折线系列的颜色为蓝色
        }

        LineSeries {
            id: lineSeries2  // 设置第二个折线系列的 ID 为 lineSeries2
            name: "电机速度2"  // 设置第二个折线系列的名称为 "电机速度2"
            axisX: axisX  // 将第二个折线系列的 X 轴设置为 axisX
            axisY: axisY  // 将第二个折线系列的 Y 轴设置为 axisY
            color: "red"  // 设置第二个折线系列的颜色为红色
        }
    }

    Column {
        spacing: 10
        anchors {
            bottom: parent.bottom
            horizontalCenter: parent.horizontalCenter
            bottomMargin: 20
        }

        Button {
            text: "Zoom In"
            onClicked: chartView.zoomIn()  // 放大图表
        }

        Button {
            text: "Zoom Out"
            onClicked: chartView.zoomOut()  // 缩小图表
        }

        Button {
            text: "Zoom In (Rectangle)"
            onClicked: chartView.zoomIn(Qt.rect(100, 100, 200, 200))  // 放大指定矩形区域
        }

        Button {
            text: "Zoom Reset"
            onClicked: chartView.zoomReset()  // 重置图表缩放
        }
    }

    Timer {
        interval: 500  // 设置计时器的间隔为 500 毫秒
        running: true  // 设置计时器为运行状态
        repeat: true  // 设置计时器为重复触发
        onTriggered: {
            let xValue = lineSeries1.count;
            let yValue1 = Math.random() * 5000;
            let yValue2 = Math.random() * 5000;
            lineSeries1.append(xValue, yValue1);
            lineSeries2.append(xValue, yValue2);

            if (xValue > axisX.max) {
                axisX.max = xValue + 10;
            }
        }
    }
}

```

程序的效果：
![[录屏2024-08-06 06.55.22_480p.mov]]

## 3.3、removeAllSeries()、removeSeries()
```jsx
import QtQuick 2.15
import QtQuick.Controls 2.15
import QtCharts 2.15
import QtQml 2.15

ApplicationWindow {
    visible: true  // 设置窗口为可见
    width: 1000  // 设置窗口宽度为 1000 像素
    height: 800  // 设置窗口高度为 800 像素
    title: qsTr("电机速度可视化")  // 设置窗口标题为 "电机速度可视化"

    ChartView {
        id: chartView  // 设置 ChartView 的 ID 为 chartView
        anchors.fill: parent  // 将 ChartView 的大小设置为填满父窗口
        antialiasing: true  // 启用抗锯齿以提高图表的绘制质量
        animationDuration: 500  // 设置动画持续时间为 500 毫秒
        animationOptions: ChartView.SeriesAnimations  // 设置动画选项为系列动画
        dropShadowEnabled: true  // 启用阴影效果
        theme: ChartView.ChartThemeBlueCerulean  // 设置图表主题为蓝色天青主题
        title: "电机速度实时监控"  // 设置图表标题
        titleColor: "darkblue"  // 设置图表标题颜色为深蓝色
        titleFont.pixelSize: 24  // 设置图表标题字体大小为 24 像素
        titleFont.bold: true  // 设置图表标题字体为粗体
        plotAreaColor: "yellow"  // 设置绘图区域背景颜色为黄色

        ValueAxis {
            id: axisX  // 设置 X 轴的 ID 为 axisX
            titleText: "时间 (s)"  // 设置 X 轴的标题文本为 "时间 (s)"
            min: 0  // 设置 X 轴的最小值为 0
            max: 19  // 设置 X 轴的最大值为 19
            tickCount: 20  // 设置 X 轴的刻度数为 20
        }

        ValueAxis {
            id: axisY  // 设置 Y 轴的 ID 为 axisY
            titleText: "电机速度 (RPM)"  // 设置 Y 轴的标题文本为 "电机速度 (RPM)"
            min: 0  // 设置 Y 轴的最小值为 0
            max: 5000  // 设置 Y 轴的最大值为 5000
            tickCount: 10  // 设置 Y 轴的刻度数为 10
        }

        LineSeries {
            id: lineSeries1  // 设置第一个折线系列的 ID 为 lineSeries1
            name: "电机速度1"  // 设置第一个折线系列的名称为 "电机速度1"
            axisX: axisX  // 将第一个折线系列的 X 轴设置为 axisX
            axisY: axisY  // 将第一个折线系列的 Y 轴设置为 axisY
            color: "blue"  // 设置第一个折线系列的颜色为蓝色
        }

        LineSeries {
            id: lineSeries2  // 设置第二个折线系列的 ID 为 lineSeries2
            name: "电机速度2"  // 设置第二个折线系列的名称为 "电机速度2"
            axisX: axisX  // 将第二个折线系列的 X 轴设置为 axisX
            axisY: axisY  // 将第二个折线系列的 Y 轴设置为 axisY
            color: "red"  // 设置第二个折线系列的颜色为红色
        }
    }

    Column {
        spacing: 10
        anchors {
            bottom: parent.bottom
            horizontalCenter: parent.horizontalCenter
            bottomMargin: 20
        }

        Button {
            text: "Zoom In"
            onClicked: chartView.zoomIn()  // 放大图表
        }

        Button {
            text: "Zoom Out"
            onClicked: chartView.zoomOut()  // 缩小图表
        }

        Button {
            text: "Remove All Series"
            onClicked: chartView.removeAllSeries()  // 移除所有系列
        }

        Button {
            text: "Remove Series 1"
            onClicked: chartView.removeSeries(lineSeries1)  // 移除第一个折线系列
        }
    }

    Timer {
        interval: 500  // 设置计时器的间隔为 500 毫秒
        running: true  // 设置计时器为运行状态
        repeat: true  // 设置计时器为重复触发
        onTriggered: {
            let xValue = lineSeries1.count;
            let yValue1 = Math.random() * 5000;
            let yValue2 = Math.random() * 5000;
            lineSeries1.append(xValue, yValue1);
            lineSeries2.append(xValue, yValue2);

            if (xValue > axisX.max) {
                axisX.max = xValue + 10;
            }
        }
    }
}
```

程序效果：
![[录屏2024-08-06 07.01.07_480p.mov]]
## 3.4、setAxisX、setAxisY
这些方法用于为指定的系列设置新的X轴或Y轴。我们可以在已有的基础代码上添加按钮来动态改变系列的轴。

```jsx
import QtQuick 2.15
import QtQuick.Controls 2.15
import QtCharts 2.15
import QtQml 2.15

ApplicationWindow {
    visible: true  // 设置窗口为可见
    width: 1000  // 设置窗口宽度为 1000 像素
    height: 800  // 设置窗口高度为 800 像素
    title: qsTr("电机速度可视化")  // 设置窗口标题为 "电机速度可视化"

    ChartView {
        id: chartView  // 设置 ChartView 的 ID 为 chartView
        anchors.fill: parent  // 将 ChartView 的大小设置为填满父窗口
        antialiasing: true  // 启用抗锯齿以提高图表的绘制质量
        animationDuration: 500  // 设置动画持续时间为 500 毫秒
        animationOptions: ChartView.SeriesAnimations  // 设置动画选项为系列动画
        dropShadowEnabled: true  // 启用阴影效果
        theme: ChartView.ChartThemeBlueCerulean  // 设置图表主题为蓝色天青主题
        title: "电机速度实时监控"  // 设置图表标题
        titleColor: "darkblue"  // 设置图表标题颜色为深蓝色
        titleFont.pixelSize: 24  // 设置图表标题字体大小为 24 像素
        titleFont.bold: true  // 设置图表标题字体为粗体
        plotAreaColor: "yellow"  // 设置绘图区域背景颜色为黄色

        ValueAxis {
            id: axisX  // 设置 X 轴的 ID 为 axisX
            titleText: "时间 (s)"  // 设置 X 轴的标题文本为 "时间 (s)"
            min: 0  // 设置 X 轴的最小值为 0
            max: 19  // 设置 X 轴的最大值为 19
            tickCount: 20  // 设置 X 轴的刻度数为 20
        }

        ValueAxis {
            id: axisY  // 设置 Y 轴的 ID 为 axisY
            titleText: "电机速度 (RPM)"  // 设置 Y 轴的标题文本为 "电机速度 (RPM)"
            min: 0  // 设置 Y 轴的最小值为 0
            max: 5000  // 设置 Y 轴的最大值为 5000
            tickCount: 10  // 设置 Y 轴的刻度数为 10
        }

        ValueAxis {
            id: newAxisX  // 预先定义新的 X 轴
            titleText: "新时间 (s)"
            min: 0
            max: 30
            tickCount: 10
            visible: false  // 初始状态下隐藏
        }

        ValueAxis {
            id: newAxisY  // 预先定义新的 Y 轴
            titleText: "新速度 (RPM)"
            min: 0
            max: 6000
            tickCount: 12
            visible: false  // 初始状态下隐藏
        }

        LineSeries {
            id: lineSeries1  // 设置第一个折线系列的 ID 为 lineSeries1
            name: "电机速度1"  // 设置第一个折线系列的名称为 "电机速度1"
            axisX: axisX  // 将第一个折线系列的 X 轴设置为 axisX
            axisY: axisY  // 将第一个折线系列的 Y 轴设置为 axisY
            color: "blue"  // 设置第一个折线系列的颜色为蓝色
        }

        LineSeries {
            id: lineSeries2  // 设置第二个折线系列的 ID 为 lineSeries2
            name: "电机速度2"  // 设置第二个折线系列的名称为 "电机速度2"
            axisX: axisX  // 将第二个折线系列的 X 轴设置为 axisX
            axisY: axisY  // 将第二个折线系列的 Y 轴设置为 axisY
            color: "red"  // 设置第二个折线系列的颜色为红色
        }
    }

    // 添加按钮以演示 setAxisX 和 setAxisY 的使用
    Column {
        spacing: 10
        anchors {
            bottom: parent.bottom
            horizontalCenter: parent.horizontalCenter
            bottomMargin: 20
        }

        Button {
            text: "Set New X Axis for Series 1"
            onClicked: {
                chartView.setAxisX(newAxisX, lineSeries1)  // 为第一个系列设置新的 X 轴
                newAxisX.visible = true  // 确保新的 X 轴可见
            }
        }

        Button {
            text: "Set New Y Axis for Series 1"
            onClicked: {
                chartView.setAxisY(newAxisY, lineSeries1)  // 为第一个系列设置新的 Y 轴
                newAxisY.visible = true  // 确保新的 Y 轴可见
            }
        }

        Button {
            text: "Reset X Axis for Series 1"
            onClicked: chartView.setAxisX(axisX, lineSeries1)  // 重置第一个系列的 X 轴
        }

        Button {
            text: "Reset Y Axis for Series 1"
            onClicked: chartView.setAxisY(axisY, lineSeries1)  // 重置第一个系列的 Y 轴
        }
    }

    Timer {
        interval: 500  // 设置计时器的间隔为 500 毫秒
        running: true  // 设置计时器为运行状态
        repeat: true  // 设置计时器为重复触发
        onTriggered: {
            let xValue = lineSeries1.count;
            let yValue1 = Math.random() * 5000;
            let yValue2 = Math.random() * 5000;
            lineSeries1.append(xValue, yValue1);
            lineSeries2.append(xValue, yValue2);

            if (xValue > axisX.max) {
                axisX.max = xValue + 10;
            }
        }
    }
}
```

视频效果：
![[录屏2024-08-06 07.14.06_480p.mov]]

## 3.5、mapToPosition、mapToValue
1. **mapToPosition 用途**：
将数据点的坐标（例如时间和电机速度）转换为图表视图中的像素位置。例如，在点击图表时，你可以将数据点 (5, 2500) 转换为图表视图中的像素位置，然后可以在图表上绘制标记或注释。

2. **mapToValue 用途**：
将图表视图中的像素位置转换为数据点的坐标。这对于从用户交互中获取数据点非常有用。例如，当用户点击图表某处时，你可以获取点击位置并转换为对应的数据点，然后进行相应处理。

3. **示例中的使用**：
• 用户点击图表时，MouseArea 捕获点击事件。
• 在点击事件中，我们将一个示例数据点 (5, 2500) 映射到图表视图中的像素位置，并在 Text 组件中显示结果。
• 然后，将映射到的像素位置转换回数据点，并在控制台中输出结果。

```jsx
import QtQuick 2.15
import QtQuick.Controls 2.15
import QtCharts 2.15
import QtQml 2.15

ApplicationWindow {
    visible: true  // 设置窗口为可见
    width: 1000  // 设置窗口宽度为 1000 像素
    height: 800  // 设置窗口高度为 800 像素
    title: qsTr("电机速度可视化")  // 设置窗口标题为 "电机速度可视化"

    ChartView {
        id: chartView  // 设置 ChartView 的 ID 为 chartView
        anchors.fill: parent  // 将 ChartView 的大小设置为填满父窗口
        antialiasing: true  // 启用抗锯齿以提高图表的绘制质量
        animationDuration: 500  // 设置动画持续时间为 500 毫秒
        animationOptions: ChartView.SeriesAnimations  // 设置动画选项为系列动画
        dropShadowEnabled: true  // 启用阴影效果
        theme: ChartView.ChartThemeBlueCerulean  // 设置图表主题为蓝色天青主题
        title: "电机速度实时监控"  // 设置图表标题
        titleColor: "darkblue"  // 设置图表标题颜色为深蓝色
        titleFont.pixelSize: 24  // 设置图表标题字体大小为 24 像素
        titleFont.bold: true  // 设置图表标题字体为粗体
        plotAreaColor: "yellow"  // 设置绘图区域背景颜色为黄色

        ValueAxis {
            id: axisX  // 设置 X 轴的 ID 为 axisX
            titleText: "时间 (s)"  // 设置 X 轴的标题文本为 "时间 (s)"
            min: 0  // 设置 X 轴的最小值为 0
            max: 19  // 设置 X 轴的最大值为 19
            tickCount: 20  // 设置 X 轴的刻度数为 20
        }

        ValueAxis {
            id: axisY  // 设置 Y 轴的 ID 为 axisY
            titleText: "电机速度 (RPM)"  // 设置 Y 轴的标题文本为 "电机速度 (RPM)"
            min: 0  // 设置 Y 轴的最小值为 0
            max: 5000  // 设置 Y 轴的最大值为 5000
            tickCount: 10  // 设置 Y 轴的刻度数为 10
        }

        LineSeries {
            id: lineSeries1  // 设置第一个折线系列的 ID 为 lineSeries1
            name: "电机速度1"  // 设置第一个折线系列的名称为 "电机速度1"
            axisX: axisX  // 将第一个折线系列的 X 轴设置为 axisX
            axisY: axisY  // 将第一个折线系列的 Y 轴设置为 axisY
            color: "blue"  // 设置第一个折线系列的颜色为蓝色
        }

        LineSeries {
            id: lineSeries2  // 设置第二个折线系列的 ID 为 lineSeries2
            name: "电机速度2"  // 设置第二个折线系列的名称为 "电机速度2"
            axisX: axisX  // 将第二个折线系列的 X 轴设置为 axisX
            axisY: axisY  // 将第二个折线系列的 Y 轴设置为 axisY
            color: "red"  // 设置第二个折线系列的颜色为红色
        }

        // 显示映射结果的 Text 组件
        Text {
            id: positionText
            anchors.top: chartView.bottom
            anchors.horizontalCenter: parent.horizontalCenter
            text: "点击图表查看映射"
        }

        // 添加一个鼠标区域来处理点击事件
        MouseArea {
            anchors.fill: parent
            onClicked: {
                var pointInValue = Qt.point(5, 2500);  // 选择一个示例点 (5, 2500)
                var pointInPosition = chartView.mapToPosition(pointInValue, lineSeries1);  // 将数据点映射到图表位置
                positionText.text = "数据点 " + pointInValue + " 映射到位置 " + pointInPosition;

                var position = pointInPosition;
                var valueFromPosition = chartView.mapToValue(position, lineSeries1);  // 将位置映射回数据点
                console.log("位置 " + position + " 映射回数据点 " + valueFromPosition);
            }
        }
    }

    Timer {
        interval: 500  // 设置计时器的间隔为 500 毫秒
        running: true  // 设置计时器为运行状态
        repeat: true  // 设置计时器为重复触发
        onTriggered: {
            let xValue = lineSeries1.count;
            let yValue1 = Math.random() * 5000;
            let yValue2 = Math.random() * 5000;
            lineSeries1.append(xValue, yValue1);
            lineSeries2.append(xValue, yValue2);

            if (xValue > axisX.max) {
                axisX.max = xValue + 10;
            }
        }
    }
}
```
![[Pasted image 20240826193219.png]]