# 一、前言
---
在网络上找到一个介绍StackLayout的例子，我觉得很不错。通过点击上面的“首页”button与“联系”button切换不同的StackLayout页面，显示不同的内容。
![[录屏 07-01-2024 11_48_16 AM.webm]]

# 二、QML代码
---
```jsx
import QtQuick
import QtQuick.Layouts
import QtQuick.Window
import QtQuick.Controls

Window {
    id: root
    visible: true
    width: 480
    height: 600
    title: qsTr("Hello World")

    ColumnLayout
    {
        anchors.fill: parent
        RowLayout
        {
            Layout.alignment: Qt.AlignHCenter
            Button{
                text: "首页"
                Layout.alignment: Qt.AlignHCenter

                onClicked:
                {
                    stackWig.currentIndex = 0;
                }
            }
            Button{
                text: "联系"
                Layout.alignment: Qt.AlignHCenter

                onClicked:
                {
                    stackWig.currentIndex = 1;
                }
            }
        }
        StackLayout
        {
            id:stackWig
            currentIndex: 0
            Rectangle
            {
                color: "#00B000"
                Text {
                    id: homePage
                    text: qsTr("首页")
                    anchors.verticalCenter: parent.verticalCenter
                    anchors.horizontalCenter: parent.horizontalCenter
                    verticalAlignment: Text.AlignVCenter
                    horizontalAlignment: Qt.AlignHCenter
                }
            }
            Rectangle
            {
                id: rectangle
                color: "steelblue"
                Text {
                    id: contactPage
                    text: qsTr("联系")
                    anchors.verticalCenter: parent.verticalCenter
                    anchors.horizontalCenter: parent.horizontalCenter
                    verticalAlignment: Text.AlignVCenter
                    horizontalAlignment: Qt.AlignHCenter
                }
            }
        }

    }
}

```

## 2.1、属性Properties
---
![[Pasted image 20240826191345.png]]
StackLayout的一般属性只有两个，count是只读的，currentIndex用于控制切换stackLayout页面。

# 三、细节补充
---
## 3.1、再增加一个按键与对应的页面
这个框架建立一个新的页面很简单，只需要增加一个Button与对应的一个Rectangle即可。
![[Pasted image 20240826191417.png]]
![[Pasted image 20240826191439.png]]
运行的效果：
![[录屏 07-01-2024 02_04_47 PM.webm]]
