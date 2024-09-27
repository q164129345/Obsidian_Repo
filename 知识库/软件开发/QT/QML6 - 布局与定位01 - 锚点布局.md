# 一、前言
---
使用锚点将元素定位到窗口的不同位置。
程序运行的效果如下：
![[Pasted image 20240826190327.png|800]]
# 二、QML代码
---
```jsx
import QtQuick 2.15
import QtQuick.Window 2.15

Window {
    visible: true
    width: 640
    height: 480
    title: "Anchor Layout Example"

    Rectangle {
        width: 640
        height: 480
        color: "lightgray"

        Rectangle {
            width: 100
            height: 100
            color: "red"
            // 锚点 , parent指的是上一层的Rectangle
            anchors.top: parent.top 
            anchors.left: parent.left
        }

        Rectangle {
            width: 100
            height: 100
            color: "green"
            // 锚点 , parent指的是上一层的Rectangle
            anchors.top: parent.top
            anchors.right: parent.right
        }

        Rectangle {
            width: 100
            height: 100
            color: "blue"
            // 锚点 , parent指的是上一层的Rectangle
            anchors.bottom: parent.bottom
            anchors.left: parent.left
        }

        Rectangle {
            width: 100
            height: 100
            color: "yellow"
            // 锚点 , parent指的是上一层的Rectangle
            anchors.bottom: parent.bottom
            anchors.right: parent.right
        }
    }
}

```

# 三、QML Type

---
## 3.1、Rectangle上找不到anchors属性
如下图所示，Rectangle的属性并没有找到anchors。
![[Pasted image 20240826190420.png]]
但是，Rectangle继承自Item类。
![[Pasted image 20240826190435.png]]
进入Item类的介绍，终于找到anchors了。
![[Pasted image 20240826190446.png]]