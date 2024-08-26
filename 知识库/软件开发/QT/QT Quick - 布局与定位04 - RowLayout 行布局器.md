# 一、前言
---
使用布局管理器来组织复杂的布局。
程序的效果如下：
![[录屏 06-28-2024 02_43_00 PM.webm]]

# 二、QML代码
---
```jsx
import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Layouts 1.15
import QtQuick.Controls 2.15

ApplicationWindow {
    visible: true
    width: 640
    height: 480
    title: "Layout Example"

    ColumnLayout {
        anchors.fill: parent
        spacing: 10

        RowLayout {
            Layout.alignment: Qt.AlignHCenter
            spacing: 10

            Rectangle {
                width: 100
                height: 100
                color: "red"
            }

            Rectangle {
                width: 100
                height: 100
                color: "green"
            }

            Rectangle {
                width: 100
                height: 100
                color: "blue"
            }
        }

        RowLayout {
            Layout.alignment: Qt.AlignHCenter
            spacing: 10

            Rectangle {
                width: 100
                height: 100
                color: "yellow"
            }

            Rectangle {
                width: 100
                height: 100
                color: "purple"
            }

            Rectangle {
                width: 100
                height: 100
                color: "orange"
            }
        }
    }
}
```

## 2.1、anchors.fill: parent
`anchors.fill: parent` 是 Qt Quick 中的一种布局方式，用于将一个元素的所有锚点（顶、底、左、右）都锚定到其父元素的对应锚点上，从而使该元素填充其父元素的整个区域。简而言之，它会让子元素完全填充父元素。
使用`anchors.fill: parent`等价于手动设置以下所有锚点：
```jsx
Rectangle {
    color: "red"
    anchors.top: parent.top
    anchors.bottom: parent.bottom
    anchors.left: parent.left
    anchors.right: parent.right
}
```
这样做的效果是一样的，但`anchors.fill: parent`更简洁和直观。

### 示例说明
```jsx
import QtQuick 2.15
import QtQuick.Window 2.15

Window {
    visible: true
    width: 640
    height: 480
    title: "Anchors Fill Example"

    Rectangle {
        width: 640
        height: 480
        color: "lightgray"

        Rectangle {
            color: "red"
            anchors.fill: parent
        }
    }
}
```
解析：
- **父Rectangle**：是一个宽640像素，高480像素，颜色为浅灰色的矩形。
- **子Rectangle**：是一个颜色为红色的矩形，使用`anchors.fill: parent`将其填充到父Rectangle的整个区域。

运行上述代码时，你会看到一个红色的矩形完全覆盖了浅灰色的矩形，这就是`anchors.fill: parent`的作用。它使得子元素的尺寸和位置完全匹配父元素。

## 2.2、Layout.alignment: Qt.AlignHCenter
`Layout.alignment: Qt.AlignHCenter` 是在使用 Qt Quick Layouts 时，用于设置子元素在布局中的水平对齐方式。`Qt.AlignHCenter` 表示水平居中对齐。
使用场景：
1. 动态布局：在动态调整窗口大小时，确保子元素始终保持指定的对齐方式。
2. 复杂界面布局：在复杂的界面布局中，可以精确控制每个子元素的位置和对齐方式。
3. 简化代码：通过使用对齐属性，减少手动计算和调整元素位置的代码量。

### 示例说明
```jsx
import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

ApplicationWindow {
    visible: true
    width: 640
    height: 480
    title: "Layout Alignment Example"

    ColumnLayout {
        anchors.fill: parent
        spacing: 10

        Rectangle {
            width: 100
            height: 100
            color: "red"
            Layout.alignment: Qt.AlignHCenter
        }

        Rectangle {
            width: 200
            height: 100
            color: "green"
            Layout.alignment: Qt.AlignHCenter
        }

        Rectangle {
            width: 150
            height: 100
            color: "blue"
            Layout.alignment: Qt.AlignHCenter
        }
    }
}
```

解析：
- **父元素 `ColumnLayout`**：一个垂直布局，将其所有子元素垂直排列。
- **子元素 `Rectangle`**：三个矩形，分别设置了不同的宽度和高度，并且每个矩形的对齐方式都设置为 `Qt.AlignHCenter`。
通过设置 `Layout.alignment: Qt.AlignHCenter`，这些矩形会在垂直布局中水平居中对齐。也就是说，每个矩形会相对于其父布局（`ColumnLayout`）的水平中心进行对齐。
其他对齐方式：
`Layout.alignment` 支持的对齐方式包括但不限于：
- `Qt.AlignLeft`：左对齐
- `Qt.AlignRight`：右对齐
- `Qt.AlignTop`：顶部对齐
- `Qt.AlignBottom`：底部对齐
- `Qt.AlignVCenter`：垂直居中对齐
- `Qt.AlignCenter`：在水平和垂直方向上同时居中对齐
多对齐方式组合：
可以组合多个对齐方式来实现复杂的对齐需求，例如同时进行水平和垂直居中对齐：

```jsx
Rectangle {
    width: 100
    height: 100
    color: "red"
    Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
}
```

# 三、细节补充
---
## 3.1、两个RowLayout之间距离很远
如下图所示，就算将spaceing调整为0，两个RowLayout之间的距离还是很远。因为spaceing的意义是两个RowLayout的最小距离，并不限制最远距离。
![[Pasted image 20240826191029.png]]
如果想随意调整下面三个正方形离上面三个正方形的距离，使用Row与Column会更好，而不是使用RowLayout与ColumnLayout。
修改的代码如下：
```jsx
import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Controls 2.15

ApplicationWindow {
    visible: true
    width: 640
    height: 480
    title: "Layout Example"

    Column {
        anchors.fill: parent
        spacing: 5  // 可以调整这个值来控制两行之间的间距

        Row {
            spacing: 10
            anchors.horizontalCenter: parent.horizontalCenter

            Rectangle {
                width: 100
                height: 100
                color: "red"
            }

            Rectangle {
                width: 100
                height: 100
                color: "green"
            }

            Rectangle {
                width: 100
                height: 100
                color: "blue"
            }
        }

        Row {
            spacing: 10
            anchors.horizontalCenter: parent.horizontalCenter

            Rectangle {
                width: 100
                height: 100
                color: "yellow"
            }

            Rectangle {
                width: 100
                height: 100
                color: "purple"
            }

            Rectangle {
                width: 100
                height: 100
                color: "orange"
            }
        }
    }
}
```

代码的效果如下：
![[录屏 06-28-2024 04_46_00 PM.webm]]