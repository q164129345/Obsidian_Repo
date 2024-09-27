## 一、前言
---
![[录屏 07-02-2024 03_40_31 PM.webm]]
通过contentItem属性，插入新的输入控件TextField。使得Dialog控件可以接收用户输入的字符串内容。

## 二、QML代码
---
```javascript
import QtQuick 2.15  // 导入Qt Quick模块
import QtQuick.Controls 2.15  // 导入Qt Quick控件模块

ApplicationWindow {  // 创建应用窗口
    visible: true  // 设置窗口可见
    width: 640  // 设置窗口宽度
    height: 480  // 设置窗口高度

    Button {  // 创建按钮控件
        text: "Show Input Dialog"  // 设置按钮上的文本
        anchors.centerIn: parent  // 将按钮锚定在父级中间
        onClicked: inputDialog.open()  // 当按钮被点击时，打开输入对话框
    }

    Dialog {  // 创建对话框控件
        id: inputDialog  // 设置对话框的ID
        title: "Input Dialog"  // 设置对话框的标题
        modal: true  // 设置对话框为模态，阻止与其他窗口的交互

        contentItem: Column {  // 设置对话框的内容为列布局
            spacing: 20  // 设置控件之间的间距
            width: parent.width  // 设置内容宽度为父级宽度
            padding: 20  // 设置内容的内边距

            TextField {  // 创建文本输入控件
                id: inputField  // 设置文本输入框的ID
                placeholderText: "Enter something..."  // 设置占位文本
                width: parent.width - 40  // 设置输入框宽度
            }

            Row {  // 创建行布局
                spacing: 20  // 设置控件之间的间距

                Button {  // 创建按钮控件
                    text: "OK"  // 设置按钮上的文本
                    onClicked: {  // 当按钮被点击时执行的操作
                        console.log("User input:", inputField.text)  // 打印用户输入的内容
                        inputDialog.close()  // 关闭对话框
                    }
                }

                Button {  // 创建按钮控件
                    text: "Cancel"  // 设置按钮上的文本
                    onClicked: inputDialog.close()  // 当按钮被点击时，关闭对话框
                }
            }
        }
    }
}
```





