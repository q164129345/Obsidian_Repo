# 导言
---
![[Pasted image 20240822210822.png]]
Rectangle控件很常用，算是简单的。然而，关于它的内容也不少。
# 一、Rectangle
---
## 1.1、创建一个Rectangle控件
![[Pasted image 20240822210910.png]]
如上图所示，创建一个Rectangle控件：
```jsx
// 矩形
Rectangle {
    x : 100
    y : 100
    z : 1 // 如果一个控件想站在另外一个控件的上面，需要使用z轴坐标
    width: 80
    height: 50
    color: "blue"
    //focus: true // 焦点，它是一个重点噢
}
```

尝试将坐标备注掉，它依然能创建，只是这个Rectangle对象被移动到左上角而已（应该QML默认给了一个起始坐标(0,0)。
关于color(颜色）的选择，从帮助文档可以找到，里面有相对丰富的配色方案。
![[Pasted image 20240822210933.png]]
从帮助文档，找到页面color QML Basic…Qt Quick 5.14.2就能找到color类型。接着，往下移动寻找丰富的配色方案。
![[Pasted image 20240822210953.png]]
如上图所示，可以找到相当丰富的配色选项。
## 1.2、Rectangle响应鼠标左键
![[Pasted image 20240822211014.png]]
如上所示，在Rectangle里创建一个anchors（锚点），fill的意思是铺满。
```jsx
// 捕捉鼠标位置(它跟focus是不是true，没有关系）
MouseArea {
    // 锚点填充满父类（这里的父类指的是Rectangle）
    anchors.fill: parent
    // 当矩形被鼠标左键点击时
    onClicked: {
        console.log("on clicked.")
	  }
}
```
运行代码，当使用鼠标左键点击蓝色矩形时，下面会弹出log。此时，证明槽函数`onClicked`被触发了。根据QML的习惯，触发槽函数`onClicked`的是信号`clicked`。
![[Pasted image 20240822211045.png]]
当我们输入onc（输入on还不能触发代码提示功能），触发代码提示功能时，看到还有很多很多有意思的槽函数可以使用。比如，`onDoubleClicked`。
## 1.3、Rectangle响应键盘的回车键
![[Pasted image 20240822211108.png]]
如上图所示，当键盘回车按下时，槽函数`Keys.onReturnPressed`被触发。
```jsx
// 矩形
Rectangle {
    x : 100
    y : 100
    z : 1 // 如果一个控件想站在另外一个控件的上面，需要使用z轴坐标
    width: 80
    height: 50
    color: "blue"
    focus: true //开启焦点，它是一个重点噢

    // 捕捉鼠标位置(它跟focus是不是true，没有关系）
    MouseArea{
        // 锚点填充满父类（这里的父类指的是Rectangle）
        anchors.fill: parent
        // 当矩形被鼠标左键点击时
        onClicked: {
            console.log("on clicked.")
        }
    }

    // 键盘的回车被按下（此时，Rectangle必须开启focus，否则Rectangle无法被键盘捕捉到）
    Keys.onReturnPressed: {
        console.log("onReturn pressed.")
    }
}
```
重点来了，如果Rectangle不使能focus的话，Rectangle控件将无法被键盘选中。所以，要让槽函数`Keys.onReturnPressed`能被触发，Rectangle控件必须使能focus属性。