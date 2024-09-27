# 导言
---
![[Pasted image 20240822210202.png|775]]
使用Qt Quick创建的第一个空白的窗口应用程序时。首先，我们就会碰到Windows类型。要使用Window类型的话，需要import QtQuick.Window 2.14。
![[Pasted image 20240822210248.png|775]]
在帮助文档里，输入Window找到关于它的文档。从文档可以了解到，它一些关于窗口的特性。
# 一、常用特性
---
## 2.1、maximumHeight、maximumWidth
![[Pasted image 20240822210323.png]]
通过它们，可以限制应用程序窗口的最大尺寸。
## 2.2、minimumWidth、minimumHeight
![[Pasted image 20240822210359.png]]
## 2.3、opacity
![[Pasted image 20240822210416.png]]
## 2.4、width，它有一个信号
![[Pasted image 20240822210434.png]]
在QML里， 也有信号与槽的概念。比如width就有一个信号widthChanged()，此时我们可以自己写一个槽函数onWidthChanged()来响应信号widthChanged()。
![[Pasted image 20240822210452.png]]
在QML里，因为有信号widthChanged()，所以能触发对应的槽函数onWidthChanged()。注意⚠️，是在信号名前面增加on，并在on之后的第一个字母改为大写。
![[Pasted image 20240822210503.png]]
如上图所示。
## 2.5、《信号与槽》的又一个例子
![[Pasted image 20240822210518.png]]
我觉得QML到处都是信号与槽。当我创建一个变量myValue时，QML自动创建了该变量对应的信号myValueChanged()与该信号对应的槽函数onMyValueChanged()。
![[Pasted image 20240822210532.png|725]]
大概就是上图的关系了，QML自动化帮我们做了很多事情。
![[Pasted image 20240822210548.png]]
在窗口宽度变化的槽函数里，让myValue++。此时，发信号myValueChanged()触发槽函数onMyValueChanged()。最后，就能捕捉窗口宽度变化的次数。