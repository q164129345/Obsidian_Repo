## 导言
---
![[Pasted image 20240904205246.png]]
![[Pasted image 20240904205305.png]]
根据官方的介绍，通用定时器可以当作一个挂钟使用。如何启动它，并在程序的任何时候获取当前的计数值。这可以用作程序内部的时间基准或实现类似挂钟功能的计时器。
![[Pasted image 20240904205319.png]]
程序效果如上图所示，大概第6S开始启动挂钟，并打印挂钟的当前时间。
源码地址：[https://github.com/q164129345/esp32_Learning/tree/main/code_SysTimer](https://github.com/q164129345/esp32_Learning/tree/main/code_SysTimer)

## 一、wall_clock.c
---
![[Pasted image 20240904205333.png]]
## 二、main.c
---
![[Pasted image 20240904205348.png]]