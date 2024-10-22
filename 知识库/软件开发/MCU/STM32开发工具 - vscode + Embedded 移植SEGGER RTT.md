# 导言
---
**RTT**(Real Time Transfer)是一种用于嵌入式中与用户进行交互的技术，它结合了SWO和半主机的优点，具有极高的性能。
使用RTT可以从MCU非常快速输出调试信息和数据，且不影响MCU实时性。这个功能可以用于很多支持J-Link的设备和MCU，兼容性强。
RTT支持两个方向的多个通道，上到主机，下到目标，它可以用于不同的目的，为用户提供尽可能多的自由。默认实现每个方向使用一个通道，用于可打印终端输入和输出。
使用J-Link RTT Viewer，可用于“虚拟”终端，允许打印到多个窗口（例如，一个用于标准输出，一个对于错误输出，一个用于调试输出）。
![[Pasted image 20241022171235.png]]

性能方面绝对无敌！！
RTT的性能明显高于其他任何用于将数据输出到主机PC的方式。平均一行文本可以在1微秒或更短的时间内输出。基本上相当于做一个memcopy()的时间。
RTT实现代码使用大约500字节的ROM和(n(通道数) * (24字节ID+24字节))的RAM。推荐的大小是1 kByte（上行信道）和16到32字节（下行信道），这取决于输入/输出的负载。
![[Pasted image 20241022171856.png]]

# 一、开始移植
---
## 1.1、官网下载、安装J-LINK软件
官网下载J-LINK的驱动程序:https://www.segger.com/downloads/jlink/JLink_Windows.exe
![[Pasted image 20241022172046.png | 800]]

## 1.2、找到RTT源码
安装J-LINK驱动后，找到软件的安装目录，我的电脑的目录如下：
![[Pasted image 20241022172250.png]]
![[Pasted image 20241022172658.png]]

首先，进入RTT文件夹，看到4个源码，它们都有用。
![[Pasted image 20241022172804.png]]

还有config文件夹里的`SEGGER_RTT_Conf.h`。
![[Pasted image 20241022173333.png]]

## 1.3、复制RTT源码到项目
如下图所示，一共5个文件复制到项目的SEGGER-RTT文件夹。
![[Pasted image 20241022173623.png]]

![[Pasted image 20241022173711.png]]

## 1.4、EIDE项目添加源码
![[Pasted image 20241022173837.png]]

# 二、代码
---
## 2.1、main.h
包含SEGGER_RTT.h头文件
![[Pasted image 20241022174045.png]]

## 2.2、main.c
调用SEGGER_RTT_Init()初始化RTT。
![[Pasted image 20241022174230.png]]

## 2.3、打印log
在想要打印log的位置，调用SEGGER_RTT_printf()打印log。
![[Pasted image 20241022174529.png]]

编译成功，下载代码。
![[Pasted image 20241022174618.png]]

# 三、J-LINK RTT Viewer
---

![[Pasted image 20241022174810.png]]









