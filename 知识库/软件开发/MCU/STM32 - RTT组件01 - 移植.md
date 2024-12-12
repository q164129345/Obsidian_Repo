# 导言
---
**RTT**(Real Time Transfer)是一种用于嵌入式中与用户进行交互的技术，它结合了SWO和半主机的优点，具有极高的性能。
使用RTT可以从MCU非常快速输出调试信息和数据，且不影响MCU实时性。这个功能可以用于很多支持J-Link的设备和MCU，兼容性强。
RTT支持两个方向的多个通道，上到主机，下到目标，它可以用于不同的目的，为用户提供尽可能多的自由。默认实现每个方向使用一个通道，用于可打印终端输入和输出。
使用J-Link RTT Viewer，可用于“虚拟”终端，允许打印到多个窗口（例如，一个用于标准输出，一个对于错误输出，一个用于调试输出）。
![[Pasted image 20241022171235.png | 800]]

性能方面绝对无敌！！
RTT的性能明显高于其他任何用于将数据输出到主机PC的方式。平均一行文本可以在1微秒或更短的时间内输出。基本上相当于做一个memcopy()的时间。
RTT实现代码使用大约500字节的ROM和(n(通道数) * (24字节ID+24字节))的RAM。推荐的大小是1 kByte（上行信道）和16到32字节（下行信道），这取决于输入/输出的负载。

开发板+J-Link：
![[Pasted image 20241209161818.png | 800]]
项目源码:https://github.com/q164129345/MCU_Develop/tree/main/jlink_rtt_viewer

# 一、下载
---
官网:https://www.segger.com/downloads/jlink/
![[Pasted image 20241209112021.png]]
按照自己的电脑的系统和芯片类型，选择安装包。比如Intel的Windows11 64位系统，从Windows栏目下，点击下载64-bit Installer。

# 二、安装
---
![[Pasted image 20241209112459.png]]
![[Pasted image 20241209112517.png]]
![[Pasted image 20241209112615.png]]
![[Pasted image 20241209112735.png]]
按照上面的流程，安装J-LINK组件。
![[Pasted image 20241209113127.png]]
安装完毕后，可以找到J-LINK RTT Viewer，证明安装顺利完成了。

# 三、移植
---
## 3.1、找到源码（5个文件）
安装J-LINK驱动后，找到软件的安装目录，我的电脑的目录如下：
![[Pasted image 20241209151634.png | 900]]
![[Pasted image 20241209151748.png | 900]]
如上两张图片所示，一共5个文件都要Copy下来。

## 3.2、将源码放入工程里
![[Pasted image 20241209152107.png]]
![[Pasted image 20241209152139.png]]
至此，移植完成！

# 四、Keil
---
## 4.1、将源码添加到项目
![[Pasted image 20241209152440.png]]
![[Pasted image 20241209152536.png]]
## 4.2、main.h
![[Pasted image 20241209153130.png]]
## 4.3、main.c
![[Pasted image 20241209153310.png]]
![[Pasted image 20241209153433.png]]
# 五、编译、下载
---
![[Pasted image 20241209153547.png]]

# 六、RTT Viewer查看log
---
![[Pasted image 20241209153907.png]]
![[Pasted image 20241209154022.png]]
![[Pasted image 20241209160740.png]]