# 导言
---
simpleFOC源码分为如下5个部分，其中communication是跟simpleFOC上位机通讯，暂时不打算使用，忽略它：
1. common(数学工具、关键类)
2. communication（simpleFOC上位机）
3. current_sensor（电流传感器）
4. drivers（驱动器，三相逆变电路）
5. sensors（位置编码器）

先从common开始移植，编译，调试。

# 一、移植
---
## 1.1、Arduino-FOC/src/common
![[Pasted image 20241225194310.png]]
如上所示，simpleFOC源码一共包含上述这些模块。

![[Pasted image 20241225112419.png | 1000]]
如上所示，将simplefoc源码的src/common的代码移植到stm32f103项目上。

## 1.2、SEGGER_RTT
参考博文：[SEGGER | 基于STM32F405 + Keil - RTT组件01 - 移植SEGGER RTT](https://blog.csdn.net/wallace89/article/details/144478238?spm=1001.2014.3001.5502)
[[STM32 - RTT组件01 - 移植]]
用于调试log打印，也可以用于替代Arduino库的print()。

## 1.3、DWT计时器
![[Pasted image 20241225142321.png | 900]]
如上图所示，dwt_timer.c是我自己编写的DWT定时器驱动代码，目的：
1. 提供us级的延时（替代Arduino提供的delayMicroseconds()）
2. 提供us级的时间戳（完成time_utils.cpp里的_micro()）

## 1.4、C++环境
![[Pasted image 20241225172348.png]]
simpleFOC使用C++语言编写，然后，.c代码不能调用.cpp代码里的函数。创建user_main.cpp与user_main.h的目的是实现C++环境的跳转。
有空的小伙伴可以试试，在main.c里调用time_tuils.cpp的_micros()函数试试，会出现编译错误。其原因就是.c代码不能调用.cpp的代码。

# 二、Keil
---
![[Pasted image 20241225115339.png | 1000]]
![[Pasted image 20241225115426.png]]
# 三、代码
---
## 3.1、main.h
![[Pasted image 20241225153237.png]]
## 3.2、main.c
![[Pasted image 20241225172132.png]]
## 3.3、user_main.cpp
![[Pasted image 20241225190321.png]]
代替main.c的while(1)死循环。
## 3.4、user_main.h
![[Pasted image 20241225175718.png]]
## 3.5、foc_utils.h
![[Pasted image 20241225133932.png | 900]]

## 3.6、foc_utils.cpp
![[Pasted image 20241225150723.png | 900]]


## 3.7、FOCMotor.h
![[Pasted image 20241225141022.png | 900]]
![[Pasted image 20241225151331.png | 900]]
![[Pasted image 20241225151758.png | 900]]
![[Pasted image 20241225152029.png | 900]]
## 3.8、FOCMotor.cpp
![[Pasted image 20241225144425.png | 900]]
![[Pasted image 20241225152412.png | 900]]
![[Pasted image 20241225151501.png | 900]]
![[Pasted image 20241225152302.png | 900]]

## 3.9、BLDCDriver.h
![[Pasted image 20241225141135.png | 900]]
## 3.10、Sensor.h
![[Pasted image 20241225151650.png | 900]]

## 3.11、time_utils.cpp
![[Pasted image 20241225174314.png]]


# 四、调试代码
---
到目前为止，暂时只能调试time_utils.cpp的功能，time_utils.cpp为simpleFOC提供精确的时间戳与时间延时，非常重要。

## 4.1、编译
![[Pasted image 20241225153337.png]]
通过第三章节的处理，解决所有代码的报错，最终编译成功。

## 4.2、使用RTT Viwer调试
![[simplefoc_dwt.gif]]
![[Pasted image 20241225165232.png]]




