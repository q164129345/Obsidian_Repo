# 导言
---
![[Pasted image 20241231113230.png]]
在FOC（Field-Oriented Control）框架中，Power Inverter（电源逆变器）扮演着将直流电（DC）转换为交流电（AC）的关键角色。这个转换是必要的，因为电机通常需要交流电来运行，而系统中的电源可能提供的是直流电。
在图中，逆变器接收来自SVPWM生成器的控制信号，并根据这些信号调整其输出，从而驱动PMSM电机按照预定的速度和转矩运行。通过这一过程，逆变器在FOC系统中起到了将控制算法的指令转化为实际电机驱动的桥梁作用。

# 一、电机开发板的三相逆变电路分析
---
![[Pasted image 20241227141718.png]]
如上图所示，simpleFOC的源码提供两种PWM驱动：
1. 3PWM
2. 6PWM

选择哪种驱动，取决于开发板的三相逆变电路的设计。

![[Pasted image 20241227142137.png]]
如上所示，电机驱动板的三相逆变电路只用了3路PWM，还有1路Enable控制，一共4个GPIO。所以，这个电机控制板需要移植的是BLDCDriver3PWM.cpp与.h的源码。

# 二、simpleFOC源码移植
---
## 1.1、BLDCMotor
![[Pasted image 20241227141313.png | 1200]]
## 1.2、BLDCDriver3PWM
![[Pasted image 20241227141437.png | 1200]]

# 三、CubeMX
---
## 3.1、TIM2
**TIM2的目的是产生中间对齐的PWM，并在下半桥导通后的中间位置触发定时器中断，进行电流采样。** 下面，将用示波器抓波形核对一遍。
### 3.1.1、Mode
![[Pasted image 20241227161205.png]]
### 3.1.2、Configuration
![[Pasted image 20241227164217.png]]
如上所示，完成Parameter Settings。为什么这样设置TIM去产生PWM，可以参考CSDN另外一篇详细的笔记：https://blog.csdn.net/wallace89/article/details/144520720?spm=1001.2014.3001.5501
![[Pasted image 20241227163507.png]]
![[Pasted image 20241227163545.png]]

# 四、代码
---
## 4.1、BLDCDriver3PWM.cpp
代码含以下函数：
1. BLDCDriver3PWM::BLDCDriver3PWM()，构造函数
2. BLDCDriver3PWM::enable()，使能三相逆变电路
3. BLDCDriver3PWM::disable()，失能三相逆变电路
4. BLDCDriver3PWM::init()，三相逆变电路初始化
5. BLDCDriver3PWM::setPwm()，根据控制的电压值转换为PWM占空比%
![[Pasted image 20241231152401.png]]
![[Pasted image 20241231152555.png]]
![[Pasted image 20241231152716.png]]
## 4.2、BLDCDriver3PWM.h
![[Pasted image 20241231153516.png]]

## 4.3、bsp_pwm.c
![[Pasted image 20241231153751.png]]
## 4.4、bsp_pwm.h
![[Pasted image 20241231154512.png]]
## 4.5、user_main.cpp
![[Pasted image 20241231155758.png]]
# 五、调试
---
## 5.1、示波器确认PWM波形
![[Pasted image 20241231154911.png]]
![[Pasted image 20241231155604.png]]
如上图所示，示波器测量出来的PWM波形的频率是20KHz，占空比50.2%，没有问题。