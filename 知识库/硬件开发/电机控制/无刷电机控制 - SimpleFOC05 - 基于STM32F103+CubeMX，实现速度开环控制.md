# 导言
---
![[Pasted image 20241227142512.png]]
如上所示，要让三相电机转起来，实现速度开环控制，需要完成FOC框架如下部分：
1. Inverse Park Transform(反帕克变换)
2. SVPWM Generator（SVPWM算法，或者克拉克逆变换）
3. Power Inverter（三相逆变电路驱动）



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

## 3.2、TIM4
**TIM4的作用是周期性地执行FOC运算，周期暂时设置500us，即2KHz。**
![[Pasted image 20241227164949.png]]

![[Pasted image 20241227165025.png]]

# 四、代码
---
## 4.1、BLDCDriver3PWM.cpp
