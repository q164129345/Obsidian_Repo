# 导言
---
![[Pasted image 20250123164010.png]]
如上图所示, 增加了电流环. **至此, SimpleFOC整个框架都移植完成, 后续只需要根据自己项目的硬件改变驱动代码即可.**

效果如下：
![[20250123-200906.mp4]]

**RTT**
![[RTT_J-Scope 1.gif | 1100]]
如上图所示，三相占空比依然是马鞍波。当我用手去给电机施加阻力时，PID要维持目标转速，将马鞍波变大，提高扭矩抵抗阻力。
项目源码：https://github.com/q164129345/MCU_Develop/tree/main/simplefoc10_stm32f103_vel_close_with_current

## 一、代码
---
## 1.1、user_main.cpp
![[Pasted image 20250123202813.png | 1100]]
![[Pasted image 20250123203116.png | 1100]]
![[Pasted image 20250123203327.png | 1100]]

# 二、细节补充
---
## 2.1、电流环的两种控制模式dc_current与foc_current
![[Pasted image 20250123203915.png | 1100]]
- dc_current模式只闭环扭矩分量Iq, Id默认为0(如果设置了相电感, Id也会有相应的值, 但不需要加入PID控制).
- foc_current模式同时闭环扭矩分量Iq与励磁分量Id.

为什么分两种模式？基于现在我对FOC的认知水平.了解到：
1. 当控制的无刷电机不会超过`额定转速`时, 用dc_current模式更合适. 此时, 只需要控制好扭矩分量Iq即可.
2. 当控制的无刷电机需要超过`额定转速`时, 需要用foc_current. 此时, 需要励磁分量Id参与控制才能实现稳定的控制.

![[Pasted image 20250123205846.png | 1100]]
如上所示, 在BLDCMotor.cpp的函数void BLDCMotor::loopFOC()看到, 分别对dc_current模式与foc_current模式的算法区分开.

## 2.2、STM32F103完成一次FOC速度闭环控制一共需要多长时间？
**STM32F103完成一次FOC速度闭环控制需要约431us.** 运行一次`loopFOC()`方法与`move()`方法. STM32F103属于Cortex-M3架构，没有硬件浮点运算器，所以执行浮点运算的效率很差。相比之下，STM32F407之类的Cortex-M4架构的MCU，有硬件浮点运算器，执行浮点运算的效率就比较高。后续，会更新STM32F407的SimpleFOC的实验。
![[Pasted image 20250124175415.png | 1100]]
![[Pasted image 20250124175518.png | 1100]]
如上所示, 通过示波器测量出来, 一次FOC速度闭环控制约431us. 


