# 导言
---
![[Pasted image 20250123164010.png]]
如上图所示，增加了电流环。

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
- dc_current模式只闭环扭矩分量Iq，Id默认为0（如果设置了相电感，Id也会有相应的值，但不需要加入PID控制）。
- foc_current模式同时闭环扭矩分量Iq与励磁分量Id。

为什么分两种模式？基于现在我对FOC的认知水平。了解到：
1. 当控制的无刷电机不会超过额定转速时，用dc_current模式更合适。此时，只需要控制好扭矩分量Iq即可。
2. 当控制的无刷电机需要超过额定转速时，需要用foc_current。此时，需要励磁分量Id参与控制才能实现稳定的控制。

![[Pasted image 20250123205846.png | 1100]]
如上所示，在BLDCMotor.cpp的函数void BLDCMotor::loopFOC()看到，分别对dc_current模式与foc_current模式的算法区分开。












