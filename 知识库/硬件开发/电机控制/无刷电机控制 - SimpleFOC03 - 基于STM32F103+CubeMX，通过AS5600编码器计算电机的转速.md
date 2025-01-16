# 导言
---
[[无刷电机控制 - SimpleFOC02 - 基于STM32F103+CubeMX，通过AS5600编码器读取电机的角度]] ，上一章节完成了电机角度的读取，接下来试一下读取电机的转速。
![[Pasted image 20241226202910.png | 1100]]
如上图所示，本章节用到`update()`方法与`getVelocity()`方法。

![[AS5600_velocity.gif | 1100]]
如上所示，左侧是RTT_Viewer，右侧是Jscope的数据曲线化。波形不平滑主要是因为电机是被我的手转动的，转速会波动比较大。

![[Pasted image 20241226205355.png | 1100]]
如上图所示，通过上一章节与这一章节的实验，终于完成了FOC框图的上述模块Position Sensor Signal Processing。

项目源码: https://github.com/q164129345/MCU_Develop/tree/main/simplefoc03_stm32f103_velocity
## 一、代码
---
## 1.1、user_main.cpp
![[Pasted image 20241226202406.png | 1100]]
如上所示，基于上一章节的代码，获取电机的转速非常简单。

