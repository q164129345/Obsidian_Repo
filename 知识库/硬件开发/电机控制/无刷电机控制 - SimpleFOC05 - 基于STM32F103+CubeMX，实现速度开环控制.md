# 导言
---
![[Pasted image 20241231160807.png]]
如上所示，要让三相电机转起来，实现速度开环控制，需要完成FOC框架如下部分：
1. Inverse Park Transform(反帕克变换)
2. SVPWM Generator（SVPWM算法，或者克拉克逆变换）

效果如下，电机终于开始旋转起来了：
![[20250102-204654.mp4]]

RTT Viewer的打印，虽然是速度开环控制，但速度也接近控制的4rad/s：
![[rtt_open_vel.gif]]
项目源码:https://github.com/q164129345/MCU_Develop/tree/main/simplefoc05_stm32f103_open_vel_ctrl

# 一、CubeMX
---
## 1.1、TIM4
**TIM4的作用是周期性地执行FOC运算，周期暂时设置500us，即2KHz。**
![[Pasted image 20241227164949.png]]

![[Pasted image 20241227165025.png]]

# 二、simpleFOC源码
---
## 2.1、BLDCMotor类
![[Pasted image 20241231180512.png]]
![[Pasted image 20250102205835.png]]
![[Pasted image 20250102210033.png]]
