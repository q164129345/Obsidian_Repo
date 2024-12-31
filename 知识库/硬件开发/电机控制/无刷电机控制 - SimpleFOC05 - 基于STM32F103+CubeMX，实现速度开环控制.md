# 导言
---
![[Pasted image 20241231160807.png]]
如上所示，要让三相电机转起来，实现速度开环控制，需要完成FOC框架如下部分：
1. Inverse Park Transform(反帕克变换)
2. SVPWM Generator（SVPWM算法，或者克拉克逆变换）

# 一、CubeMX
---
## 1.1、TIM4
**TIM4的作用是周期性地执行FOC运算，周期暂时设置500us，即2KHz。**
![[Pasted image 20241227164949.png]]

![[Pasted image 20241227165025.png]]


