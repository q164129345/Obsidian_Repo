# 导言
---
![[Pasted image 20250116195502.png | 1100]]
**FOC的精髓是电流环，但并不是没有电流环就没办法用FOC。** 如上所示，simpleFOC在没有电流环前提下的速度闭环控制框架。此时，闭环的关键是电机的编码器。其效果如下：
![[20250116-205844.mp4]]

**RTT Viewer与J-Scope**
![[RTT.gif | 1100]]

**所谓速度闭环，就是目标速度与反馈速度非常接近，但不可能完全重合**。如上图所示，红色曲线是目标速度，蓝色曲线是反馈速度，两根曲线基本重合。当我用手握住电机时，蓝色曲线的反馈速度接近0。当我松开手后，蓝色曲线的反馈速度又跟红色曲线的目标速度重合。

项目源码：https://github.com/q164129345/MCU_Develop/tree/main/simplefoc08_stm32f103_velocity_close

# 一、代码
---
## 1.1、user_main.cpp
![[Pasted image 20250116210758.png | 1100]]
![[Pasted image 20250116210918.png | 1100]]

# 二、细节补充
---
## 2.1、使用J-Scope观察Ua、Ub、Uc的波形
![[Pasted image 20250117150103.png | 1100]]

![[J-Scope.gif]]
如上图所示，使用J-Scope读取三相占空比dc_a、dc_b、dc_c的波形。

