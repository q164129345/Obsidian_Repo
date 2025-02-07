# 导言
---
棘轮手感的效果（双向）：
![[20250207-170113.mp4]]
参考灯哥开源的B站视频(`原理的讲解，务必多看几遍`)：
https://www.bilibili.com/video/BV1VW4y1F7Rj/?spm_id_from=333.1391.0.0&vd_source=5a3da2d3c2504fa7535978c724745a9e

**棘轮手感的实现本质是使用FOC的位置闭环控制模式.** 相当于把原本连续的角度轨迹离散化, 从而使得电机只能停在固定的角度上. 本次实验离散化的角度数量是8个, 分别是A0～A7.

项目源码: https://github.com/q164129345/MCU_Develop/tree/main/simplefoc12_stm32f103_Ratchet_Control

# 一、原理
---
![[Pasted image 20250207171506.png | 1100]]
![[Pasted image 20250207173757.png | 800]]如上所示, 将一个圆形平均八等分, 每一个分度位置之间相差45度, B点与相邻的分度位置相差22.5度. 

![[Pasted image 20250207170849.png || 1100]]
当旋钮电机位置在(A0,B)之间运动时, 位置闭环的targetAngle = A0.

![[Pasted image 20250207171018.png | 1100]]
当旋钮电机位置在(B,A1)之间运动时, 位置闭环的targetAngle = A1. 反方向也是一样的.

# 二、代码实现
---
## 2.1、main.cpp
![[Pasted image 20250207175857.png | 1100]]
![[Pasted image 20250207175956.png | 1100]]

# 三、细节补充
---
## 3.1、阻尼手感调节
通过调节电流环的PID最大值, 可以实现阻尼效果的调节. 关于阻尼的调节,每一个电机效果不一样, PID参数的调整需要多尝试.
![[Pasted image 20250207180426.png]]