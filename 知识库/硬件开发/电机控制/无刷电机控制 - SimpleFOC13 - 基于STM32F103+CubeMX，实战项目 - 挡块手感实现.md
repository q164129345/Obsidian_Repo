# 导言
---
挡块手感效果：
![[20250207-202524.mp4]]
如上所示,实现思路参考灯哥开源: https://www.bilibili.com/video/BV1b64y1P7BK?spm_id_from=333.788.videopod.sections&vd_source=5a3da2d3c2504fa7535978c724745a9e
![[Pasted image 20250207203016.png]]
如上所示, 右侧是红色部分是禁区,摆臂进入禁区后,会弹回来. 当摆臂在左侧时, 电机松轴, 完全没有力量.
**挡块手感的实现本质还是使用FOC的位置闭环控制模式.**  
项目源码: https://github.com/q164129345/MCU_Develop/tree/main/simplefoc13_stm32f103_stopper_control


# 一、代码
---
## 2.1、main.cpp
![[Pasted image 20250207204954.png | 1100]]
![[Pasted image 20250207205028.png | 1100]]




