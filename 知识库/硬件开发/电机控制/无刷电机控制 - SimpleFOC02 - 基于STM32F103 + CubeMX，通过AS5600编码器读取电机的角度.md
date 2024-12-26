# 导言
---
[[无刷电机控制 - SimpleFOC01 - 基于STM32F103+CubeMX，移植核心的common代码]]
上一个笔记完成common代码的移植，下一步是完成位置编码器的角度读取。
![[Pasted image 20241225203755.png | 500]]
如上所示，AS5600编码器读取云台电机的角度信息。

# 一、Keil
---
![[Pasted image 20241225204308.png | 1000]]
![[Pasted image 20241225204405.png]]

# 二、代码
---
![[Pasted image 20241225204652.png | 1000]]
因为AS5600使用的是I2C通讯接口，跟源码MagneticSensorI2C类很相似，可以直接Copy过来修改。

## 2.1、AS5600_I2C.h

