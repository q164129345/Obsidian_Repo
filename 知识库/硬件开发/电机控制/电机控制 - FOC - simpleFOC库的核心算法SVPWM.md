# 导言
---
simpleFOC的源码地址：https://github.com/simplefoc/Arduino-FOC
![[Pasted image 20241204152929.png | 1000]]
在学习simpleFOC源码的过程中，不小心发现simpleFOC的核心代码SVPWM算法的实现居然改变了。为此，简单记录一下。
如上图所示，在simpleFOC的v2.3.2版本上有一个很重要的更新，核心代码SVPWM算法的实现更新了。更新的原因来自：https://community.simplefoc.com/t/embedded-world-2023-stm32-cordic-co-processor/3107/164?u=candas1
![[Peek 2024-12-04 15-49.gif]]
如上视频看到，新的SVPWM算法确实能很好地实现SVPWM（马鞍波），关键是代码更加简洁了。


# 一、源码
---
## 1.1、V2.3.1及以下版本
![[Pasted image 20241204154419.png | 1000]]
![[Pasted image 20241204154457.png | 1000]]
如上两张图，simpleFOCV2.3.1及以下版本，BLDCMotor.cpp文件的方法``void BLDCMotor::setPhaseVoltage(float Uq, float Ud, float angle_el)``里的SVPWM算法的实现，从switch(sector)看到在6个扇区里计算Ta、Tb、Tc。

## 1.2、V2.3.2及以上版本
![[Pasted image 20241204153927.png | 1000]]
如上图所示，simpleFOCV2.3.2及以上版本，BLDCMotor.cpp文件的方法``void BLDCMotor::setPhaseVoltage(float Uq, float Ud, float angle_el)``里SVPWM算法的实现。从代码的长度看来，代码真的很简洁。






