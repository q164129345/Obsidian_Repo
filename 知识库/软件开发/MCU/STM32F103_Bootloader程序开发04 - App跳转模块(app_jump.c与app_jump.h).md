# 导言
---
![[Pasted image 20250525202632.png]]
上一章节《[[STM32F103_Bootloader程序开发03 - 启动入口与升级模式判断(boot_entry.c与boot_entry.h)]]》学会使用"C/C++的构造函数（constructor）机制"让我们自己编写的函数`_SystemStart()`在main()之前先运行。

**“无须deinit”是现代bootloader设计的推荐实践。** 本章节将结合上一章节的代码，梳理“无须deinit"的实现过程。
![[Pasted image 20250527110136.png]]
如上所示，摘自bootloader开源项目[mOTA](https://gitee.com/DinoHaw/mOTA)。

项目地址：
github: 
gitee(国内): 

# 一、Keil设置Flash与RAM
---
## 1.1、App程序
![[Pasted image 20250527105242.png]]
Flash区域按照《[[STM32F103_Bootloader程序开发02 - Bootloader程序架构与STM32F103的Flash内存规划]]》的规划进行设置，如上所示。
RAM分成两部分：
- IRAM1给程序使用。
- IRAM2的大小刚好是8个字节，这里定义了一个全局变量`update_flag`，记得勾选`No Init`。

## 1.2、bootloader程序
![[Pasted image 20250527105959.png]]
Flash区域按照《[[STM32F103_Bootloader程序开发02 - Bootloader程序架构与STM32F103的Flash内存规划]]》的规划进行设置，如上所示。
RAM分成两部分：
- IRAM1给程序使用。
- IRAM2的大小刚好是8个字节，这里定义了一个全局变量update_flag，记得勾选`No Init`。























