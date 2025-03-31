# 导言
---
《[[STM32F103_LL库+寄存器学习笔记14 - CAN发送完成中断]]》上一章节完成CAN发送完成中断，在梳理二级发送缓存之前，先梳理怎样监控CAN发送失败。

![[LL15_CAN_ERROR.gif]]

如上所示：
1. 当我关掉CAN分析仪的CAN通道1，CAN错误状态寄存器CAN_ESR的TEC（发送错误计数器）持续递增，且最终递增至最大值0x80(十进制128)。此时，EPVF（错误被动标志）与EWGF（错误警告标志）都被置1。
2. 当我关掉CAN分析仪的CAN通道1，CAN错误状态寄存器CAN_ESR的TEC（发送错误计数器）开始递增时，LEC（上次错误代码）等于0x03（ACK错误）。
3. 当我重启开启CAN分析仪的CAN通道1，CAN错误状态寄存器CAN_ESR的TEC（发送错误计数器）持续递减，且最终递减至0。此时，EPVF（错误被动标志）与EWGF（错误警告标志）与LEC（上次错误代码）都被置0。

项目地址：
- 寄存器方式：https://github.com/q164129345/MCU_Develop/tree/main/stm32f103_ll_library14_Can_Send_Interrupt
- HAL库方式：https://github.com/q164129345/MCU_Develop/tree/main/stm32f103_hal_library14_Can_Send_Interrupt

# 一、梳理寄存器
---
## 1.1、CAN错误状态寄存器CAN_ESR
![[Pasted image 20250330204515.png]]
![[Pasted image 20250330204537.png]]
如上所示：
1. 跟CAN发送相关的是TEC（9位发送错误计数器）。正如上面的gif动图所示，发送失败一次，TEC会加1。发送成功一次，TEC会减1。
2. LEC（上次错误代码）、EPVF（错误被动标志）、EWGF（错误警告标志）跟CAN接收与CAN发送都有关系。
