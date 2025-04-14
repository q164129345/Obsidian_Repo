# 导言
---
《[[STM32F103_HAL库+寄存器学习笔记14 - CAN发送完成中断]]》上一章节完成CAN发送完成中断，在梳理二级发送缓存之前，先梳理怎样监控CAN发送失败。

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
1. 跟CAN发送相关的是TEC（9位发送错误计数器）。正如上面的gif动图所示，发送失败一次，TEC会快速+8。发送成功一次，TEC会减1。
2. LEC（上次错误代码）、EPVF（错误被动标志）、EWGF（错误警告标志）跟CAN接收与CAN发送都有关系。

## 1.2、TEC递增至128，不再递增
![[Pasted image 20250331192805.png]]
如上所示，让CAN分析仪关闭，总线只有STM32F103一个CAN节点。此时，STM32F103往CAN总线上发送CAN消息时，会触发确认ACK错误（TEC=0x03）。因为没有另外的CAN节点回复ACK。
![[Pasted image 20250402102532.png]]
如上所示，因为确认ACK错误（TEC = 0x03）导致的发送失败，TEC递增至十六进制的0x80(十进制128)。此时，不会再递增了。

## 1.3、TEC会在什么情况下突破128，递增至200多？触发Bus-off离线？
![[Pasted image 20250402112322.png| 800]]
![[CAN_ERROR_TEST.gif]]
如上所示，当我用杜邦线将CANH、CANL短接后，TEC很快递增至0xF8（十进制248）。此时，寄存器CAN_ESR的BOFF（Bus-off离线标志）置1。CAN分析仪不再收到STM32F103开发板发出来的CAN消息，因为STM32F103开发板进入了离线模式。因为自身的问题（CANH与CANL出现了短路），不能干扰CAN总线，以免影响到其他CAN节点通讯。

## 1.4、触发Bus-off离线后，怎样恢复？
![[Pasted image 20250402134107.png]]
如上所示，从《STM32F1中文参考手册》的22.7.6章节看到，在进入Bus-off状态后，可以在自动或者在软件的请求下，从离线状态恢复。在下一章节，我会尝试用代码方式，解除离线状态恢复至正常工作状态。
