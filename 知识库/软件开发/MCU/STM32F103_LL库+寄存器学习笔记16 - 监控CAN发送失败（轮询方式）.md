# 导言
---
《[[STM32F103_LL库+寄存器学习笔记15 - 梳理CAN发送失败时，涉及哪些寄存器]]》从上一章节看到，当CAN消息发送失败时，CAN错误状态寄存器ESR的TEC会持续累加，LEC等于0x03（ACK错误）。本次实验的目的是编写一个函数CAN_Check_Error()，在main()的主循环里每隔50ms执行一次，将寄存器CAN_ESR的各个段赋值给全局变量gCanESR，方便应用程序获取寄存器CAN_ESR的各个段的内容。

效果如下：
![[LL16_CAN_ERROR.gif | 1100]]

项目地址：https://github.com/q164129345/MCU_Develop/tree/main/stm32f103_ll_library16_Can_Send_Error

# 一、代码（寄存器）
---
## 1.1、myCanDrive_reg.c
![[Pasted image 20250331160741.png | 1000]]
![[Pasted image 20250331160411.png | 1000]]

## 1.2、myCanDrive_reg.h
![[Pasted image 20250331160622.png | 1000]]
## 1.3、main.c
![[Pasted image 20250408143955.png | 1000]]
如上所示，CAN通讯正常的情况下，条件if是不会成立的。

## 1.4、编译、下载
![[Pasted image 20250408144032.png | 1000]]
编译成功，没有错误。

# 二、细节补充
---
## 2.1、为什么CAN分析仪关闭CAN1通道后，LEC = 0x03(确认ACK错误)？
![[Pasted image 20250331190559.png]]
如上所示，当CAN分析仪关闭CAN1通道后，LEC = 0x03、TEC很快就递增到128。
![[Pasted image 20250331190747.png]]
如上所示，LEC等于二进制011(十进制3)时，表示确认（ACK）错误。简单来说，就是STM32F103将CAN消息发送出去后，没有其他CAN节点回复ACK，最终触发确认ACK错误。

**CAN的ACK机制原理**
- ACK槽（ACK Slot）：每个CAN帧的末尾包含1位ACK槽，发送节点会将其置为隐性（1）。
- 接收节点责任：所有正常工作的接收节点（包括目标节点和其他监听节点）必须在ACK槽位置将其拉为显性（0），表示已正确接收。
- 错误触发条件：若总线上无任何节点响应ACK，发送节点检测到ACK槽仍为隐性（1），则会触发ACK错误（LEC=0x03）。
![[Pasted image 20250331192805.png]]
## 2.2、怎样触发Bus-off离线状态？
![[Pasted image 20250408202231.png]]
我尝试在物理层上制造问题，将CAN总线的CANH与CANL短接就能让STM32F103开发板进入Bus-off离线状态。

## 2.3、进入Bus-off离线状态后，怎样用代码自动恢复？
![[Pasted image 20250408144116.png | 1000]]
![[Pasted image 20250408151054.png | 1000]]
如上所示，当检查到是bus-off错误后，调用函数`CAN_BusOff_Recover()`恢复外设CAN的接收与发送功能。
![[Bus-off.gif | 1100]]
如上所示，当我将物理层的CANH、CANL短接后，CAN_ESR寄存器的TEC快速暴增至254，且LEC=5（显性位错误），最终触发Bus-off离线状态。接着运行函数`CAN_BusOff_Recover()`恢复正常工作，顺利发送与接收CAN报文。
