# 导言
---
通过采用 CAN 发送完成中断，你可以让 CPU 从忙等待中解放出来，专注于其他任务，从而实现更高的效率和更好的系统响应。中断能让系统在处理多个任务时保持较好的调度和响应，适合复杂实时系统的设计。**总之，使用CAN发送完成中断，可以解放CPU，避免死等的情况。**

STM32CubeMX的CAN代码不支持LL库，所以只能用HAL库，或者自己撸寄存器（最高效、最底层的方式）。

项目地址：


## CAN发送中断
![[Pasted image 20250327191507.png]]
如上图所示，触发CAN发送中断的条件是发送邮箱0～2中任一个的消息发送出去，发送邮箱变成空。

# 一、CubeMX
---
![[Pasted image 20250321085420.png]]
如上所示，勾选CAN TX Interrupts全局中断。另外，其他设置跟上一章节一样[[STM32F103_LL库+寄存器学习笔记13 - 梳理CAN外设与发送报文]]。

# 二、代码（HAL库）
---
## 2.1、can.c
![[Pasted image 20250321085902.png | 800]]
如上所示，勾选CAN TX Interrupts全局中断后，can.c的函数`HAL_CAN_MspInit()`里增加了优先级的设置与全局中断的开启。
## 2.2、stm32f1xx_it.c
![[Pasted image 20250321090514.png | 800]]
如上所示，勾选CAN TX Interrupts全局中断后，stm32f1xx_it.c增加函数`USB_HP_CAN1_TX_IRQHandler()`，它就是CAN发送的中断回调的主函数。

## 2.3、main.c
![[Pasted image 20250327195406.png | 1100]]





