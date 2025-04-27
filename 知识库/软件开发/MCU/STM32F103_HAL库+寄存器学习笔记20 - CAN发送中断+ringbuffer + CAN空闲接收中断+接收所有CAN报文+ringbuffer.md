
# 导言
---
![[Pasted image 20250418134814.png]]
如上所示，在[[STM32F103_HAL库+寄存器学习笔记19 - CAN发送中断+CAN接收中断+接收所有CAN报文+ringbuffer数据结构]]的基础上，为CAN发送也安排上强大的ringbuffer（环形缓冲区）。CAN发送有三个发送邮箱，为什么还另外需要ringbuffer？
1. **三邮箱限制**：STM32F103 的 CAN 控制器只有三个发送邮箱，当应用层短时间内产生超过 3 条待发帧时，多余的帧就无法立即写入邮箱，会被丢弃或需要阻塞（阻塞相对来说更可靠一些，但是会阻塞！）。
2. **突发流量平滑**：环形缓冲区可以暂存超出邮箱数的那些帧，在邮箱空闲时再依次发送，避免丢帧。
3. **解耦业务与硬件**：应用层只需往环形缓冲区写入数据，不用关心底层邮箱是否空闲，降低了发送接口的耦合度。

**总之，在CAN发送的链路上增加ringbuffer可以有效地解决上述三个问题（尤其是第一与第二个问题）。**

项目地址：
github:
- HAL库: https://github.com/q164129345/MCU_Develop/tree/main/stm32f103_hal_library20_Can_Send_Rec_With_RB
- 寄存器方式: https://github.com/q164129345/MCU_Develop/tree/main/stm32f103_ll_library20_Can_Send_Rec_With_RB

gitee(国内):
- HAL库: https://gitee.com/wallace89/MCU_Develop/tree/main/stm32f103_hal_library20_Can_Send_Rec_With_RB
- 寄存器方式: https://gitee.com/wallace89/MCU_Develop/tree/main/stm32f103_ll_library20_Can_Send_Rec_With_RB

## 使用for循环一口气发送50条CAN报文
为了验证ringbuffer可以解决上述提到的问题，尤其是问题1（三邮箱限制）与问题2（突发流量平滑）。在主循环里调用for循环一口气发送50条CAN报文，看看效果！
![[Pasted image 20250423162654.png]]
![[Pasted image 20250423164504.png]]
如上所示：
1. 在debug模式里，将对应的全局变量置1，就可以运行对应的测试函数。

## 测试一口气发送50条CAN报文，没有ringbuffer
![[test_no_ringbuffer.gif]]
如上所示，将全局变量test置1，运行函数`CAN_Test_Send50Frames()`一口气发送50条CAN报文。接着，CAN分析仪只接收到3条报文。原因很简单，另外47条报文因为发送邮箱挤满了，所以都溢出了。

## 测试一口气发送50条CAN报文，有ringbuffer
![[test_with_ringbuffer.gif]]
如上所示，将全局变量testRB置1，运行函数`CAN_Test_Send50Frames_Use_Ringbuffer()`一口气发送50条报文。接着，CAN分析仪接收到50条报文。实践证明，ringbuffer作为二级缓存能有效解决突发高流量问题与三个邮箱的局限性问题。

# 一、代码（HAL库）
---
## 1.1、myCanDrive.c
![[Pasted image 20250423175242.png]]
![[Pasted image 20250423181229.png]]
![[Pasted image 20250423181425.png]]
![[Pasted image 20250423181634.png]]

## 1.2、stm32f1xx_it.c
![[Pasted image 20250423181843.png]]

## 1.3、main.c
![[Pasted image 20250423182001.png]]
![[Pasted image 20250423182045.png]]

## 1.4、代码编译
![[Pasted image 20250423182230.png]]

# 二、代码（寄存器方式）
---
## 1.1、myCanDrive_reg.c
![[Pasted image 20250424182528.png]]
如上所示，RX Ringbuffer与TX Ringbuffer都是二级缓存。
![[Pasted image 20250424182946.png]]
![[Pasted image 20250424182821.png]]
如上所示，这两个函数基本跟HAL库一样，只是开与关中断的函数改为寄存器方式直接操作。
![[Pasted image 20250424183113.png]]
如上所示，在CAN发送完成中断函数`USB_HP_CAN1_TX_IRQHandler()`里调用函数`CAN_Get_CANMsg_From_RB_To_TXMailBox_IT()`，在中断里将ringbuffer里的CAN报文放入空闲的发送邮箱。

## 1.2、main.c
![[Pasted image 20250424183446.png]]
![[Pasted image 20250424183538.png]]

## 1.3、代码编译
![[Pasted image 20250424183703.png]]

## 1.4、测试
![[LL_20_CAN_TX_RB.gif]]
如上所示，测试结果跟HAL库一样。


