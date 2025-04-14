# 导言
---
![[Pasted image 20250411165351.png]]
如上所示，根据《STM32F1中文参考手册》的章节22.8，通过寄存器CAN_IER可以打开FIFO1溢出中断。然后，在全局中断里查看寄存器CAN_RF1R的FOVR1是否被置1。

![[Pasted image 20250411115934.png]]
如上所示，寄存器CAN_RF1R的FOVR1的说明。

项目地址：
- HAL库
- 寄存器方式：

# 一、代码（HAL库）
---
## 1.1、HAL库对接收FIFO的溢出中断支持不足
![[Pasted image 20250411170627.png | 1100]]
如上所示，FIFO1溢出中断并没有像接收中断一样(`HAL_CAN_RxFifo1MsgPendingCallback()`)，另外，有一个对应的函数名给我们去使用。错误类型的中断，全部统一用`HAL_CAN_ErrorCallback()`。

> 几番调试后发现HAL_CAN_ErrorCallback()也不行，FIFO1溢出中断产生的时候，不会进入`HAL_CAN_ErrorCallback()`。所以决定不用HAL库的函数HAL_CAN_ErrorCallback()!!!

## 1.2、myCanDrive.c
![[Pasted image 20250414114747.png | 1100]]
![[Pasted image 20250414115006.png | 1100]]
如上所示，在全局中断函数`CAN1_RX1_IRQHandler()`里调用自己编写的FIFO1接收溢出中断处理函数`CAN_FIFO1_Overflow_Handler()`。函数`CAN_FIFO1_Overflow_Handler()`的目的很简单，只是将全局变量`g_RxOverflowError`累加。通过全局变量`g_RxOverflowError`等于10的话，证明有10个CAN消息被丢掉了，丢掉的原因是FIFO1满了，导致接收溢出。

实际项目上，我们就是通过全局变量`g_RxOverflowError`来监控CAN接收的情况。如果有接收溢出的话，证明我们应调整CAN接收过滤器。让那些我们不关心的CAN报文禁止进入接收FIFO1。

## 1.3、myCanDrive.h
![[Pasted image 20250414115114.png | 1100]]
## 1.4、stm32f1xx_it.c
![[Pasted image 20250414115219.png | 1100]]

# 二、测试（HAL库）
---
## 2.1、编译代码
![[Pasted image 20250414115744.png | 1100]]
如上所示，代码编译成功。

## 2.2、debug测试
![[Pasted image 20250414142100.png | 1100]]
如上所示：
1. 程序初始化后，使用CAN分析仪发送4个CAN报文到CAN总线上，立刻触发了STM32F103的接收FIFO1溢出中断。
2. 在Keil的debug模式下，观察System Viewer->CAN->CAN_RF1R，看到FOVR1被置1（FIFO1溢出了）、看到FULL1被置1（FIFO1满了，没有空闲的接收邮箱）、看到FMP1 = 3（FIFO1的三个邮箱都有CAN报文）。
3. 调试断点卡在函数`CAN_FIFO1_Overflow_Handler()`里。
![[Pasted image 20250414142226.png | 1100]]
如上所示，云释放函数`CAN_FIFO1_Overflow_Handler()`里的断点后，全局变量`g_RxOverflowError`从0变成1，表示FIFO1接收溢出了（丢失了）一条CAN报文。
总的效果如下：
![[LL18_CAN_RX_FIFO1_HAL.gif | 1100]]

# 三、代码（寄存器方式）
---
## 3.1、myCanDrive_reg.c
![[Pasted image 20250414180021.png | 1100]]
函数`CAN_Config()`里，将代码`CAN1->IER |= CAN_IER_FMPIE1;`注释掉，方便触发FIFO1溢出中断。接着，通过代码`CAN1->IER |= CAN_IER_FOVIE1;`开启FIFO1溢出中断。

![[Pasted image 20250414180242.png | 1100]]
为了让代码更加模块化，将FIFO1溢出中断处理与之前的FIFO1挂号中断处理都各自编写一个函数来处理。
![[Pasted image 20250414180516.png | 1100]]
在全局中断里，根据标志位判断到底是什么中断，根据中断标志进行处理。

# 四、测试（寄存器方式）
---
## 4.1、编译代码
![[Pasted image 20250414180654.png | 1100]]

## 4.2、debug测试
![[Pasted image 20250414180852.png | 1100]]
![[Pasted image 20250414180954.png | 1100]]
总的来说，效果跟HAL库一样。但是，寄存器的代码精简很多，效率最高。
![[LL18_CAN_RX_FIFO1_Reg.gif | 1100]]
如上所示，效果跟HAL库一样。实验成功！！！






