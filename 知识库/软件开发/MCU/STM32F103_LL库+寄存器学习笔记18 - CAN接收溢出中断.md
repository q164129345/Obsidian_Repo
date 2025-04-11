# 导言
---
![[Pasted image 20250411165351.png]]
如上所示，根据《STM32F1中文参考手册》的章节22.8，通过寄存器CAN_IER可以打开FIFO1溢出中断。然后，在全局中断里查看寄存器CAN_RF1R的FOVR1是否被置1。

![[Pasted image 20250411115934.png]]
如上所示，寄存器CAN_RF1R的FOVR1的说明。

# 一、代码（HAL库）
---
## 1.1、HAL库对接收FIFO的溢出中断支持不足
![[Pasted image 20250411170627.png | 1100]]
如上所示，FIFO1溢出中断并没有像接收中断一样(`HAL_CAN_RxFifo1MsgPendingCallback()`)，有一个对应的函数名给我们去使用。错误类型的中断，全部统一用`HAL_CAN_ErrorCallback()`。

> 调试后发现HAL_CAN_ErrorCallback()也不行，所以决定不用HAL库的函数HAL_CAN_ErrorCallback()。

## 1.2、myCanDrive.c











