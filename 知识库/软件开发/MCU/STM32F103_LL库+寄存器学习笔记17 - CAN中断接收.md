# 导言
---
![[Pasted image 20250409194119.png]]
如上所示，STM32F103有两个3级深度的接收FIFO。外设CAN想要正常接收CAN报文，必须配置接收FIFO与接收滤波器。两者缺一不可，否则导致无法接收CAN报文。

![[Pasted image 20250409194247.png]]
如上所示，当接收FIFO的3级深度（缓存）都被占满时，将会导致溢出。此时，必须让MCU快读取接收FIFO的邮箱，避免出现溢出。

# 一、CubeMX
---
CubeMX不主持生成外设CAN的LL库代码，仅支持HAL库。所以，先看看HAL库怎样实现接收CAN报文。

## 1.1、开启接收FIFO1中断
![[Pasted image 20250409194909.png | 800]]
如上所示，只需要在NVIC_Settings里勾选CAN_RX1_interrupt即可开启接收FIFO1中断。另外，也可以选择接收FIFO0中断。二者只能选择其一，不能两个都选择。



