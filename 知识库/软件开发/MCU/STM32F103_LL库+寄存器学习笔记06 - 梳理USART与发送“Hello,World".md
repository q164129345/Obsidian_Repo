# 导言
---
USART是嵌入式非常重要的通讯方式，它的功能强大、灵活性高且用途广泛。只停留在HAL库层面上用USART只能算是入门，要加深对USART的理解，必须从寄存器层面入手。接下来，先从最简单的USART串行发送开始。

效果如下所示：
每隔1是发送一条字符串"Hello,Wrold.\r\n"。
![[LL06_USART.gif | 1100]]

项目地址：https://github.com/q164129345/MCU_Develop/tree/main/stm32f103_ll_library06_usart

# 一、CubeMX
---
![[20250228-112018.jpg | 1100]]
![[Pasted image 20250228112241.png | 1100]]

# 二、代码
---
## 2.1、main.c
![[Pasted image 20250228143832.png | 800]]
![[Pasted image 20250228144218.png | 800]]
![[Pasted image 20250228144433.png | 800]]
## 2.1.1、MX_USART1_UART_Init()
![[Pasted image 20250228144946.png | 800]]
#### 2.1.1.1、USART1_TX(PA9)
![[Pasted image 20250228151053.png | 800]]
复习一下GPIO的寄存器CRL/CRH，如下：
![[Pasted image 20250228145817.png | 800]]
#### 2.1.1.2、USART1_RX(PA10)
![[Pasted image 20250228151137.png | 800]]
复习一下GPIO的寄存器CRL/CRH，如下：
![[Pasted image 20250228145926.png | 800]]
#### 2.1.1.3、为什么USART模式下，PA9与PA10要这样设置GPIO？？？
![[Pasted image 20250228153335.png | 800]]
如上所示，《STM32F1参考手册》章节8.1.11看到，当GPIO用在USART功能时，TX引脚需要设置推挽复用输出，RX引脚需要设置浮空输入或者带上拉输入。
![[Pasted image 20250228153609.png | 800]]
如上所示，从这张表里看到另外一个很常用的通讯方式CAN总线，TX引脚也是设置推挽复用输出，RX引脚也是设置浮空输入或者带上拉输入。

#### 2.1.1.4、USART1
![[Pasted image 20250228151538.png | 800]]

# 三、寄存器梳理
---
## 3.1、总结USART发送的核心知识
![[Pasted image 20250228171324.png | 1100]]
《STM32F1参考手册》的章节25.3.2-发送器一定要多看几遍，总的来说：
1. 将要发送的字节放入USART_DR寄存器之前，一定要等待USART_SR的位TXE = 1。代码如下：
```c
void USART1_SendChar_Reg(uint8_t c) {  
	// 等待TXE置位（发送缓冲区空）  
	while (!(USART1->SR & (1 << 7)));  
	// 写入数据到DR  
	USART1->DR = c;  
}
```
1. USART_TXE是发送的”第一级缓冲“状态，表示数据从USART_DR到移位寄存器的转移完成，并不意味着数据已经完全发送出去。如下图所示：
![[Pasted image 20250228173209.png | 1100]]
3. USART_TC才是代表数据完成发送出去，TC = 1时，表明硬件发送通道完全空闲。比如在低功耗场景中，通过TC = 1的判断，进入低功耗模式。
4. 单字节发送流程，TXE先置1，TC后置1。用户写入USART_DR->TXE清零。接着，数据转移到移位寄存器->TXE置1，此时可以写入新数据。最后，移位寄存器逐位发送数据->发送完成，TC置1。

## 3.2、总结USART接收的核心知识
![[Pasted image 20250228180939.png | 1100]]
如上所示。