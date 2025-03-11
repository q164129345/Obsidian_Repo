# 导言
---
上一章节《[[STM32F103_LL库+寄存器学习笔记08 - DMA串口发送，开启DMA传输完成中断]]》完成DMA辅助串口发送。接着，梳理DMA辅助串口接收，且启动串口接收空闲中断。

效果如下所示：
![[LL09_USART_IDLE_DMA.gif | 1000]]
串口助手发送字符串`"LL_Example09_DMA_Rece_IDLE\r\n"`给STM32F103，接着马上收到来自STM32F103发出来的字符串`"LL_Example09_DMA_Rece_IDLE\r\n"`。

来一个高强度的测试效果，115200 波特率意味着每秒传输 115200 个比特。如果使用标准格式（1 个起始位、8 个数据位、1 个停止位，共 10 个比特传输一个字节），那么每秒可以传输 115200 / 10 = 11520 个字节。换算下来，每毫秒大约传输 11520 / 1000 ≈ 11.52 字节，**实际约为 11 个字节/ms。**
接着，我用100个字节/10ms去测试收发的效果（相当于每1S传输10000bytes）。因为每一次发送的字节总数多了，所以RX与TX的缓存区我都改为2K了。效果如下所示：
![[LL09_USART_IDLE_DMA8.gif]]
最后，发送了87100bytes，接收了87100bytes，发送=接收，**没有丢包！USART1的发送与接收都是DMA完成的！**

项目地址：https://github.com/q164129345/MCU_Develop/tree/main/stm32f103_ll_library09_usart_dma_rec_with_ilde

# 一、CubeMX
---
![[Pasted image 20250304140027.png | 800]]
如上所示，跟上一章节的设置一样。

# 二、代码（LL库）
---
## 2.1、usart.c
![[Pasted image 20250306150543.png | 800]]
如上所示，在函数`MX_USART1_UART_Init()`里添加三句代码。

## 2.2、main.c
![[Pasted image 20250306164936.png | 800]]
![[Pasted image 20250306151055.png | 800]]
![[Pasted image 20250306164757.png | 800]]
## 2.3、stm32f1xx_it.c
![[Pasted image 20250306151509.png | 800]]
![[Pasted image 20250306170022.png | 800]]

## 2.4、dma.c
代码不用动，CubeMX自动生成。dma.c启动了DMA1的时钟，打开DMA1的通道4与通道5的全局中断，并设置中断优先级0。
```c
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dma.c
  * @brief   This file provides code for the configuration
  *          of all the requested memory to memory DMA transfers.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "dma.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure DMA                                                              */
/*----------------------------------------------------------------------------*/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */


```

## 2.5、编译、下载
---
![[Pasted image 20250306152702.png | 800]]
![[LL09_USART_IDLE_DMA 1.gif | 800]]
如上所示，功能正常。

# 三、寄存器梳理
---
## 3.1、DMA知识划重点
![[Pasted image 20250306153155.png]]
DMA1通道5辅助USART1接收串口数据，一般启动循环模式。在循环模式下，当写指针写到最后一个内存时，会自动地回到第一个内存。跟数据结构ringbuffer一样。
![[Pasted image 20250306153608.png]]
正如上面所说，在循环模式下，DMA写到最后一个内存时会自动回到第一个内存。此时，旧数据被新数据覆盖。可以通过加入中断“传输过半”来防止这件事情的发生。后续章节会调试这个中断功能。

## 3.2、启动USART1接收空闲中断
![[Pasted image 20250306155357.png]]
![[Pasted image 20250306155432.png]]
```c
USART1->CR1 |= (1UL << 4UL);        // 使能USART1空闲中断 (IDLEIE, 位4)
```

## 3.3、启动USART1的DMA接收
![[Pasted image 20250306155604.png]]
![[Pasted image 20250306155626.png]]
```c
USART1->CR3 |= (1UL << 6UL);        // 使能USART1的DMA接收请求（DMAR，位6）
```

## 3.4、配置USART1_RX的DMA1通道5
![[Pasted image 20250306155930.png]]
![[Pasted image 20250306160028.png]]
![[Pasted image 20250306160151.png]]
如上所示，DMA1通道5辅助USART1_RX的设置。
![[Pasted image 20250306160505.png]]
![[Pasted image 20250306160522.png]]
如上所示，配置完成关键的寄存器CCR之后，继续配置如上三个寄存器。分别是：
1. 寄存器CPAR （设置外设的地址，本章节是&USART1->DR）
2. 寄存器CMAR（设置存储器地址）
3. 寄存器CNDTR（设置缓存区的长度）

代码如下：
```c
__STATIC_INLINE void DMA1_Channel5_Configure(void) {
    RCC->AHBENR |= (1UL << 0UL); // 开启DMA1时钟
    /* 1. 禁用DMA通道5，等待其完全关闭 */
    DMA1_Channel5->CCR &= ~(1UL << 0);  // 清除EN位
    while(DMA1_Channel5->CCR & 1UL);    // 等待DMA通道5关闭

    /* 2. 配置外设地址和存储器地址 */
    DMA1_Channel5->CPAR = (uint32_t)&USART1->DR;  // 外设地址为USART1数据寄存器
    DMA1_Channel5->CMAR = (uint32_t)rx_buffer;    // 存储器地址为rx_buffer
    DMA1_Channel5->CNDTR = RX_BUFFER_SIZE;        // 传输数据长度为缓冲区大小
    /* 3. 配置DMA1通道5 CCR寄存器
         - DIR   = 0：外设→存储器
         - CIRC  = 1：循环模式
         - PINC  = 0：外设地址不自增
         - MINC  = 1：存储器地址自增
         - PSIZE = 00：外设数据宽度8位
         - MSIZE = 00：存储器数据宽度8位
         - PL    = 10：优先级设为高
         - MEM2MEM = 0：非存储器到存储器模式
    */
    DMA1_Channel5->CCR = 0;  // 清除之前的配置
    DMA1_Channel5->CCR |= (1UL << 5);       // 使能循环模式 (CIRC，bit5)
    DMA1_Channel5->CCR |= (1UL << 7);       // 使能存储器自增 (MINC，bit7)
    DMA1_Channel5->CCR |= (3UL << 12);      // 设置优先级为非常高 (PL置为“11”，bit12-13)

    /* 4. 使能DMA通道5 */
    DMA1_Channel5->CCR |= 1UL;  // 置EN位启动通道
}
```

## 3.5、USART1全局中断
```c
void USART1_IRQHandler(void)
{
    // 检查USART1 SR寄存器的IDLE标志（bit4）
    if (USART1->SR & (1UL << 4)) {
        uint32_t tmp;
        // 清除IDLE标志：先读SR再读DR
        tmp = USART1->SR;
        tmp = USART1->DR;
        (void)tmp;
        // 禁用DMA1通道5：清除CCR寄存器的EN位（bit0）
        DMA1_Channel5->CCR &= ~(1UL << 0);
        // (可选)等待通道确实关闭：while(DMA1_Channel5->CCR & (1UL << 0));
        // 计算本次接收的字节数
        // recvd_length = RX_BUFFER_SIZE - DMA1_Channel5->CNDTR
        uint16_t recvd_length = RX_BUFFER_SIZE - DMA1_Channel5->CNDTR;
        // 如果需要将数据作为字符串处理，添加结束符
        if(recvd_length < RX_BUFFER_SIZE) {
            rx_buffer[recvd_length] = '\0';
        }
        // 将接收到的数据复制到发送缓冲区
        memcpy((void*)tx_buffer, (const void*)rx_buffer, recvd_length);
        // 标记接收完成
        rx_complete = 1;
        // 重置DMA接收：设置CNDTR寄存器为RX_BUFFER_SIZE
        DMA1_Channel5->CNDTR = RX_BUFFER_SIZE;
        // 重新使能DMA1通道5：设置EN位（bit0）
        DMA1_Channel5->CCR |= 1UL;
    }
}
```
### 3.5.1、判断USART1的IDLE中断
![[Pasted image 20250306161212.png]]
![[Pasted image 20250306161236.png]]
```c
if (USART1->SR & (1UL << 4UL)) {
	// 检测到空闲中断
}
```

### 3.5.2、清除IDLE标志
![[Pasted image 20250306162636.png]]
![[Pasted image 20250306162742.png]]
```c
uint32_t tmp;
/* 清除IDLE标志，必须先读SR，再读DR */
tmp = USART1->SR;
tmp = USART1->DR;
(void)tmp;
```

### 3.5.3、计算DMA接收到的字节数
![[Pasted image 20250306171416.png]]
```c
recvd_length = RX_BUFFER_SIZE - DMA1_Channel5->CNDTR; // 计算DMA接收到的字节数
```

### 3.5.4、重启DMA接收
![[Pasted image 20250306171658.png]]
![[Pasted image 20250306171732.png]]
```c
// 重置DMA接收：设置CNDTR寄存器为RX_BUFFER_SIZE
DMA1_Channel5->CNDTR = RX_BUFFER_SIZE;
// 重新使能DMA1通道5：设置EN位（bit0）
DMA1_Channel5->CCR |= 1UL;
```

如上所示，在全局中断`USART1_IRQHandler()`里重新启动DMA接收，需要重新配置寄存器CNDTR来再一次告诉DMA缓存区的大小。接着，通过寄存器CCR的bit0再一次DMA1通道5。

# 四、代码（寄存器方式）
## 4.1、main.c
![[Pasted image 20250306172206.png | 800]]
![[Pasted image 20250306172425.png | 800]]
```c
__STATIC_INLINE void USART1_Configure(void) {
    /* 1. 使能外设时钟 */
    // RCC->APB2ENR 寄存器控制 APB2 外设时钟
    RCC->APB2ENR |= (1UL << 14UL); // 使能 USART1 时钟 (位 14)
    RCC->APB2ENR |= (1UL << 2UL);  // 使能 GPIOA 时钟 (位 2)
    /* 2. 配置 GPIO (PA9 - TX, PA10 - RX) */
    // GPIOA->CRH 寄存器控制 PA8-PA15 的模式和配置
    // PA9: 复用推挽输出 (模式: 10, CNF: 10)
    GPIOA->CRH &= ~(0xF << 4UL);        // 清零 PA9 的配置位 (位 4-7)
    GPIOA->CRH |= (0xA << 4UL);         // PA9: 10MHz 复用推挽输出 (MODE9 = 10, CNF9 = 10)
    // PA10: 浮空输入 (模式: 00, CNF: 01)
    GPIOA->CRH &= ~(0xF << 8UL);        // 清零 PA10 的配置位 (位 8-11)
    GPIOA->CRH |= (0x4 << 8UL);         // PA10: 输入模式，浮空输入 (MODE10 = 00, CNF10 = 01)
    // 开启USART1全局中断
    NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0)); // 优先级1（优先级越低相当于越优先）
    NVIC_EnableIRQ(USART1_IRQn);
    /* 3. 配置 USART1 参数 */
    // (1) 设置波特率 115200 (系统时钟 72MHz, 过采样 16)
    // 波特率计算: USART_BRR = fPCLK / (16 * BaudRate)
    // 72MHz / (16 * 115200) = 39.0625
    // 整数部分: 39 (0x27), 小数部分: 0.0625 * 16 = 1 (0x1)
    USART1->BRR = (39 << 4UL) | 1;      // BRR = 0x271 (39.0625)
    // (2) 配置数据帧格式 (USART_CR1 和 USART_CR2)
    USART1->CR1 &= ~(1UL << 12UL);      // M 位 = 0, 8 位数据
    USART1->CR2 &= ~(3UL << 12UL);      // STOP 位 = 00, 1 个停止位
    USART1->CR1 &= ~(1UL << 10UL);      // 没奇偶校验
    // (3) 配置传输方向 (收发双向)
    USART1->CR1 |= (1UL << 3UL);        // TE 位 = 1, 使能发送
    USART1->CR1 |= (1UL << 2UL);        // RE 位 = 1, 使能接收
    // (4) 禁用硬件流控 (USART_CR3)
    USART1->CR3 &= ~(3UL << 8UL);       // CTSE 和 RTSE 位 = 0, 无流控
    // (5) 配置异步模式 (清除无关模式位)
    USART1->CR2 &= ~(1UL << 14UL);      // LINEN 位 = 0, 禁用 LIN 模式
    USART1->CR2 &= ~(1UL << 11UL);      // CLKEN 位 = 0, 禁用时钟输出
    USART1->CR3 &= ~(1UL << 5UL);       // SCEN 位 = 0, 禁用智能卡模式
    USART1->CR3 &= ~(1UL << 1UL);       // IREN 位 = 0, 禁用 IrDA 模式
    USART1->CR3 &= ~(1UL << 3UL);       // HDSEL 位 = 0, 禁用半双工
    // (6) 中断
    USART1->CR1 |= (1UL << 4UL);        // 使能USART1空闲中断 (IDLEIE, 位4)
    // (7) 关联DMA的接收请求
    USART1->CR3 |= (1UL << 6UL);        // 使能USART1的DMA接收请求（DMAR，位6） 
    // (7) 启用 USART
    USART1->CR1 |= (1UL << 13UL);       // UE 位 = 1, 启用 USART
}
```
![[Pasted image 20250306172545.png | 800]]
![[Pasted image 20250306172723.png | 800]]
![[Pasted image 20250306172843.png | 800]]

## 4.2、stm32f1xx_it.c
![[Pasted image 20250306172954.png | 800]]
![[Pasted image 20250306173050.png | 800]]

## 4.3、编译、下载
![[Pasted image 20250306173150.png | 1100]]
如上所示，编译通过。
![[LL09_USART_IDLE_DMA1.gif | 1100]]
如上所示，效果跟LL库一样。


# 五、细节补充
---
## 5.1、有没有办法知道接收的字符串的长度超过DMA接收缓冲区？
![[Pasted image 20250306193728.png | 800]]
如上所示，DMA接收的缓冲区大小是64个字节。如果电脑的串口助手一次性发送的字节数超过64的话，会怎样？？
![[LL09_USART_IDLE_DMA2.gif]]
如上所示，串口助手发送字符串"LL_Example09_DMA_Rece_IDLELL_Example09_DMA_Rece_IDLELL_Example09_DMA_Rece_IDLE"，长度78bytes。接着，单片机回传的字符串是"\_DMA_Rece_IDLE"，其长度是14bytes。为什么？咱们计算一下。
DMA接收缓冲区大小64bytes，如果一次性发送78bytes的话，相当于多了78-64=14bytes。原因是DMA1通道5设置了循环模式，当发送数据量超过缓冲区大小（如78字节）时，DMA在接收满64字节后自动重载，导致旧数据被覆盖；IDLE中断时读取的有效数据只剩下新写入的部分（14bytes）。发生这种情况时，USART1与DMA都没有产生错误，对于它们俩来说都是正常。那有什么办法让我们知道上位机曾经发送过一串字符串，长度超过我们设置的缓存区？好让我们把缓冲区调大，避免数据被覆盖？
**使用寄存器DMA_ISR的位TCIFx来判断DMA接收有没有被重载（字符串长度有没有超过DMA接收缓存区）。** 例如本章节，在USART1的IDLE中断里判断DMA1->ISR的位17-TCIF5是不是等于1，如果TCIF5等于1的话，代表DMA接收被重载了。
![[LL09_USART_IDLE_DMA3.gif]]
如上所示：
1. 发送字符串"LL_Example09_DMA_Rece_IDLE"，长度26bytes。 ISR中`GIF5`、`TCIF5`、`HTIF5`、`TEIF5`都未置位。
2. 发送字符串"LL_Example09_DMA_Rece_IDLELL_Example09_DMA_Rece_IDLE"，长度52bytes。ISR中`GIF5=1`，`HTIF5=1(DMA传输过半事件）`，`TCIF5=0`，`TEIF5=0`。
3. 发送字符串"LL_Example09_DMA_Rece_IDLELL_Example09_DMA_Rece_IDLELL_Example09_DMA_Rece_IDLE"，长度78bytes。ISR中`GIF5`、`TCIF5(DMA传输完成事件)`、`HTIF5(DMA传输过半事件)`均为1，但`TEIF5`仍为0，表示没有错误发生。

代码如下:
```c
uint16_t usart1_rec_buffer_full = 0; // 如果这个值大于0,相当于发生过DMA接收缓冲区被重载的事件，应适当调整接收缓冲区大小
void USART1_IRQHandler(void)
{
    // 检查IDLE中断标志（USART1->SR位4）
    if (USART1->SR & (1UL << 4))
    {
        uint32_t tmp;
        // 按照要求先读SR，再读DR，清除IDLE标志
        tmp = USART1->SR;
        tmp = USART1->DR;
        (void)tmp;
        
        // 禁用DMA1通道5（清除CCR的EN位）
        DMA1_Channel5->CCR &= ~(1UL << 0);
        
        // 检查TCIF5是否置位
        // TCIF5位于DMA1->ISR中（对于通道5通常是位17）
        uint8_t tc_flag = (DMA1->ISR & (1UL << 17)) ? 1 : 0;
        if (tc_flag)
        {
            // 清除TCIF5标志，通过IFCR寄存器写1到相应位（位17）
            DMA1->IFCR |= (1UL << 17);
            // 此时可认为接收数据已超过DMA接收缓存区的大小
            usart1_rec_buffer_full++;
        }
        
        // 计算本次接收的字节数：
        // recvd_length = RX_BUFFER_SIZE - 当前DMA剩余字节数（CNDTR寄存器）
        uint16_t recvd_length = RX_BUFFER_SIZE - DMA1_Channel5->CNDTR;
        
        // 如果需要将数据作为字符串处理，在末尾添加结束符（仅当数据未填满整个缓冲区时）
        if (recvd_length < RX_BUFFER_SIZE)
        {
            rx_buffer[recvd_length] = '\0';
        }
        
        // 将接收到的数据从接收缓冲区复制到发送缓冲区
        memcpy((void*)tx_buffer, (const void*)rx_buffer, recvd_length);
        
        // 标记接收完成，主循环中可根据rx_complete进行后续处理
        rx_complete = 1;
        
        // 重置DMA接收：设置CNDTR为RX_BUFFER_SIZE，并重新使能DMA1通道5
        DMA1_Channel5->CNDTR = RX_BUFFER_SIZE;
        DMA1_Channel5->CCR |= 1UL;  // 置EN位启动通道
    }
}
```

