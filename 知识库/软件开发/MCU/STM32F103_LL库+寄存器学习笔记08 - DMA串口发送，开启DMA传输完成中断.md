# 导言
---
DMA辅助USART发送与接收能够显著降低CPU负担，尤其在大数据量传输时优势明显。这样可以让CPU腾出更多资源去处理其他任务，同时利用DMA的并行传输能力提高数据传输效率。
大概可以这样理解：STM32F1的CPU是一个核，DMA1是第二个核，DMA2是第三个核。掌握DMA的使用后，相当于你的STM32F1芯片从单核变成三核。虽然，DMA只能用于数据的搬运，但是一个计算机系统做的绝大部分的活不就是数据搬运吗？所以，DMA必须学会！必须学会！必须学会！

效果如下：
![[LL08_USART_Send 1.gif | 800]]
如上所示，电脑串口助手发送字符串"Send_Msg\r\n"到单片机，单片机将字符串"Send_Mag\0"返回来。这一次将字符串返回来的不是CPU，而是DMA！

项目地址：https://github.com/q164129345/MCU_Develop/tree/main/stm32f103_ll_library08_usart_dma_send

# 一、CubeMX
---
![[Pasted image 20250304140027.png | 800]]
如上所示，在CubeMX上配置了DMA协助USART1_RX、USART1_TX。通常，RX的优先级要比TX高，先保证不丢失接收的数据。
![[Pasted image 20250304140350.png | 1100]]

## 1.1、usart.c
![[Pasted image 20250304140751.png | 800]]
![[Pasted image 20250304140908.png | 800]]
![[Pasted image 20250304141015.png | 800]]
如上所示，跟上一章节相比，在函数`MX_USART1_UART_Init()`多些DMA相关的代码。

## 1.2、stm32f1xx_it.c
![[Pasted image 20250304144720.png | 800]]
如上所示，在stm32f1xx_it.c里多了两个DMA1的全局中断函数。
![[Pasted image 20250304150756.png | 1000]]
如上所示，《STM32F1参考手册》的章节9.1.2的中断向量表看到，DMA1的通道4～通道5都有各自的全局中断地址，所以中断回调函数都不一样。

## 1.3、dma.c
![[Pasted image 20250305110205.png | 800]]
如上所示，LL库已经包含了DMA的时钟与全局中断代码。

# 二、代码（LL库）
---
开始使用LL库，实现USART1的DMA发送程序。
## 2.1、main.c
![[Pasted image 20250305112529.png | 800]]
![[Pasted image 20250305113051.png | 800]]
![[Pasted image 20250305113534.png | 800]]
![[Pasted image 20250305113912.png | 800]]

## 2.2、stm32f1xx_it.c
![[Pasted image 20250305134849.png | 800]]
如上所示，在DMA1通道4的全局中断里主要都是清除标志与关闭DMA通道。
![[Pasted image 20250305135021.png | 800]]
如上所示，USART1全局中断的内容没有变化，跟上一章节一样。

## 2.3、编译、下载
![[Pasted image 20250305144036.png | 800]]
如上所示，编译通过。
![[LL08_USART_Send 2.gif | 800]]
如上所示，电脑串口助手发送字符串"Send_Msg\r\n"到单片机，单片机将字符串"Send_Mag\0"返回来。这一次将字符串返回来的不是CPU，而是DMA！

# 三、寄存器梳理
---
## 3.1、DMA知识划重点
![[Pasted image 20250305144958.png ]]
![[Pasted image 20250305153000.png]]
如上所示，DMA经过总线矩阵，可以访问APB1的所有外设、APB2的所有外设、SRAM、Flash。
![[Pasted image 20250305160352.png]]
如上所示，函数USART1_SendString_DMA()的代码是按照这个流程配置的。
![[Pasted image 20250305160612.png]]
如上所示，三个中断都很重要，都会用得上。
![[Pasted image 20250305161028.png]]
如上所示，根据《STM32F1参考手册》的章节10.3.7 - DMA请求映像看到，USART1_TX对应DMA1的通道4。

## 3.2、开启DMA1的时钟
![[Pasted image 20250305161941.png]]
![[Pasted image 20250305162010.png]]
如上所示，寄存器RCC_AHBENR的位0-DMA1EN置1，开启DMA1时钟。
```c
RCC->AHBENR |= (1UL << 0UL); // 开启DMA1时钟
```

## 3.3、配置USART1_TX的DMA1通道4
![[Pasted image 20250305171242.png]]
![[Pasted image 20250305171712.png]]
![[Pasted image 20250305172300.png]]
如上所示，配置DMA1协助USART1发送字符串，基本上将每一位都设置一遍（除了bit0）。
```c
// 配置DMA1的通道4：普通模式，内存到外设(flash->USART1_TX)，优先级高，存储器地址递增、数据大小都是8bit
__STATIC_INLINE void DMA1_Channel4_Configure(void) {
    // 开启时钟
    RCC->AHBENR |= (1UL << 0UL); // 开启DMA1时钟
    // 设置并开启全局中断
    NVIC_SetPriority(DMA1_Channel4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
    NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    // 数据传输方向
    DMA1_Channel4->CCR &= ~(1UL << 14UL); // 外设到存储器模式
    DMA1_Channel4->CCR |= (1UL << 4UL); // DIR位设置1（内存到外设）
    // 通道优先级
    DMA1_Channel4->CCR &= ~(3UL << 12UL); // 清除PL位
    DMA1_Channel4->CCR |= (2UL << 12UL);  // PL位设置10，优先级高
    // 循环模式
    DMA1_Channel4->CCR &= ~(1UL << 5UL);  // 清除CIRC位，关闭循环模式
    // 增量模式
    DMA1_Channel4->CCR &= ~(1UL << 6UL);  // 外设存储地址不递增
    DMA1_Channel4->CCR |= (1UL << 7UL);   // 存储器地址递增
    // 数据大小
    DMA1_Channel4->CCR &= ~(3UL << 8UL);  // 外设数据宽度8位，清除PSIZE位，相当于00
    DMA1_Channel4->CCR &= ~(3UL << 10UL); // 存储器数据宽度8位，清除MSIZE位，相当于00
    // 中断
    DMA1_Channel4->CCR |= (1UL << 1UL);   // 开启发送完成中断
}
```

![[Pasted image 20250305205808.png | 1100]]

如上所示，函数DMA1_Channel4_Configure()运行完之后，从debug模式的寄存器列表看到只有DMA1的寄存器CCR4有值。
![[Pasted image 20250305210154.png | 1100]]
如上所示，代码跟debug模式下看到的寄存器状态一一对应上。

## 3.4、串口DMA发送
```c
void USART1_SendString_DMA(const char *data, uint16_t len)
{
    // 等待上一次DMA传输完成（也可以添加超时机制）
    while(tx_dma_busy);
    tx_dma_busy = 1; // 标记DMA正在发送
    // 如果DMA通道4正在使能，则先禁用以便重新配置
    if(DMA1_Channel4->CCR & 1UL) { // 检查EN位（bit0）是否置位
        DMA1_Channel4->CCR &= ~1UL;  // 禁用DMA通道4（清除EN位）
        while(DMA1_Channel4->CCR & 1UL); // 等待DMA通道4完全关闭
    }
    // 配置DMA通道4：内存地址、外设地址及数据传输长度
    DMA1_Channel4->CMAR  = (uint32_t)data;         // 配置内存地址
    DMA1_Channel4->CPAR  = (uint32_t)&USART1->DR;  // 配置外设地址
    DMA1_Channel4->CNDTR = len;                    // 配置传输数据长度
    // 开启USART1的DMA发送请求：CR3中DMAT（第7位）置1
    USART1->CR3 |= (1UL << 7UL);
    // 启动DMA通道4：设置EN位启动DMA传输
    DMA1_Channel4->CCR |= 1UL;
}
```
### 3.4.1、配置DMA1通道4的内存地址
![[Pasted image 20250305190710.png]]
```c
void USART1_SendString_DMA(const char *data, uint16_t len) {
    //
	DMA1_Channel4->CMAR  = (uint32_t)data;   // 配置内存地址
	//
}
```
如上所示，寄存器DMA_CMAR就是告诉DMA从哪里（32位地址）开始搬运数据。

### 3.4.2、配置DMA1通道4的外设地址
![[Pasted image 20250305190928.png]]

如上所示，寄存器DMA_CPAR就是告诉DMA搬运数据的目的地（32位地址）。
```c
DMA1_Channel4->CPAR  = (uint32_t)&USART1->DR;  // 配置外设地址串口1的DR寄存器，把字节数据发送出去
```

### 3.4.3、配置DMA1通道4的搬运数量
![[Pasted image 20250305191150.png]]
```c
void USART1_SendString_DMA(const char *data, uint16_t len) {
    //
	DMA1_Channel4->CNDTR = len;  // 配置传输数据长度
	//
}
```

### 3.4.4、开启USART1的DMA发送请求
![[Pasted image 20250305191545.png]]
```c
USART1->CR3 |= (1UL << 7UL); // 开启USART1的DMA发送请求：CR3中DMAT（第7位）置1
```

### 3.4.5、启动DMA1通道4传输数据
![[Pasted image 20250305191859.png]]
![[Pasted image 20250305191916.png]]
```c
DMA1_Channel4->CCR |= 1UL;
```

## 3.5、DMA1通道4全局中断
```c
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */
    if(DMA1->ISR & (1UL << 13)) {
        // 清除DMA传输完成标志：在IFCR寄存器中写1清除对应标志
        DMA1->IFCR |= (1UL << 13);
        // 禁用DMA通道4（清除CCR寄存器的EN位，位0）
        DMA1_Channel4->CCR &= ~(1UL << 0);
        // 清除USART1中DMAT位，关闭DMA发送请求（CR3寄存器的位7）
        USART1->CR3 &= ~(1UL << 7);
        // 标记DMA发送完成
        tx_dma_busy = 0;
    }
  /* USER CODE END DMA1_Channel4_IRQn 1 */
}
```

### 3.5.1、判断是不是DMA1通道4传输完成中断
![[Pasted image 20250305194022.png]]
如上所示，寄存器DMA_ISR的bit13-TCIF4代表的是DMA1通道4的传输完成中断。
```c
if (DMA1->ISR & (1UL << 13UL)) {
   // DMA1通道4传输完成
}
```

### 3.5.2、清除DMA1通道4传输完成标志
![[Pasted image 20250305202605.png]]
如上所示，通过寄存器DMA_IFCR的位13-CTCIF置1来清除传输完成中断。
```c
DMA1->IFCR |= (1UL << 13); // 清除DMA传输完成标志：在IFCR寄存器中写1清除对应标志
```

# 四、代码（寄存器方式）
---
## 4.1、main.c
![[Pasted image 20250305203540.png | 800]]
![[Pasted image 20250305203634.png | 800]]
![[Pasted image 20250305203732.png | 800]]
![[Pasted image 20250305203835.png | 800]]
## 4.2、stm32f1xx_it.c
![[Pasted image 20250305203945.png | 800]]

## 4.3、编译、debug调试
![[Pasted image 20250305204310.png | 800]]
如上所示，成功编译。
![[LL08_USART_Send1.gif | 800]]
如上所示，效果跟LL库一样。



