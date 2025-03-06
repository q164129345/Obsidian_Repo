# 导言
---
上一章节《[[STM32F103_LL库+寄存器学习笔记08 - DMA辅助串口发送，DMA传输完成中断]]》完成DMA辅助串口发送。接着，梳理DMA辅助串口接收，且启动串口接收空闲中断。

效果如下所示：
![[LL09_USART_IDLE_DMA.gif | 1000]]
串口助手发送字符串`"LL_Example09_DMA_Rece_IDLE\r\n"`给STM32F103，接着马上收到来自STM32F103发出来的字符串`"LL_Example09_DMA_Rece_IDLE\r\n"`。

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

```c
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 DMA Init */

  /* USART1_TX Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_4, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PRIORITY_MEDIUM);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MDATAALIGN_BYTE);

  /* USART1_RX Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_5, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PRIORITY_HIGH);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MDATAALIGN_BYTE);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */
  LL_USART_EnableDMAReq_RX(USART1); // 使能USART1_RX的DMA请求
  
  LL_USART_EnableIT_IDLE(USART1); // 开启USART1接收空闲中断
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_4); // 使能DMA1通道4的传输完成中断
  /* USER CODE END USART1_Init 2 */

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

```

## 2.2、main.c
![[Pasted image 20250306164936.png | 800]]
![[Pasted image 20250306151055.png | 800]]
![[Pasted image 20250306164757.png | 800]]
```c
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t rx_complete = 0;
volatile uint16_t recvd_length = 0;

volatile uint8_t tx_buffer[TX_BUFFER_SIZE];
volatile uint8_t tx_dma_busy = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if 1
__STATIC_INLINE void Enable_Peripherals_Clock(void) {
    SET_BIT(RCC->APB2ENR, 1UL << 0UL);  // 启动AFIO时钟  // 一般的工程都要开
    SET_BIT(RCC->APB1ENR, 1UL << 28UL); // 启动PWR时钟   // 一般的工程都要开
    SET_BIT(RCC->APB2ENR, 1UL << 5UL);  // 启动GPIOD时钟 // 晶振时钟
    SET_BIT(RCC->APB2ENR, 1UL << 2UL);  // 启动GPIOA时钟 // SWD接口
    __NOP(); // 稍微延时一下下
}


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
#endif

#if 0
// 配置DMA1通道5用于USART1_RX
__STATIC_INLINE void DMA1_Channel5_Configure(void) {
    /* 配置DMA1通道5用于USART1_RX接收 */
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t)rx_buffer);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t)&USART1->DR);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, RX_BUFFER_SIZE);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5);
}

/**
  * @brief  使用DMA发送字符串，采用USART1_TX对应的DMA1通道4
  * @param  data: 待发送数据指针（必须指向独立发送缓冲区）
  * @param  len:  待发送数据长度
  * @retval None
  */
void USART1_SendString_DMA(const char *data, uint16_t len)
{
    // 等待上一次DMA传输完成（也可以添加超时机制）
    while(tx_dma_busy);
    tx_dma_busy = 1; // 标记DMA正在发送
    
    // 如果DMA通道4正在使能，则先禁用以便重新配置
    if (LL_DMA_IsEnabledChannel(DMA1, LL_DMA_CHANNEL_4))
    {
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
        while(LL_DMA_IsEnabledChannel(DMA1, LL_DMA_CHANNEL_4));
    }
    
    // 配置DMA通道4：内存地址、外设地址及数据传输长度
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_4, (uint32_t)data);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_4, (uint32_t)&USART1->DR);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, len);
    
    // 开启USART1的DMA发送请求（CR3中DMAT置1，通常为第7位）
    LL_USART_EnableDMAReq_TX(USART1); // 开启USART1的DMA发送请求
    
    // 启动DMA通道4，开始DMA传输
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);
}

#else
__STATIC_INLINE void DMA1_Channel5_Configure(void) {
    RCC->AHBENR |= (1UL << 0UL); // 开启DMA1时钟
    /* 1. 禁用DMA通道5，等待其完全关闭 */
    DMA1_Channel5->CCR &= ~(1UL << 0);  // 清除EN位
    while(DMA1_Channel5->CCR & 1UL);     // 等待DMA通道5关闭

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
#endif

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  //Enable_Peripherals_Clock(); // 启动所需外设的时钟
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  //LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
  //LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /** NOJTAG: JTAG-DP Disabled and SW-DP Enabled
  */
  LL_GPIO_AF_Remap_SWJ_NOJTAG();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  SysTick->CTRL |= 0x01UL << 1UL; // 开启SysTick中断
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  //MX_DMA_Init();
  //MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  DMA1_Channel5_Configure(); // DMA1通道5
  DMA1_Channel4_Configure(); // DMA1通道4
  USART1_Configure(); // USART1
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (rx_complete) {
        /* 在发送之前，可以选择禁用中断或者采取其他同步机制，
           以避免在发送过程中被中断修改tx_buffer */
        USART1_SendString_DMA((const char*)tx_buffer, recvd_length);
        rx_complete = 0; // 清除标志，等待下一次接收
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(72000000);
  LL_SetSystemCoreClock(72000000);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

```
## 2.3、stm32f1xx_it.c
![[Pasted image 20250306151509.png | 800]]
![[Pasted image 20250306170022.png | 800]]
```c
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
__IO uint32_t gTickCount = 0;
__IO uint8_t pinStatus = 0;

extern volatile uint8_t rx_buffer[];
extern volatile uint8_t rx_complete;
extern volatile uint16_t recvd_length;

extern volatile uint8_t tx_dma_busy;
extern volatile uint8_t tx_buffer[];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  gTickCount++;
  
  if(READ_BIT(GPIOB->IDR, 0x01 << 4UL)) {
      pinStatus = 0x01; // PB4当前高电平
  } else {
      pinStatus = 0x00; // PB4当前低电平
  }
  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */
    // 检查传输完成标志（TC）是否置位（LL库提供TC4接口）
//    if(LL_DMA_IsActiveFlag_TC4(DMA1))
//    {
//        // 清除DMA传输完成标志
//        LL_DMA_ClearFlag_TC4(DMA1);
//        // 禁用DMA通道4，确保下次传输前重新配置
//        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
//        // 清除USART1中DMAT位，关闭DMA发送请求
//        LL_USART_DisableDMAReq_TX(USART1);
//        // 标记DMA发送完成
//        tx_dma_busy = 0;
//    }
    // 寄存器方式
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

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  /* USER CODE BEGIN USART1_IRQn 1 */
    // LL库方式
//    if (LL_USART_IsActiveFlag_IDLE(USART1)) {
//        uint32_t tmp;
//        /* 清除IDLE标志，必须先读SR，再读DR */
//        tmp = USART1->SR;
//        tmp = USART1->DR;
//        (void)tmp;
//        /* 暂时禁用DMA接收，防止数据继续写入 */
//        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_5);
//        /* 计算本次接收的字节数：
//           recvd_length = RX_BUFFER_SIZE - 当前DMA剩余传输字节数 */
//        recvd_length = RX_BUFFER_SIZE - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_5);
//        /* 将接收到的数据从接收缓冲区复制到发送缓冲区 */
//        memcpy((void*)tx_buffer, (const void*)rx_buffer, recvd_length);
//        /* 标记接收完成 */
//        rx_complete = 1;
//        /* 重置DMA接收：重新设置数据长度后再使能DMA */
//        LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, RX_BUFFER_SIZE);
//        LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5);
//    }
    // 寄存器方式
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
        recvd_length = RX_BUFFER_SIZE - DMA1_Channel5->CNDTR;
        // 将接收到的数据复制到发送缓冲区
        memcpy((void*)tx_buffer, (const void*)rx_buffer, recvd_length);
        // 标记接收完成
        rx_complete = 1;
        // 重置DMA接收：设置CNDTR寄存器为RX_BUFFER_SIZE
        DMA1_Channel5->CNDTR = RX_BUFFER_SIZE;
        // 重新使能DMA1通道5：设置EN位（bit0）
        DMA1_Channel5->CCR |= 1UL;
    }
  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

```

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
