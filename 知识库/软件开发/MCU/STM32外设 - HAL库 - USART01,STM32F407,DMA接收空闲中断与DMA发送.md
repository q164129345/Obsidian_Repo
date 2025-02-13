## 导言
---
Keil Version : V5.39.0.0
CubeMX:6.5.0
HAL库把本：V1.26.2
硬件：安富莱的STM32V5开发板
![[Pasted image 20240816224848.png]]

==这个笔记有缺陷，DMA发送被DMA接收打断，导致串口发送异常终止。需要看本笔记的6.2章:==
[[STM32外设 - HAL库 - USART01,STM32F407,DMA接收空闲中断与DMA发送#6.2、本例程有bug，DMA发送会有问题]]


## 一、STM32CubeMX
---
### 1.1、RCC
![[Pasted image 20240816225005.png]]
![[Pasted image 20240816225044.png]]
### 1.2、SYS
![[Pasted image 20240816225228.png]]
### 1.3、USART1
![[Pasted image 20240816225255.png]]
![[Pasted image 20240816225313.png]]
![[Pasted image 20240816225327.png]]
![[Pasted image 20240816225342.png]]
## 二、usart1_drive.c 与 usart1_drive.h
---
### 2.1、usart1.c
```c
#include "usart1_drive.h"
#include "usart.h"

extern UART_HandleTypeDef huart1;

#define RX_BUFFER_SIZE 255
volatile uint8_t rxData[RX_BUFFER_SIZE] = {0,};
volatile static uint8_t flagTxComplete = 1;  // 发送完成标志位，1：发送完成，可以进行下一次发送。0：没有发送完成
volatile uint16_t receivedBytes = 0;

/**
  * @brief  Start USART1 DMA Receive
  * @param  pData: Pointer to data buffer
  * @param  Size: Amount of data to be received
  * @retval None
  */
static void USART1_Start_DMA_Receive(uint8_t *pData, uint16_t Size)
{
    if (HAL_UART_Receive_DMA(&huart1, pData, Size) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief  Start USART1 DMA Transmit
  * @param  pData: Pointer to data buffer
  * @param  Size: Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef USART1_Start_DMA_Transmit(uint8_t *pData, uint16_t Size)
{
    if (flagTxComplete == 0)
    {
        // 上一次传输还未完成
        return HAL_BUSY;
    }

    flagTxComplete = 0;  // 清除标志位
    return HAL_UART_Transmit_DMA(&huart1, pData, Size);
}

/**
  * @brief  UART IDLE callback
  * @param  huart: UART handle
  * @retval None
  */
void USER_UART_IDLE_Callback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) // 判断是不是空闲中断
        {
            // 清除空闲中断标志
            __HAL_UART_CLEAR_IDLEFLAG(huart);
            // 停止DMA接收
            HAL_UART_DMAStop(huart);
            // 计算接收到的字节数
            receivedBytes = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);

            // 打印调试信息
            char tempBuffer[RX_BUFFER_SIZE + 1];
            memcpy(tempBuffer, (char*)rxData, receivedBytes);
            tempBuffer[receivedBytes] = '\\0'; // 确保字符串以空字符结尾
            normal_info("Received data: %s, Bytes: %d\\n", tempBuffer, receivedBytes);
            
            // 处理接收到的数据，将数据放入ringbuffer或者RTOS的消息队列，不要在中断里解析报文，并运行其他程序
            
            // 可以根据项目需求添加数据处理逻辑
            USART1_Start_DMA_Receive((uint8_t*)rxData, RX_BUFFER_SIZE);
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // 发送完成后的处理逻辑，可以根据项目需求添加
        flagTxComplete = 1; // 设置标志位，指示传输完成
    }
}

void USART1_Drive_Init(void)
{
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); // 开启接收空闲中断
    USART1_Start_DMA_Receive((uint8_t*)rxData, RX_BUFFER_SIZE); // 启动DMA接收
}

```

### 2.2、usart1.h

```c
#ifndef __USART1_DRIVE_H
#define __USART1_DRIVE_H

#include "stm32f4xx_hal.h"
#include "main.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#ifdef __cplusplus
extern "C" {
#endif

HAL_StatusTypeDef USART1_Start_DMA_Transmit(uint8_t *pData, uint16_t Size);
void USART1_Drive_Init(void);

// 中断回调
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void USER_UART_IDLE_Callback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
} 
#endif

#endif /* __USART1_DRIVE_H */

```

## 三、main.c

---

```c
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "EventRecorder.h"
#include "usart1_drive.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t txData[] = "Hello, RFID!";  // 用于DMA发送的数据
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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  EventRecorderInitialize(EventRecordAll,1U);
  EventRecorderStart();
  
  USART1_Drive_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_GPIO_TogglePin(LD1_GPIO_Port,LD1_Pin);
    HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
    HAL_GPIO_TogglePin(LD3_GPIO_Port,LD3_Pin);
    //normal_info("LED is running.\\r\\n");
    if (HAL_BUSY == USART1_Start_DMA_Transmit(txData, sizeof(txData) - 1)) {
        normal_info("USART1 TX: HAL_BUSY\\n");
    }
    HAL_Delay(300);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\\r\\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

```

## 四、stm32f4xx_it.c

---
![[Pasted image 20240817112522.png]]
## 五、串口工具调试

---
![[Pasted image 20240817112606.png]]
![[Pasted image 20240817112622.png]]


## 六、细节补充

---

### 6.1、怎样判断一次收到的字符串长度超过缓存区的长度？

暂时没有找到方法解决。我觉得可靠的方法是

- 尽可能地增大缓存区，尽可能确保每一次都能容纳所有接收到的报文。
- 必须使用ringbuffer（裸机），或者使用RTOS的消息队列。尽可能地减少在中断处理的时间。

### 6.2、本例程有bug，DMA发送会有问题
![[Pasted image 20240829191122.png]]
如上图所示，HAL_UART_DMAStop(huart)会中止DMA接收的同时，又会终止DMA发送。导致DMA发送完成中断将无法进入。思考了两天，终于找到解决方案。
思路来自如下这个教学视频：
![[1623908199-1-192.mp4]]

![[Pasted image 20240829192425.png]]
![[Pasted image 20240829192514.png]]
如上图所示：
- 当接收标志位被置位，表示DMA已经搬运了数据。
- 也要查看DMA是否在发送，没有发生的话才会进入处理报文。

![[Pasted image 20240829192646.png]]
如上图所示：
- 发送的时候，也要判断DMA是不是正在发送；
- 发送的时候，要判断ringbuffer里有没有数据；


