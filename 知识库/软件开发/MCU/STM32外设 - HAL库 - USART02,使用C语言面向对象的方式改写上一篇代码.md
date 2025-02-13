### 导言
----
[[STM32外设 - HAL库 - USART01,STM32F407,DMA接收空闲中断与DMA发送]]
在上一章节的基础上，改为面向对象的方式来重构代码。

**C语言使用面向对象的方法编写驱动代码的好处如下：**
1. 代码可读性和可维护性
面向对象的编程（OOP）通过将数据和操作数据的函数封装在一起，使代码更易于理解和维护。这样，代码的组织结构更清晰，更容易定位和修复错误。
2. 模块化和重用性
面向对象的方法鼓励代码模块化。对象和类可以独立开发、测试和复用。这种模块化设计使得代码更具灵活性，可以在不同项目中重复使用，减少了重复代码。
3. 封装性
OOP通过封装（Encapsulation）保护对象的内部状态，只允许通过定义好的接口访问对象数据。这样可以避免外部代码直接修改对象的内部状态，减少了错误的可能性，提高了代码的安全性和稳定性。
4. 继承性和扩展性
虽然C语言本身不支持继承，但通过结构体和函数指针可以模拟出类似的效果。这允许在不修改现有代码的情况下扩展功能，增强了代码的灵活性和可扩展性。
5. 多态性
通过函数指针和结构体，C语言可以实现多态性，这使得同一接口可以有不同的实现方式。 多态性提高了代码的灵活性，使得程序更容易扩展和修改。
6. 数据抽象
OOP允许创建抽象的数据类型，只暴露必要的接口，隐藏实现细节。这种数据抽象使得代码更简洁，减少了外部依赖，有利于代码的维护和更新。
7. 协作开发
面向对象的方法使得团队开发更为高效。每个开发者可以专注于不同的对象或类，相互之间的依赖减少，提高了开发效率。

### 一、usart_drive.c
----
```c
#include "usart_drive.h"
#include "usart.h"

/**
 * @brief 启动USART DMA接收
 * 
 * @param me usart_drive对象指针
 */
static void USART_Start_DMA_Receive(struct usart_drive* me)
{
    if (NULL == me) return;
    if (HAL_UART_Receive_DMA(me->huart, (uint8_t *)me->rxData, USART_RX_BUF_SIZE) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief 设置发送完成标志位
 * 
 * @param me usart_drive对象指针
 * @param flag 标志位值
 */
static void USART_Set_Flag_Tx_Complete(struct usart_drive* me, uint8_t flag)
{ 
    if (NULL == me) return; 
    me->flagTxComplete = flag > 0 ? true : false; 
}

/**
 * @brief 获取发送完成标志位
 * 
 * @param me usart_drive对象指针
 * @return uint8_t 标志位值
 */
static uint8_t USART_Get_Flag_Tx_Complete(struct usart_drive* me)
{ 
    if (NULL == me) return false;
    return me->flagTxComplete;
}

/**
 * @brief 启动USART DMA发送
 * 
 * @param me usart_drive对象指针
 * @param pData 指向数据缓冲区的指针
 * @param Size 数据大小
 * @return HAL_StatusTypeDef HAL状态
 */
static HAL_StatusTypeDef USART_Start_DMA_Transmit(struct usart_drive* me, uint8_t *pData, uint16_t Size)
{
    if (NULL == me) return HAL_ERROR;
    if (me->flagTxComplete == 0) {
        // 上一次传输还未完成
        return HAL_BUSY;
    }
    me->flagTxComplete = 0;  // 清除标志位
    return HAL_UART_Transmit_DMA(me->huart, pData, Size);
}

/**
 * @brief UART空闲中断回调函数
 * 
 * @param me usart_drive对象指针
 */
static void USER_UART_IDLE_Callback(struct usart_drive* me)
{
    if (NULL == me) return;
    if (me->huart->Instance == USART1)
    {
        if (RESET != __HAL_UART_GET_FLAG(me->huart, UART_FLAG_IDLE)) // 判断是不是空闲中断
        {
            // 清除空闲中断标志
            __HAL_UART_CLEAR_IDLEFLAG(me->huart);
            // 停止DMA接收
            HAL_UART_DMAStop(me->huart);
            // 计算接收到的字节数
            me->receivedBytes = USART_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(me->huart->hdmarx);
            
            // 检查接收到的字节数是否超过缓冲区大小
            if (me->receivedBytes > 0 && me->receivedBytes <= USART_RX_BUF_SIZE) {
                // 打印调试信息
                char tempBuffer[USART_RX_BUF_SIZE + 1];
                memcpy(tempBuffer, (unsigned char*)me->rxData, me->receivedBytes);
                tempBuffer[me->receivedBytes] = '\0'; // 确保字符串以空字符结尾
                normal_info("Received string: %s, Bytes: %d\n", tempBuffer, me->receivedBytes); // 打印字符串
                
                // 以16进制打印出来
                printf("Received data in hex: ");
                for (uint16_t i = 0; i < me->receivedBytes; i++) {
                    printf("%02X ", tempBuffer[i]);
                }
                printf("\n");
            } else {
                // 处理接收溢出情况
                normal_info("Received data overflow! Bytes: %d\n", me->receivedBytes);
                me->receivedBytes = 0; // 重置接收字节数
            }
            // 处理接收到的数据，将数据放入ringbuffer或者RTOS的消息队列，不要在中断里解析报文，并运行其他程序
            
            // 可以根据项目需求添加数据处理逻辑
            USART_Start_DMA_Receive(me); // 重新启动DMA接收
        }
    }
}

/**
 * @brief USART驱动对象初始化
 * 
 * @param me usart_drive对象指针
 * @param huart UART句柄
 */
void Usart_Drive_Object_Init(struct usart_drive* me, UART_HandleTypeDef *huart)
{
    if (huart == NULL) return;
    
    me->huart = huart; // 对象句柄
    
    me->flagTxComplete = 0x01;
    
    /* 对象方法初始化 */
    me->DMA_Sent = USART_Start_DMA_Transmit;
    me->Get_Flag_Tx_Complete = USART_Get_Flag_Tx_Complete;
    me->Set_Flag_Tx_Complete = USART_Set_Flag_Tx_Complete;
    me->User_IDLE_Callback = USER_UART_IDLE_Callback;
    
    __HAL_UART_ENABLE_IT(me->huart, UART_IT_IDLE); // 开启接收空闲中断
    USART_Start_DMA_Receive(me); // 启动DMA接收
}


```


### 二、usart_drive.h
----
```c
#ifndef _USART1_DRIVE_H_
#define _USART1_DRIVE_H_

#include "stm32f4xx_hal.h"
#include "main.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "stdbool.h"


#ifdef __cplusplus
extern "C" {
#endif

#define USART_RX_BUF_SIZE 255

typedef struct usart_drive{
    /* 成员 */
    UART_HandleTypeDef* huart;                  // HAL对象句柄
    volatile uint8_t    rxData[USART_RX_BUF_SIZE]; // DMA接收缓存区
    volatile uint8_t    flagTxComplete;         // 标志位：发送完成
    volatile int16_t    receivedBytes;         // 接收的字节数
    
    /* 方法 */
    void (*User_IDLE_Callback) (struct usart_drive* me); // 空闲中断处理
    HAL_StatusTypeDef (*DMA_Sent) (struct usart_drive* me, uint8_t *pData, uint16_t Size); // DMA发送
    
    // 设置器
    void (*Set_Flag_Tx_Complete) (struct usart_drive* me, uint8_t flag); // 设置标志位(发送完成）
    // 获取器
    uint8_t (*Get_Flag_Tx_Complete) (struct usart_drive* me); // 获取标志位（发送完成）
    
}Usart_Drive;

// 对象实例化
void Usart_Drive_Object_Init(struct usart_drive* me, UART_HandleTypeDef *huart);


#ifdef __cplusplus
} 
#endif



#endif /* __USART1_DRIVE_H */

```

### 三、main
----
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
#include "usart_drive.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern UART_HandleTypeDef huart1;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t txData[] = "Hello, RFID!";  // 用于DMA发送的数据
Usart_Drive g_Usart1;
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
  
  Usart_Drive_Object_Init(&g_Usart1, &huart1); // 实例化串口1
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
    //normal_info("LED is running.\r\n");
    if (HAL_BUSY == g_Usart1.DMA_Sent(&g_Usart1, txData, sizeof(txData) - 1)) {
        normal_info("USART1 TX: HAL_BUSY\n");
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

/**
 * @brief USART发送完成中断回调函数
 * 
 * @param me UART_HandleTypeDef对象指针
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // 发送完成后的处理逻辑，可以根据项目需求添加
        g_Usart1.Set_Flag_Tx_Complete(&g_Usart1, 0x01);
    }
}


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

### 四、stm32f4xx_it.c
----
```c
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart_drive.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */
extern Usart_Drive g_Usart1;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
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
  * @brief This function handles Pre-fetch fault, memory access fault.
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

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
  g_Usart1.User_IDLE_Callback(&g_Usart1); // USART接收空闲中断处理
  //USER_UART_IDLE_Callback(&huart1);
  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream7 global interrupt.
  */
void DMA2_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream7_IRQn 0 */

  /* USER CODE END DMA2_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA2_Stream7_IRQn 1 */

  /* USER CODE END DMA2_Stream7_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

```