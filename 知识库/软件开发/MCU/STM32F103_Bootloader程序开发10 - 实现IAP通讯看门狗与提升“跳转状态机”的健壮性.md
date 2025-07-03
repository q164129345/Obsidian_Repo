# 导言
----
![[Pasted image 20250703230116.png]]
本教程使用正点原子战舰板开发。

在嵌入式系统的生命周期中，固件更新（IAP，In-Application Programming）是不可或缺的一环。一个设计精良的Bootloader不仅是产品迭代的基石，更是系统稳定性的最后一道防线。在本系列之前的文章中，我们已经成功实现了通过Ymodem协议进行固件的下载、校验与搬运。

然而，一个真正“生产级”的Bootloader还需要处理一些更为棘手的问题：`当IAP上位机请求MCU进入bootloader程序后，如何确保bootloader在各种异常情况下都不会"变砖"？`这个问题看似简单，背后却隐藏着巨大的风险。想象一下，当现场的设备收到一条进入IAP模式的指令后，它跳转到了Bootloader。但此时，如果上位机软件意外崩溃、网络连接中断，或是操作人员不慎关闭了升级工具，设备会发生什么？它将永远地停留在Bootloader中，像一块焦急等待指令却永远等不到的"砖头"，无法恢复正常工作，直到有人到现场手动复位。

为了避免这种灾难性的后果，我们必须为Bootloader装上一颗更加智能的"大脑"。这颗"大脑"需要具备看门狗一样的警觉性，能够在漫长的等待中保持耐心，又能在真正的通信中断时果断决策，安全地返回App。本文将详细探讨如何实现一个具备通信看门狗功能（通讯超时倒计时）的IAP机制，并加固"跳转状态机"的健壮性，确保我们的Bootloader在任何异常情况下都能做出最正确的选择，真正做到万无一失。

项目地址：  
github: https://github.com/q164129345/MCU_Develop/tree/main/bootloader10_stm32f103_iap_done
gitee(国内): https://gitee.com/wallace89/MCU_Develop/tree/main/bootloader10_stm32f103_iap_done

# 一、启动场景
---
![[Pasted image 20250630191223.png]]
如上所示，展示了引导加载程序（Bootloader）的两种主要启动场景：
1. 正常上电启动
	- 触发条件：这是最常规的启动流程。当设备上电、硬件复位或发生任何非预期的系统重置时，都会进入这个路径。
2. IAP升级请求
	- 触发条件：这是专为固件更新设计的特殊路径。当设备正在运行主应用程序（App）时，它通过串口、CAN总线或其他通信方式，接收到了来自上位机（PC端工具）的特定“IAP升级请求”指令。

总结来说，这个框图展示了一个健壮的Bootloader设计思想：所有启动路径都必须先经过Bootloader，由它来统一决策系统的下一个状态，从而确保了系统在任何情况下都有一个明确、可靠的入口点。

## 1.1、正常上电启动
![[Pasted image 20250630192147.png]]
该流程图展示了当系统在没有明确IAP升级请求的情况下（例如正常上电或硬件复位）启动时，Bootloader程序如何通过检查应用程序（App）的有效性来决定下一步操作，并引入了超时机制以增强系统的健壮性。

1. **启动与执行**
	系统启动后，控制权首先交给Bootloader。Bootloader作为系统的引导者，开始执行其预设的启动检查任务。

2. **核心决策点：检查App有效性**
	这是整个流程的核心决策环节。Bootloader会通过一系列技术手段（例如检查向量表、CRC校验等）来判断存储在Flash中的App固件是否完整、合法、可执行。这个检查会产生两种截然不同的结果：

3. **分支一：App有效**
	当Bootloader确认存在一个有效的App时，它并不会立即跳转。它会认为这可能是一次意外的重启。Bootloader会启动一个短暂的倒计时（例如10秒）。在这个倒计时期间，系统会停留在Bootloader中，并监听是否有来自上位机的IAP命令。如果在倒计时结束前没有收到任何IAP指令，Bootloader便会认为这是一次正常的启动，随即安全地跳转到主应用程序（App），系统恢复正常工作。这个倒计时机制为用户提供了一个进入IAP模式的“机会窗口”，同时保证了在无操作情况下系统能够自动恢复。

4. **分支二：App无效**
	当Bootloader发现App固件不存在、已损坏或不合法时，系统已经无法正常工作。此时，唯一的出路就是进行固件升级来修复系统。Bootloader会禁用任何超时或自动跳转机制。它会坚定地停留在IAP模式，并持续监听通信端口。设备会一直运行在Bootloader中，直到它成功接收并安装一个有效的App固件。这是一种必要的“故障保护”模式，确保了即使在主程序损坏的情况下，设备依然保留了被修复的能力，避免了“变砖”的风险。

总结来说，一个高度健壮和智能的Bootloader设计理念：根据App的完整性来动态调整自身的行为策略。它既能在App正常时提供便捷的升级入口和自动恢复能力，又能在App损坏时坚守阵地，确保系统总能通过IAP升级得以重生。

## 1.2、IAP升级请求
![[Pasted image 20250630192701.png]]
该流程图展示了当App程序响应上位机的指令，主动请求进入IAP模式后的完整处理逻辑。`其核心思想是引入一个“通信看门狗”式的倒计时机制，以应对升级过程中可能出现的各种通信异常。`

1. **请求与跳转**
	流程始于一个外部触发事件：上位机向正在运行的App程序发送了升级指令。App在接收到指令后，会执行两个关键动作：首先在共享内存区写入一个特定的“升级标志”，然后主动触发系统软件复位。系统重启后，Bootloader检测到这个“升级标志”，便知道需要进入IAP升级模式。

2. **核心机制：启动倒计时（通信看门狗）**
	进入IAP模式后，Bootloader并不会无限期地盲目等待。它会立即启动一个倒计时器（例如10秒）。这个倒计时器扮演着“通信看门狗”的角色，它假设在接下来的10秒内必须检测到有效的IAP通信活动。

3. **三种并发场景与处理**
	1. 收到IAP升级报文，重置倒计时
		这是最理想的正常通信场景。上位机开始发送数据（例如Ymodem协议的起始信号或数据包），Bootloader成功接收到这些报文。每当Bootloader收到一个有效的数据包，它就会重置倒计时器（俗称`“喂狗”`）。这个动作表明通信是活跃的，上位机和设备之间的连接是正常的。只要通信持续进行，倒计时就会被不断重置，永远不会达到超时，从而保证了升级流程可以顺利进行，不受超时机制的干扰。
	2. IAP升级正常，搬运、跳转新App
		在通信看门狗的保护下，整个IAP升级流程（包括固件接收、数据校验、Flash擦写）顺利完成。Bootloader在确认新固件已经完整无误地写入Flash后，会执行搬运操作（如果需要的话），然后跳转到新的App程序开始运行。
	3. IAP升级异常，倒计时结束，跳转App
		这是`关键的异常处理场景`。可能的原因包括：上位机在发送完IAP请求后就崩溃了；网络中断；串口线在中途被拔掉；或者上位机发送的数据格式错误，Bootloader无法识别。由于Bootloader在设定的时间内（例如10秒）没有收到任何有效的IAP报文来“喂狗”，倒计时器最终会计时结束。：Bootloader会判断发生了通信超时。此时，为了防止设备“变砖”，它会放弃本次升级，并`自动跳转回原来的App程序。`这个“安全返回”机制是整个系统健壮性的核心保障。

总结来说，这个流程图展示了一个非常成熟和可靠的IAP设计。它通过一个简单的倒计时器，巧妙地实现了一个功能强大的通信看门狗，确保了无论升级过程是顺利、中断还是从未开始，系统总能处于一个安全、可恢复的状态。

# 二、代码
---
## 2.1、boot_entry.c
```c
#include "boot_entry.h"
#include "flash_map.h"
#include "app_jump.h"
#include "bootloader_define.h"

/**
 * @brief   更完善的App有效性检查
 */
bool Is_App_Valid_Enhanced(uint32_t app_start_addr)
{
    uint32_t app_stack_ptr = *(volatile uint32_t*)app_start_addr; //! 获取栈指针
    uint32_t app_reset_vec = *(volatile uint32_t*)(app_start_addr + 4); //! 获取复位向量
    //! 检查1：栈指针必须在RAM范围内
    if ((app_stack_ptr < 0x20000000) || (app_stack_ptr > 0x20020000)) {
        return false; //! 栈指针不在RAM范围内
    }
    //! 检查2：复位向量必须是Thumb指令（奇数）
    if ((app_reset_vec & 1) == 0) {
        return false; //! 复位向量不是Thumb指令
    }
    //! 检查3：复位向量必须在Flash范围内
    if ((app_reset_vec < app_start_addr) || 
        (app_reset_vec > (app_start_addr + FLASH_APP_SIZE))) {
        return false; //! 复位向量不在Flash范围内
    }
    //! 检查4：不能是空Flash的特征值
    if ((app_stack_ptr == 0xFFFFFFFF) || (app_reset_vec == 0xFFFFFFFF)) {
        return false; //! 栈指针或复位向量是空Flash的特征值
    }
    //! 可选：检查5：CRC校验（最可靠但最慢）
    //! if (!FW_Verify_CRC32(FLASH_APP_START_ADDR, app_size)) {
    //!     return false;
    //! }
    return true;
}

/**
  * @brief  上电，系统早期初始化回调（main()前自动调用）
  * @note
  *   - 本函数通过 GCC/ARMCC 的 @c __attribute__((constructor)) 属性修饰，系统启动后、main()执行前自动运行。
  *   - 适用于进行早期日志重定向、环境检测、固件完整性校验、启动标志判断等全局初始化操作。
  *   - 随项目进展，可逐步完善本函数内容，建议仅放置不依赖外设初始化的关键代码。
  *
  * @attention
  *   - 使用 @c __attribute__((constructor)) 机制要求工程链接脚本/启动文件正确支持 .init_array 段。
  *   - 若编译器或启动脚本不支持该机制，请将该函数内容手动放入 main() 最前面调用。
  *
  * @see    log_printf()
  */
__attribute__((constructor))
static void _SystemStart(void)
{
    //! 不要在构造函数中调用RTT重定向printf，例如调用Retarget_RTT_Init()。
    //! 只要在bootloader或者App的main()中调用Retarget_RTT_Init()。这个函数里的log_printf()会正常打印出来。
    uint64_t flag = IAP_GetUpdateFlag();

    if (flag == BOOTLOADER_RESET_MAGIC_WORD) {
        //! --- Case: Bootloader 内部跳转流程 ---
        //! 这是 IAP_Ready_To_Jump_App 的第二步，需要立即跳转，不进入main()
        log_printf("Bootloader internal jump detected, jumping to app immediately. flag=0x%08X%08X\n", \
        (uint32_t)(flag >> 32), (uint32_t)flag);
        IAP_Ready_To_Jump_App(); // 此函数不会返回
    } 
    
    //! 所有其他情况（包括App请求IAP、正常启动、App无效等）都交给main()处理
    //! 不清除标志，让main()能够正确判断启动原因
    log_printf("Proceeding to main() for detailed boot analysis.\n");
}


```
![[Pasted image 20250630193741.png | 1100]]
如上所示，这个新增函数的目的是“判断App区的App程序是否有效”。后续，大家尽情发挥自己的想象力去优化它，改进它的玩法！

## 2.2、main.c
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
#include "bsp_usart_hal.h"
#include "retarget_rtt.h"
#include "app_jump.h"
#include "soft_crc32.h"
#include "op_flash.h"
#include "fw_verify.h"
#include "ymodem.h"
#include "boot_entry.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

//! USART1缓存 RX方向
uint8_t gUsart1RXDMABuffer[2048];
uint8_t gUsart1RXRBBuffer[2048];
//! USART1缓存 TX方向
uint8_t gUsart1TXDMABuffer[2048];
uint8_t gUsart1TXRBBuffer[2048];
//! 实例化Usart1
USART_Driver_t gUsart1Drv = {
    .huart = &huart1,
    .hdma_rx = &hdma_usart1_rx,
    .hdma_tx = &hdma_usart1_tx,
};

//! YModem协议处理器实例
YModem_Handler_t gYModemHandler;

//! 添加 IAP 完成延迟计数器，确保最终 ACK 有时间发送
static uint32_t iap_complete_delay_counter = 0;
static bool iap_complete_pending = false;

//! 添加 IAP 超时机制 - 根据App有效性决定是否启用
#define IAP_TIMEOUT_SECONDS     10
#define IAP_TIMEOUT_MS          (IAP_TIMEOUT_SECONDS * 1000)
static uint32_t iap_timeout_counter = 0;
static bool iap_timeout_enabled = false;  // 默认禁用，根据启动原因决定
static bool iap_communication_detected = false;

//! 保存启动原因的快照，用于后续超时决策
volatile static uint64_t gUpdateFlag = 0;
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
static void Timeout_Handler_MS(void);
static void Timeout_Counter_Reset(void);
static void Timeout_Counter_Enable(void); 

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
#if LOG_ENABLE
  Retarget_RTT_Init(); //! RTT重定向printf
#endif
  log_printf("Entering the main initialization of the bootloader.\n");
  uint32_t fre = 0;
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

  //! USART1初始化
  USART_Config(&gUsart1Drv,
               gUsart1RXDMABuffer, gUsart1RXRBBuffer, sizeof(gUsart1RXDMABuffer),
               gUsart1TXDMABuffer, gUsart1TXRBBuffer, sizeof(gUsart1TXDMABuffer));
  
  //! YModem协议处理器初始化（完全解耦版本）
  YModem_Init(&gYModemHandler);
  
  //! 启动原因分析和处理(超时机制)
  Timeout_Counter_Enable();

  log_printf("Bootloader init successfully.\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    Timeout_Handler_MS(); //! 超时处理

    //! 30ms
    if (0 == fre % 30) {
        HAL_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin); //! 心跳灯快闪（在bootlaoder程序里，心跳灯快闪。App程序，心跳灯慢闪。肉眼区分当前跑什么程序）
    }
    
    //! 2ms
    if (0 == fre % 2) {
        //! YModem协议处理 - 逐字节从ringbuffer里拿出数据来解析
        while(USART_Get_The_Existing_Amount_Of_Data(&gUsart1Drv)) {
            uint8_t data;
            if (USART_Take_A_Piece_Of_Data(&gUsart1Drv, &data)) {
                YModem_Run(&gYModemHandler, data); //! 运行YModem协议
                Timeout_Counter_Reset(); // 重置超时计数器，保持通信活跃
            }
        }

        //! 根据YModem协议的状态，处理IAP流程
        if (gYModemHandler.state == YMODEM_STATE_COMPLETE && !iap_complete_pending) {
            //! 第一次检测到完成状态，启动延迟计数器
            log_printf("YModem: IAP upgrade completed. Starting delay to ensure final ACK transmission...\r\n");
            iap_complete_pending = true;
            iap_complete_delay_counter = 0;
        } else if (iap_complete_pending) {
            //! 延迟期间，继续发送任何待发送的响应数据
            iap_complete_delay_counter++;
            
            //! 延迟足够时间确保最终ACK发送完成（大约500ms = 250次2ms周期）
            if (iap_complete_delay_counter >= 250) {
                log_printf("YModem: delay completed, starting firmware verification and copy...\r\n");
                uint32_t file_size = YMode_Get_File_Size(&gYModemHandler);
                log_printf("Firmware size: %d bytes.\r\n", file_size);
                if (HAL_OK == FW_Firmware_Verification(FLASH_DL_START_ADDR, file_size)) {
                    log_printf("CRC32 verification was successful\r\n");
                    if (OP_FLASH_OK == OP_Flash_Copy(FLASH_DL_START_ADDR, FLASH_APP_START_ADDR, FLASH_APP_SIZE)) { //! 将App下载缓存区的固件搬运到App区
                        log_printf("The firmware copy to the app area successfully.\r\n");
                        log_printf("Jump to the application.\r\n");
                        HAL_Delay(500);
                        IAP_Ready_To_Jump_App(); //! 跳转App
                    }
                } else {
                    log_printf("CRC32 verification failed\r\n");
                    //! 重要：重置状态和YModem处理器，准备下次传输
                    log_printf("YModem: resetting for next transmission...\r\n");
                    iap_complete_pending = false;
                    iap_complete_delay_counter = 0;
                    YModem_Reset(&gYModemHandler);
                }
            }
        } else if (gYModemHandler.state == YMODEM_STATE_ERROR) {
            log_printf("YModem: transmission error, reset protocol processor.\r\n"); //! 传输出错，重置协议处理器
            //! 重置延迟状态
            iap_complete_pending = false;
            iap_complete_delay_counter = 0;
            iap_communication_detected = false; //! 重置通信标志
            iap_timeout_counter = 0; //! 重置超时计数器
            YModem_Reset(&gYModemHandler);
        }

        //! 检查是否有YModem响应数据需要发送
        if (YModem_Has_Response(&gYModemHandler)) {
            uint8_t response_buffer[16];
            uint8_t response_length = YModem_Get_Response(&gYModemHandler, response_buffer, sizeof(response_buffer));
            if (response_length > 0) {
                //! 将响应数据发送给上位机
                USART_Put_TxData_To_Ringbuffer(&gUsart1Drv, response_buffer, response_length);
            }
        }
        
        USART_Module_Run(&gUsart1Drv); //! Usart1模块运行
    }
    
    fre++;
    HAL_Delay(1);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 * @brief   IAP超时处理函数
 * @note    每1ms调用一次，管理IAP模式下的超时倒计时
 * @details 
 *   - 持续监控通信状态，每秒显示剩余时间
 *   - 超时后根据启动原因智能处理：
 *     * App请求模式：返回App
 *     * 正常启动且App有效：跳转App
 *     * App无效：重置YModem等待新固件
 */
static void Timeout_Counter_Reset(void)
{
    if (iap_timeout_enabled) {
        iap_timeout_counter = 0; // 重置计数器
        if (!iap_communication_detected) {
            iap_communication_detected = true;
            log_printf("IAP communication established, timeout counter reset.\n");
        }
    }
}

/**
 * @brief   检测通信活跃性并重置超时计数器
 * @note    当检测到IAP通信时调用，重置超时机制确保持续通信
 * @details 
 *   - 仅在超时机制启用时生效
 *   - 首次检测到通信会标记通信状态并记录日志
 *   - 每次调用都会重置超时计数器到0
 */
static void Timeout_Counter_Enable(void)
{
    gUpdateFlag = IAP_GetUpdateFlag();
    bool app_valid = Is_App_Valid_Enhanced(FLASH_APP_START_ADDR);
    log_printf("=== Bootloader Boot Analysis ===\n");
    log_printf("Boot flag: 0x%08X%08X\n", (uint32_t)(gUpdateFlag >> 32), (uint32_t)gUpdateFlag);
    log_printf("App valid: %s\n", app_valid ? "YES" : "NO");
    
    if (gUpdateFlag == FIRMWARE_UPDATE_MAGIC_WORD) {
        //! App请求进入IAP模式
        IAP_SetUpdateFlag(0); // 清除RAM中的标志，避免重复触发
        log_printf("=== IAP Mode: App Request ===\n");
        log_printf("Timeout: ENABLED (%d seconds)\n", IAP_TIMEOUT_SECONDS);
        log_printf("Will return to App if no communication detected.\n");
        iap_timeout_enabled = true;
    } else {
        //! 正常启动
        IAP_SetUpdateFlag(0); // 清除可能存在的无效标志
        if (app_valid) {
            //! 正常启动，启用超时
            log_printf("=== IAP Mode: Normal Entry (App Valid) ===\n");
            log_printf("Timeout: ENABLED (%d seconds)\n", IAP_TIMEOUT_SECONDS);
            log_printf("Will return to App if no communication detected.\n");
            iap_timeout_enabled = true;
        } else {
            //! App无效，必须等待固件
            log_printf("=== IAP Mode: App Invalid ===\n");
            log_printf("Timeout: DISABLED\n");
            log_printf("Will wait indefinitely for firmware download.\n");
            iap_timeout_enabled = false;
        }
    }
}

/**
 * @brief   超时处理函数 - 管理IAP模式下的超时机制
 * @note    
 *   - 此函数每1ms被调用一次，用于实现IAP模式的倒计时和超时处理
 *   - 持续监控通信活跃状态，即使已建立通信也会继续计时
 *   - 超时时间由IAP_TIMEOUT_MS宏定义（默认10秒）
 *   - 每秒会打印一次剩余倒计时，帮助用户了解超时状态
 *
 * @details
 *   改进的超时机制工作流程：
 *   1. 检查是否启用超时（iap_timeout_enabled）
 *   2. 持续递增计数器，不管是否检测到通信
 *   3. 每1000ms（1秒）打印一次剩余时间提示
 *   4. 达到超时时间后，根据App有效性智能处理：
 *      - App有效：跳转到App程序
 *      - App无效：重置YModem协议，重新等待升级
 */
static void Timeout_Handler_MS(void)
{
    //! === IAP 超时检查逻辑（每1ms执行） ===
    if (iap_timeout_enabled) {
        iap_timeout_counter++; // 每1ms累加1
        
        // 每秒打印一次倒计时
        if (iap_timeout_counter % 1000 == 0) {
            uint32_t remaining_seconds = (IAP_TIMEOUT_MS - iap_timeout_counter) / 1000;
            const char* status = iap_communication_detected ? "Active" : "Waiting";
            log_printf("IAP timeout (%s): %d seconds remaining...\n", status, remaining_seconds);
        }
        
        // 超时检查
        if (iap_timeout_counter >= IAP_TIMEOUT_MS) {
            log_printf("IAP timeout reached! Handling based on boot reason...\n");
            log_printf("Original boot flag: 0x%08X%08X\n", 
                      (uint32_t)(gUpdateFlag >> 32), (uint32_t)gUpdateFlag);
            
            // 智能超时处理
            if (gUpdateFlag == FIRMWARE_UPDATE_MAGIC_WORD) {
                //! App请求进入IAP模式，App肯定有效
                log_printf("Boot reason: App request -> Jumping back to App...\n");
                HAL_Delay(100);
                IAP_Ready_To_Jump_App();
            } else {
                bool app_valid = Is_App_Valid_Enhanced(FLASH_APP_START_ADDR);
                if (app_valid) {
                    log_printf("Boot reason: Other, App valid -> Jumping to App...\n");
                    HAL_Delay(100);
                    IAP_Ready_To_Jump_App();
                } else {
                    log_printf("Boot reason: Other, App invalid -> Resetting YModem for retry...\n");
                    iap_complete_pending = false;
                    iap_complete_delay_counter = 0;
                    iap_communication_detected = false;
                    iap_timeout_counter = 0;
                    YModem_Reset(&gYModemHandler);
                    log_printf("YModem reset completed, waiting for new firmware...\n");
                }
            }
        }
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
![[Pasted image 20250630194431.png | 1100]]
![[Pasted image 20250630194536.png | 1100]]
![[Pasted image 20250630195920.png | 1100]]

# 三、测试程序
---
## 3.1、正常上电启动
### 3.1.1、App区没有合法App
![[Pasted image 20250701094939.png | 1000]]
使用J-Flash将MCU的所有固件擦除掉，然后只烧录bootloader。此时，MCU只有bootloader程序，App区与App缓存区都是空白的。如上所示，检验到App区没有合法的App，bootloader不会跳转至App，一直等待IAP升级请求。

接着，使用python3编写的IAP上位机跟MCU完成IAP升级。如下所示，IAP升级结束后，MCU成功跳转至App运行！
![[IAP_02.gif]]

### 3.1.2、App区有合法App
![[Pasted image 20250630201950.png | 1000]]
动态的效果：
![[IAP_01.gif]]

## 3.2、IAP升级请求
本章笔记主要针对bootloader的功能完善，IAP升级请求的跳转需要在App程序完成。所以，将在下一章节开发、验证这个功能。

## 3.3、IAP升级意外中断
这是`关键的异常处理场景`。可能的原因包括：上位机在发送完IAP请求后就崩溃了；网络中断；串口线在中途被拔掉；或者上位机发送的数据格式错误，Bootloader无法识别。由于Bootloader在设定的时间内（例如10秒）没有收到任何有效的IAP报文来“喂狗”，倒计时器最终会计时结束。：Bootloader会判断发生了通信超时。此时，为了防止设备“变砖”，它会放弃本次升级，并`自动跳转回原来的App程序。`这个“安全返回”机制是整个系统健壮性的核心保障。

![[IAP_03.gif]]
如上所示，在IAP升级发送第4包的时候，我强制退出IAP上位机。接着，bootloader程序会重新倒计时10S。倒计时结束后，重新回到之前的App上运行。

# 四、细节补充
----
## 4.1、bootloader调试完毕后，记得关闭RTT log打印
`我碰到一个问题，bootloader的RTT Viwer打印log开启后，App程序的RTT Viwer打印log会失效，不正常！` 针对这个问题，我暂时没有找到解决方案。所以，调试完bootloader程序后，记得关闭RTT log打印，避免App程序的RTT 打印异常。
![[Pasted image 20250701102210.png]]



