# 导言
---
![[Pasted image 20250701174048.png]]
流程始于一个外部触发事件：上位机向正在运行的App程序发送了升级指令。App在接收到指令后，会执行两个关键动作：首先在共享内存区写入一个特定的“升级标志”，然后主动触发系统软件复位。系统重启后，Bootloader检测到这个“升级标志”，便知道需要进入IAP升级模式。

![[Pasted image 20250701190733.png]]
如上所示，为了让程序尽可能简单。当App程序收到IAP上位机的字符串“A5A5A5A5"时，触发App跳转bootloader。

> IAP升级请求，大家可以根据自己的协议来修改，实战项目一定要增加CRC校验，保证串口通讯的稳定性。我是为了程序简单，使用字符串“A5A5A5A5"来触发跳转而已。

项目地址：  
github: https://github.com/q164129345/MCU_Develop/tree/main/bootloader11_stm32f103_app_jump_boot
gitee(国内): https://gitee.com/wallace89/MCU_Develop/tree/main/bootloader11_stm32f103_app_jump_boot

# 一、App代码（不是bootloader代码）
---
## 1.1、jump_boot.c
```c
#include "bootloader_define.h"
#include "flash_map.h"
#include "jump_boot.h"


#if defined(__IS_COMPILER_ARM_COMPILER_5__)
volatile uint64_t update_flag __attribute__((at(FIRMWARE_UPDATE_VAR_ADDR), zero_init));

#elif defined(__IS_COMPILER_ARM_COMPILER_6__)
    #define __INT_TO_STR(x)     #x
    #define INT_TO_STR(x)       __INT_TO_STR(x)
    volatile uint64_t update_flag __attribute__((section(".bss.ARM.__at_" INT_TO_STR(FIRMWARE_UPDATE_VAR_ADDR))));

#else
    #error "variable placement not supported for this compiler."
#endif

/**
  * @brief  获取固件升级标志位
  * @note   读取保存于指定RAM地址的升级标志变量（通常用于判断bootloader的运行状态）
  * @retval uint64_t 固件升级标志的当前值
  */
static uint64_t IAP_GetUpdateFlag(void)
{
    return update_flag;
}

/**
  * @brief  设置固件升级标志位
  * @param  flag 需要设置的标志值
  * @note   修改指定RAM地址的升级标志变量
  * @retval 无
  */
static void IAP_SetUpdateFlag(uint64_t flag)
{
    update_flag = flag;
}

/**
  * @brief  解析串口接收到的数据
  * @note   根据接收到的数据，判断是否需要跳转到Bootloader
  *         目标字符串: "A5A5A5A5" (8字节)
  *         ASCII: 0x41,0x35,0x41,0x35,0x41,0x35,0x41,0x35
  * @param  data 接收到的数据
  * @retval 无
  */
void IAP_Parse_Command(uint8_t data)
{
    // 目标字符串"A5A5A5A5"的ASCII码序列
    static const uint8_t target_sequence[8] = {
        0x41, 0x35, 0x41, 0x35,  // "A5A5"
        0x41, 0x35, 0x41, 0x35   // "A5A5"
    };
    
    // 静态变量：记录当前匹配的字节位置
    static uint8_t match_index = 0;
    
    // 检查当前字节是否与目标序列匹配
    if (data == target_sequence[match_index]) {
        match_index++;  // 匹配成功，移动到下一个位置
        
        // 检查是否接收完整的8字节序列
        if (match_index >= sizeof(target_sequence)) {
            // 完整匹配成功，跳转到Bootloader
            match_index = 0;  // 重置状态，为下次做准备
            //! 设置固件升级标志位
            IAP_SetUpdateFlag(FIRMWARE_UPDATE_MAGIC_WORD);
            //! 等待10ms，确保标志位设置成功
            LL_mDelay(10);
            //！此函数不会返回，MCU将复位
            NVIC_SystemReset();
        }
    } else {
        // 不匹配，重置状态机
        match_index = 0;
        
        // 特殊处理：如果当前字节恰好是序列的第一个字节，则开始新的匹配
        if (data == target_sequence[0]) {
            match_index = 1;
        }
    }
}
```

![[Pasted image 20250701191239.png | 1100]]
如上所示，jump_boot.c的绝大部分代码跟bootloader程序的jump_app.c一模一样。本章节的重点是`IAP_Parse_command(uint8_t data)`函数。

## 1.2、jump_boot.h
```c
/**
 * @file    jump_boot.h
 * @brief   应用程序与Bootloader跳转功能的头文件声明
 * @author  Wallace.zhang
 * @date    2025-05-25
 * @version 1.0.0
 * @copyright
 * (C) 2025 Wallace.zhang. 保留所有权利.
 * @license SPDX-License-Identifier: MIT
 */

#ifndef __JUMP_BOOT_H
#define __JUMP_BOOT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"
#include <stdbool.h>

/**
  * @brief  解析串口接收到的数据
  */
void IAP_Parse_Command(uint8_t data);



#ifdef __cplusplus
}
#endif

#endif /* __JUMP_BOOT_H */

```

## 1.3、myUsartDrive_reg.c
![[Pasted image 20250701191758.png | 1100]]

# 二、测试代码
----
## 2.1、测试App跳转bootloader
![[JUMP_BOOT1.gif]]
如上所示，在工程目录下的文件夹iap_py里，使用终端指令`python3 jump_command.py --port COM8 --baud 115200`运行python程序jump_command.py，可以让App程序跳转运行bootloader程序。
从RTT Viwer打印的log看来，App成功地跳转bootloader程序。

![[Pasted image 20250701193338.png]]

> 如上所示，指令python3 jump_command.py --port COM8 --baud 115200的本质是在端口COM8上，使用波特率115200的通讯频率来发送字符串“A5A5A5A5”而已。

**jump_command.py参数：**
- `--port` - 串口号（必需），如COM8
- `--baud` - 波特率，默认115200





