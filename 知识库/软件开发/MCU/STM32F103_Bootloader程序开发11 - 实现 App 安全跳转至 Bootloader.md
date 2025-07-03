
## 导言
---
![[Pasted image 20250703230116.png]]
本教程使用正点原子战舰板开发。

想象一下，我们的单片机 App 正在稳定地运行着，突然我们想给它升级一下，添加个新功能。我们该如何安全地通知它："嘿，准备好接收新固件了" ? 这就需要 App 和 Bootloader 之间建立一个可靠的"秘密握手"机制。

整个流程就像一次精心策划的"交接仪式"：
1.  **外部通知**：上位机（比如我们的电脑）通过串口向正在运行的 App 发送一个预设好的"升级"指令。
2.  **设置信物**：App 收到指令后，并不会立刻停止工作，而是在一块共享的 RAM 内存中写入一个"暗号"（我们称之为"升级标志位"）。这块内存就像是 App 留给 Bootloader 的一个"秘密信箱"。
3.  **主动重启**：留下"暗号"后，App 主动触发一次软件复位，开始"交接"。
4.  **验证信物**：系统重启后，控制权首先交给 Bootloader。Bootloader 会立刻检查那个"秘密信箱"。
    *   如果发现了预设的"暗号"，它就知道："哦，App 需要我来执行升级任务"，随即进入 IAP（In-Application Programming）模式，准备接收新固件。
    *   如果信箱是空的，它就认为一切正常，直接跳转到 App，让 App 像往常一样运行。

![[Pasted image 20250701174048.png]]

为了简化演示，我们约定，当 App 的串口收到字符串 `A5A5A5A5` 时，就触发这次跳转。

![[Pasted image 20250701190733.png]]

> **友情提示**：在真实的商业项目中，简单的字符串匹配是远远不够的。为了保证通信的绝对可靠，防止数据干扰导致意外的固件升级，我们必须引入 **CRC 校验** 等错误检测机制。这里的 `A5A5A5A5` 只是一个为了教学目的而简化的触发信号。

**项目地址：**
*   **Gitee (国内推荐)**: [https://gitee.com/wallace89/MCU_Develop/tree/main/bootloader11_stm32f103_app_jump_boot](https://gitee.com/wallace89/MCU_Develop/tree/main/bootloader11_stm32f103_app_jump_boot)
*   **GitHub**: [https://github.com/q164129345/MCU_Develop/tree/main/bootloader11_stm32f103_app_jump_boot](https://github.com/q164129345/MCU_Develop/tree/main/bootloader11_stm32f103_app_jump_boot)

## 一、App 端代码
---

现在，我们来看看 App 程序需要增加哪些代码来实现这个"交接仪式"。

### 1.1、 jump_boot.c

这个文件负责处理从 App 跳转到 Bootloader 的所有核心逻辑。

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

**代码亮点解析:**
1.  **`update_flag` 变量的定义**：
    *   **`__attribute__((at(...)))`**: 这是告诉编译器："请务必把 `update_flag` 这个变量放在 `FIRMWARE_UPDATE_VAR_ADDR` 这个内存地址上"。这就像给 App 和 Bootloader 一个共同的、固定的"信箱"地址，确保双方都能找到对方。
    *   **`volatile`**: 这个关键字至关重要。它告诉编译器："这个变量的值随时可能被意想不到的方式改变（比如被硬件或其他程序），所以你不要自作聪明地去优化它。每次使用它的时候，都必须老老实实地从内存里重新读取"。这可以防止 App 重启后，Bootloader 读到一个被缓存的、不正确的值。

2.  **`IAP_Parse_Command` 函数**：
    *   这个函数就像一个"哨兵"，时刻监听着串口发来的每一个字节。
    *   它内部的 `match_index` 就像一个进度条，记录着"秘密口令"的匹配进度。每收到一个正确的字符，进度条就加一；一旦收到错误的字符，进度条就清零重来，非常严谨。
    *   当8个字符全部匹配成功，就意味着"口令正确"，哨兵就会立刻执行预设的三个步骤：**写标志、延时、复位**。

### 1.2、jump_boot.h

头文件很简单，主要是声明 `IAP_Parse_Command` 函数，以便在其他文件中（比如串口中断服务程序中）调用它。

```c
/**
 * @file    jump_boot.h
 * @brief   应用程序与Bootloader跳转功能的头文件声明
 * @author  Wallace.zhang
 * @date    2025-05-25
 * @version 1.0.0
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
  * @brief  解析串口接收到的数据，判断是否需要跳转到Bootloader
  * @param  data 接收到的单个字节
  */
void IAP_Parse_Command(uint8_t data);

#ifdef __cplusplus
}
#endif

#endif /* __JUMP_BOOT_H */
```

### 1.3、myUsartDrive_reg.c
![[Pasted image 20250701191758.png | 1100]]
如上图所示，在USART1的接收ringbuffer里，一个一个字节拿出去解析。

## 二、功能测试
---
现在，让我们来验证一下"交接仪式"是否能顺利进行。

### 2.1、测试 App 跳转到 Bootloader
我们使用一个 Python 脚本来模拟上位机，向单片机发送"秘密口令"。
![[JUMP_BOOT1.gif]]

**操作步骤：**
1.  将 App 程序下载到 STM32 开发板并运行。
2.  在项目 `iap_py` 文件夹下打开终端，运行以下指令（请根据你的实际情况修改 COM 口）：
    ```sh
    python3 jump_command.py --port COM8 --baud 115200
    ```
3.  观察 RTT Viewer 的日志。Bootloader 的日志出现，这表明 App 成功复位并跳转到了 Bootloader！

![[Pasted image 20250701193338.png]]

> **`jump_command.py` 脚本是做什么的？**
> 它其实很简单，就是通过指定的串口（`--port`）和波特率（`--baud`），向上位机发送了一串文本 `A5A5A5A5`。

**脚本参数说明:**
*   `--port`: 你的设备所连接的串口号 (必填)，例如 `COM8` (Windows) 或 `/dev/ttyUSB0` (Linux)。
*   `--baud`: 波特率 (可选)，默认为 `115200`。





