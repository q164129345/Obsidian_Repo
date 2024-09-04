## 导言
---
本次实验的效果是BOOT按钮按下后，触发GPIO0的外部中断，捕捉上升沿与下降沿。通过串口将捕捉到的上升沿与下降沿的总数打印出来。
![[Pasted image 20240904203514.png]]
串口打印的效果如下：
![[Pasted image 20240904203557.png]]
源码地址：[https://github.com/q164129345/esp32_Learning/tree/main/code_EXIT](https://github.com/q164129345/esp32_Learning/tree/main/code_EXIT)

## 一、exit.c
---
如下图所示，调用函数`GPIO_Exit_Init()`就能完成外部中断的设置，并关联好中断回调函数`exit_GPIO_ISR_Handler()`。
![[Pasted image 20240904203625.png]]


调用`setup_GPIO_Glitch_Filter()`使用毛刺过滤器，能提高电气抗干扰的能力。但是，它是可选项。

`gpio_install_isr_service()`用于注册中断服务，入口参数有很多选择。据说，直接把0扔进去足够满足大部分的应用，官方的例程这样用`gpio_install_isr_service(0)`。
```c
//Keep the LEVELx values as they are here; they match up with (1<<level)
#define ESP_INTR_FLAG_LEVEL1        (1<<1)  ///< Accept a Level 1 interrupt vector (lowest priority)
#define ESP_INTR_FLAG_LEVEL2        (1<<2)  ///< Accept a Level 2 interrupt vector
#define ESP_INTR_FLAG_LEVEL3        (1<<3)  ///< Accept a Level 3 interrupt vector
#define ESP_INTR_FLAG_LEVEL4        (1<<4)  ///< Accept a Level 4 interrupt vector
#define ESP_INTR_FLAG_LEVEL5        (1<<5)  ///< Accept a Level 5 interrupt vector
#define ESP_INTR_FLAG_LEVEL6        (1<<6)  ///< Accept a Level 6 interrupt vector
#define ESP_INTR_FLAG_NMI           (1<<7)  ///< Accept a Level 7 interrupt vector (highest priority)
#define ESP_INTR_FLAG_SHARED        (1<<8)  ///< Interrupt can be shared between ISRs
#define ESP_INTR_FLAG_EDGE          (1<<9)  ///< Edge-triggered interrupt
#define ESP_INTR_FLAG_IRAM          (1<<10) ///< ISR can be called if cache is disabled
#define ESP_INTR_FLAG_INTRDISABLED  (1<<11) ///< Return with this interrupt disabled
```
![[Pasted image 20240904203654.png]]


如下图所示，每进入一次中断回调函数，变量callback_CNT++。
![[Pasted image 20240904203709.png]]

## 二、main.c
---
main.c代码简单，调用GPIO_Exit_Init()初始化GPIO0的外部中断即可。
![[Pasted image 20240904203737.png]]

## 三、细节补充
---
### 3.1、解决添加了宏`IRAM_ATTR`导致编译失败
首先，确保头文件。

```c
#include "esp_attr.h"
```
然后，确保IRAM_ATTR的位置，必须在void的前面。
```c
static IRAM_ATTR void exit_GPIO_ISR_Handler(void *arg)
```

### 3.2、中断回调函数里不能使用printf
会卡死程序。