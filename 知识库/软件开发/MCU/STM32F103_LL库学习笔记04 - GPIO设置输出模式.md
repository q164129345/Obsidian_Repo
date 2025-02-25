# 导言
---
[[STM32F103_LL库学习笔记03 - GPIO设置输入模式，并轮询GPIO的电平状态]]，上一章节完成GPIO输入模式的设置。接下來梳理GPIO的输出模式配置，并且控制GPIO输出高电平或低电平。
首先，先梳理一下LL库怎样去设置GPIO的输出模式与控制GPIO输出高电平或低电平。

# 一、CubeMX
---
![[Pasted image 20250225195917.png | 1100]]
![[Pasted image 20250225200152.png | 1100]]
如上图所示，设置PB5为输出的推挽输出模式，下拉，初始化后输出低电平，控制频率Low（2M）。

# 二、代码
---
## 2.1、MX_GPIO_Init()
![[Pasted image 20250225204508.png | 800]]
如上所示，还是使用结构体`LL_GPIO_InitTypeDef`来填写GPIO的功能，然后通过`LL_GPIO_Init()`初始化GPIO。

## 2.2、控制GPIO输出高电平或低电平
LL库的如下函数用于控制GPIO输出：
- `LL_GPIO_SetOutputPin(GPIOx, Pin)`：将指定引脚置为高电平。
- `LL_GPIO_ResetOutputPin(GPIOx, Pin)`：将指定引脚置为低电平。
- `LL_GPIO_TogglePin(GPIOx, Pin)`：翻转指定引脚的电平状态（高变低，低变高）。

![[Pasted image 20250225214301.png | 800]]
如上所示，在`main()`的`while(1)`死循环里，通过`LL_GPIO_SetOutputPin`与`LL_GPIO_ResetOutputPin`控制PB5电平，1S高，1S低，一直轮询。
![[Pasted image 20250225214524.png | 800]]
在SysTick中断回调里查询PB5的状态（高还是低），GPIO不管是输入模式还是输出模式，都能通过寄存器IDR获取当前的电平状态。
