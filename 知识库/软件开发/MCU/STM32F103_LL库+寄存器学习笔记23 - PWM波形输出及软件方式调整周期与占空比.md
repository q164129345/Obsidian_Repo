# 导言
---
脉宽调制（PWM）是 STM32 定时器最常用的输出模式之一，广泛应用于电机驱动、LED 调光、伺服控制和功率管理等场景。本篇文章将以 TIM5 为例，从寄存器层面深入剖析 PWM 输出的原理与实现步骤。通过本篇博客，你不仅能掌握 CubeMX 及 LL 库的调用，更能从底层寄存器视角构建完整的 PWM 输出思维，为后续复杂控制奠定坚实基础。
本章节使用TIM5生成周期为1ms的PWM波形，占空比50%。并介绍如何通过软件方式改变PWM波形的周期与占空比。

如图所示，PWM的频率是1kHz（周期1ms），占空比500us（50%）。
![[Pasted image 20250513152433.png | 1000]]

项目地址：
github:
- LL库: https://github.com/q164129345/MCU_Develop/tree/main/stm32f103_ll_library23_TIM_Generate_PWM
- 寄存器方式: https://github.com/q164129345/MCU_Develop/tree/main/stm32f103_reg_library23_TIM_Generate_PWM

gitee(国内):
- LL库: https://gitee.com/wallace89/MCU_Develop/tree/main/stm32f103_ll_library23_TIM_Generate_PWM
- 寄存器方式: https://gitee.com/wallace89/MCU_Develop/tree/main/stm32f103_reg_library23_TIM_Generate_PWM

# 一、LL库
---
## 1.1、CubeMX
![[Pasted image 20250513153026.png | 1000]]
![[Pasted image 20250513153130.png | 1000]]
![[Pasted image 20250513153226.png | 1000]]

## 1.2、tim.c
![[Pasted image 20250513153506.png | 1000]]
CubeMX 根据上述配置，自动生成了相应初始化代码。

## 1.3、main.c
![[Pasted image 20250513153649.png | 1000]]

## 1.4、编译、调试
![[Pasted image 20250513153726.png | 1000]]
编译通过。

## 1.5、修改PWM波形的周期与占空比
![[Pasted image 20250513154351.png | 1000]]
如上所示：
1. 函数`LL_TIM_OC_SetCompareCH1()`修改CH1的占空比，即TIM5_CCR1的值。
2. 函数`LL_TIM_SetAutoReload()`修改TIM5_ARR的值，改变TIM5的PWM周期。

![[Pasted image 20250513154829.png | 1000]]
如上所示，PWM的周期从原来的1ms变成2ms，占空比500us变成100us。所以，占空比  = 100us / 2000us = 5%。

# 二、寄存器方式
---
## 2.1、TIM5PWMOutput_reg.c
```c
#include "TIM5PWMOutput/TIM5PWMOutput_reg.h"

/**
 * @brief  启动 TIM5 通道1 的 PWM 输出
 * @retval 无
 */
void TIM5_PWM_Start(void)
{
    /* 使能 CC1 输出 */
    TIM5->CCER |= TIM_CCER_CC1E;
    /* 使能定时器计数 */
    TIM5->CR1 |= TIM_CR1_CEN;
    /* 立即生效：触发更新事件，加载预装载寄存器 */
    TIM5->EGR |= TIM_EGR_UG;
}

/**
 * @brief  停止 TIM5 通道1 的 PWM 输出
 * @retval 无
 */
void TIM5_PWM_Stop(void)
{
    /* 禁用定时器计数 */
    TIM5->CR1 &= ~TIM_CR1_CEN;
    /* 禁用 CC1 输出 */
    TIM5->CCER &= ~TIM_CCER_CC1E;
}

/**
 * @brief  设置 PWM 占空比（通道1）
 * @param  ccr: 比较寄存器值，范围 0 ~ (ARR+1)
 * @retval 无
 */
void TIM5_PWM_SetDuty(uint16_t ccr)
{
    TIM5->CCR1 = ccr;
    /* 若需立即生效，可取消注释触发更新事件 */
    // TIM5->EGR |= TIM_EGR_UG;
}

/**
 * @brief  设置 PWM 周期（自动重装载寄存器）
 * @param  arr: 自动重装载寄存器值（周期 - 1）
 * @retval 无
 */
void TIM5_PWM_SetPeriod(uint16_t arr)
{
    TIM5->ARR = arr;
    /* 若已使能 ARR 预装载，则需要触发更新事件 */
    TIM5->EGR |= TIM_EGR_UG;
}

/**
 * @brief  配置并初始化 TIM5 通道1 PWM 输出（PA0 = TIM5_CH1）
 * @param  arr: 自动重装载寄存器值（周期 - 1）
 * @param  psc: 预分频器值
 * @retval 无
 */
void TIM5_PWM_Init(uint16_t psc, uint16_t arr, uint16_t ccr)
{
    /* 1. 使能时钟：TIM5（APB1）, GPIOA（APB2） */
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    /* 2. 配置 PA0 为复用推挽输出 */
    GPIOA->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0);
    GPIOA->CRL |=  (GPIO_CRL_MODE0_0   /* 输出模式，10 MHz */
                  | GPIO_CRL_CNF0_1);  /* 复用推挽 */

    /* 3. 配置 TIM5 基础计数 */
    TIM5->PSC  = psc;    /* 预分频 */
    TIM5->ARR  = arr;    /* 自动重装载 */
    /* 使能 ARR 预装载 */
    TIM5->CR1 |= TIM_CR1_ARPE;

    /* 4. 配置通道1 为 PWM1 模式 */
    TIM5->CCMR1 &= ~TIM_CCMR1_OC1M;
    TIM5->CCMR1 |=  (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1); /* PWM 模式1 */
    /* 使能 CCR1 预装载 */
    TIM5->CCMR1 |= TIM_CCMR1_OC1PE;
    /* 默认占空比 0 */
    TIM5->CCR1 = ccr;

    /* 5. 设置极性（高电平有效）并暂时禁用输出 */
    TIM5->CCER &= ~TIM_CCER_CC1P;
    TIM5->CCER &= ~TIM_CCER_CC1E;
}
```

## 2.2、TIM5PWMOutput_reg.h

```c
/**
 * @file    TIM5PWMOutput_reg.h
 * @brief   基于寄存器的 TIM5 驱动接口与寄存器定义
 *
 * 本文件提供 STM32F1 系列 MCU 的 TIM5 基础定时器底层寄存器访问定义、
 * 配置参数以及初始化和控制函数原型。通过直接操作寄存器，实现对 TIM5
 * 外设的高效、精细化控制。
 *
 * @note
 * - 该驱动不依赖 HAL/LL 库，完全通过寄存器位操作完成时钟使能、
 *   预分频、计数器模式等配置。
 *
 * @version 1.0.0
 * @date    2025-05-13
 * @author  Wallace.zhang
 *
 * @copyright
 * (C) 2025 Wallace.zhang。保留所有权利。
 *
 * @license SPDX-License-Identifier: MIT
 */
#ifndef __TIM5PWMOUTPUT_REG_H
#define __TIM5PWMOUTPUT_REG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void TIM5_PWM_Start(void);
void TIM5_PWM_Stop(void);
void TIM5_PWM_SetDuty(uint16_t ccr);
void TIM5_PWM_SetPeriod(uint16_t arr);
void TIM5_PWM_Init(uint16_t psc, uint16_t arr, uint16_t ccr);


#ifdef __cplusplus
}
#endif

#endif /* __TIM5PWMOUTPUT_REG_H */



```

## 2.3、main.c
![[Pasted image 20250513164550.png | 1000]]
如上所示，在main()函数里依次调用两个函数即可。

## 2.4、编译、调试
![[Pasted image 20250513164650.png | 1000]]
将代码烧录进去，效果跟LL库一样。

# 三、梳理寄存器
---
## 3.1、TIMx_CCMR1捕获/比较模式寄存器1
![[Pasted image 20250513170056.png]]
![[Pasted image 20250513170236.png]]

如上所示，设置PWM模式1将OC1M设置110。
```c
/* 4. 配置通道1 为 PWM1 模式 */
TIM5->CCMR1 &= ~TIM_CCMR1_OC1M; // 清0
TIM5->CCMR1 |=  (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1); /* PWM 模式1 */
```

另外，将OC1PE设置1，使能CCR1的预装载。
```c
/* 使能 CCR1 预装载 */
TIM5->CCMR1 |= TIM_CCMR1_OC1PE;
```

## 3.2、TIMx_CCER捕获/比较使能寄存器
![[Pasted image 20250513171115.png]]
```c
/* 5. 设置极性（高电平有效）并暂时禁用输出 */
TIM5->CCER &= ~TIM_CCER_CC1P; // 高电平有效
TIM5->CCER &= ~TIM_CCER_CC1E; // 禁用输出
```

## 3.3、TIMx_EGR事件产生寄存器
![[Pasted image 20250513171405.png]]
```c
/* 6. 触发更新事件，立即加载预装载寄存器 */
TIM5->EGR |= TIM_EGR_UG;
```

# 四、细节补充
---
## 4.1、PWM波形的启动与关闭
![[Pasted image 20250513172255.png | 1000]]
如上所示，从寄存器代码看来，启动与关闭PWM输出有两个开关。分别是CHx的开关（CC1E）与定时器的开关（CEN）。
