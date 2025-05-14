# 导言
---
《[[STM32外设 - HAL库 - TIM，产生互补PWM波，中心对齐模式1 + PWM模式2（FOC算法专用）]]》之前的笔记使用HAL库实现三相中心对齐且互补的PWM波形，本章节将采用LL库与寄存器方式实现。详细的解释请看之前的HAL库笔记。

![[Pasted image 20250513203010.png | 1000]]

项目地址：
github:
- LL库: https://github.com/q164129345/MCU_Develop/tree/main/stm32f103_ll_library24_TIM_Center_PWM_FOC
- 寄存器方式: https://github.com/q164129345/MCU_Develop/tree/main/stm32f103_reg_library24_TIM_Center_PWM_FOC

gitee(国内):
- LL库: https://gitee.com/wallace89/MCU_Develop/tree/main/stm32f103_ll_library24_TIM_Center_PWM_FOC
- 寄存器方式: https://gitee.com/wallace89/MCU_Develop/tree/main/stm32f103_reg_library24_TIM_Center_PWM_FOC

# 一、LL库
---
## 1.1、CubeMX
![[Pasted image 20250513201619.png | 1000]]
如上所示，为什么这样设置，请看之前的HAL库笔记《[[STM32外设 - HAL库 - TIM，产生互补PWM波，中心对齐模式1 + PWM模式2（FOC算法专用）]]》。

![[Pasted image 20250513201748.png | 1000]]
如上所示，3个通道的设置一样，这里只演示通道1的设置。

![[Pasted image 20250513201902.png | 1000]]

## 1.2、tim.c
![[Pasted image 20250513202108.png | 1000]]

## 1.3、main.c
![[Pasted image 20250513202426.png | 1000]]
如上所示，6个通道分别使能之后，还要打开一个总开关。接着，启动定时器，PWM输出。

## 1.4、编译、调试
![[Pasted image 20250513202825.png | 1000]]
编译通过。
![[Pasted image 20250513203010.png | 1000]]
如上所示，用示波器测量通道1的波形。PWM频率10KHz，占空比非常接近50%。

# 二、寄存器方式
---
## 2.1、TIM1_FOC_PWM_reg.c
```c
#include "TIM1FOCPWM/TIM1_FOC_PWM_reg.h"

/**
  * @brief  将 TIM1 完整 remap 到 PE7/PE8~PE15，并配置 PE8~PE13 为
  *         TIM1_CH1N/CH1, CH2N/CH2, CH3N/CH3
  */
static void TIM1_GPIO_Init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOE);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
    /**TIM1 GPIO Configuration
        PE8     ------> TIM1_CH1N
        PE9     ------> TIM1_CH1
        PE10     ------> TIM1_CH2N
        PE11     ------> TIM1_CH2
        PE12     ------> TIM1_CH3N
        PE13     ------> TIM1_CH3
        */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_8|LL_GPIO_PIN_9|LL_GPIO_PIN_10|LL_GPIO_PIN_11
                              |LL_GPIO_PIN_12|LL_GPIO_PIN_13;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    LL_GPIO_AF_EnableRemap_TIM1();
}

/**
  * @brief  设置 TIM1 的死区时间（Dead-Time Generator）
  * @param  dtg 死区生成器值（0～255），对应 BDTR.DTG[7:0]
  *            - 当 dtg < 128 时，DeadTime ≈ dtg × T_DTS
  *            - 大于等于 128 则进入模式2/3，如需超长死区才用
  * @note   使用前请确保 TIM1_BDTR.MOE = 1，且定时器已使能时钟
  */
static void TIM1_SetDeadTime(uint8_t dtg)
{
    /* 1. 清除旧的 DTG 字段 */
    TIM1->BDTR &= ~TIM_BDTR_DTG;
    /* 2. 写入新的死区值 */
    TIM1->BDTR |= (uint32_t)dtg;
}

/**
  * @brief  触发定时器的更新事件（等价于向 EGR 寄存器写入 UG 位）
  * @note   EGR 寄存器写 1 触发事件，写 0 无效果，因此直接赋值或 SET_BIT 都可
  */
__STATIC_INLINE void TIM1_GenerateEvent_UPDATE(void)
{
    /* 方式一：直接赋值，触发 UG */
    TIM1->EGR = TIM_EGR_UG;
    
    /*  
    // 或者方式二：使用 CMSIS 宏置位
    SET_BIT(TIMx->EGR, TIM_EGR_UG);
    */
}

/**
  * @brief      设置 TIM1 通道 1/2/3 的比较值，并触发更新事件使新值立即生效
  * @param[in]  CH1CompareValue  通道 1 的比较值（写入 CCR1）
  * @param[in]  CH2CompareValue  通道 2 的比较值（写入 CCR2）
  * @param[in]  CH3CompareValue  通道 3 的比较值（写入 CCR3）
  * @note       写入 CCRx 后，需要调用 TIM1_GenerateEvent_UPDATE()  
  *             向 EGR 寄存器写入 UG 位，才能将预装载的比较值刷新到实际计数比较单元
  */
void TIM1_Set_Compare_Value(uint32_t CH1CompareValue, uint32_t CH2CompareValue, uint32_t CH3CompareValue)
{
    TIM1->CCR1 = CH1CompareValue;
    TIM1->CCR2 = CH2CompareValue;
    TIM1->CCR3 = CH3CompareValue;
    TIM1_GenerateEvent_UPDATE(); // 及时更新比较值
}

/**
  * @brief  启动 TIM1，开始输出 PWM
  * @note   会重新触发一次更新事件，确保预装载值写入立即生效；  
  *         并重新打开主输出，使 PWM 端口有效。
  */
void TIM1_PWM_Start(void)
{
    /* 1. 触发更新事件，刷新 ARR/CCR */
    TIM1->EGR = TIM_EGR_UG;
    /* 2. 使能主输出 */
    TIM1->BDTR |= TIM_BDTR_MOE;
    /* 3. 使能定时器计数 */
    TIM1->CR1 |= TIM_CR1_CEN;
}

/**
  * @brief  停止 TIM1 PWM 输出
  * @note   关闭计数并禁止主输出，确保 PWM 输出端口失效
  */
void TIM1_PWM_Stop(void)
{
    /* 1. 禁止定时器计数 */
    TIM1->CR1 &= ~TIM_CR1_CEN;
    /* 2. 禁用主输出 */
    TIM1->BDTR &= ~TIM_BDTR_MOE;
}

/**
  * @brief  基于寄存器初始化 TIM1 产生 3 路 PWM 互补波形
  * @param  period   定时器重装载值 (ARR)，决定 PWM 周期
  * @param  duty1    通道 1 的比较值 (CCR1)，决定占空比
  * @param  duty2    通道 2 的比较值 (CCR2)
  * @param  duty3    通道 3 的比较值 (CCR3)
  * @note   产生 PWM 模式 2，带互补输出，自动重载、比较寄存器均作预装载，高电平有效
  */
void TIM1_PWM_Init(uint16_t period,
                   uint16_t duty1,
                   uint16_t duty2,
                   uint16_t duty3)
{
    /* 1. 使能时钟：TIM1、GPIOE、AFIO */
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    
    /* 2. 初始化GPIO */
    TIM1_GPIO_Init();
    
    /* 3. 时间基准配置：无分频，上升计数 */
    TIM1->PSC = 0;               /* PSC = 0，即分频 1 */
    TIM1->ARR = period;          /* 自动重载值 */
    TIM1->CR1 |= TIM_CR1_ARPE    /* ARR 预装载 */
               | TIM_CR1_CMS_0;  /* CMS = 01：中心对齐模式1 */

    /* 4. PWM 模式2 + 预装载 (OCxM = 111, OCxPE = 1) */
    TIM1->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC1PE
                   | TIM_CCMR1_OC2M | TIM_CCMR1_OC2PE);
    TIM1->CCMR1 |=  TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE
                 | TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE;

    TIM1->CCMR2 &= ~(TIM_CCMR2_OC3M | TIM_CCMR2_OC3PE);
    TIM1->CCMR2 |=  TIM_CCMR2_OC3M_0 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE;

    /* 5. 写入比较值 */
    TIM1->CCR1 = duty1;
    TIM1->CCR2 = duty2;
    TIM1->CCR3 = duty3;
    
    /* 6. 清除所有极性位，高电平有效 */
    TIM1->CCER &= ~(
         TIM_CCER_CC1P | TIM_CCER_CC1NP
       | TIM_CCER_CC2P | TIM_CCER_CC2NP
       | TIM_CCER_CC3P | TIM_CCER_CC3NP
    );
    
    /* 7. 使能通道输出及互补输出 */
    TIM1->CCER |= TIM_CCER_CC1E  | TIM_CCER_CC1NE
                | TIM_CCER_CC2E  | TIM_CCER_CC2NE
                | TIM_CCER_CC3E  | TIM_CCER_CC3NE;
    
    /* 8. 设置死区时间（100 个定时器时钟周期，大约 1.39μs） */
    TIM1_SetDeadTime(100);
    
    /* 9. 主输出使能（BDTR 寄存器的 MOE 位）*/
    TIM1->BDTR |= TIM_BDTR_MOE;

    /* 10. 触发一次更新事件，立即把 ARR/CCR/CCMR1 等预装载寄存器写入工作寄存器 */
    TIM1_GenerateEvent_UPDATE();
}

```

## 2.2、TIM1_FOC_PWM_reg.h
```c
/**
 * @file    TIM1_FOC_PWM_reg.h
 * @brief   基于寄存器的 TIM1 高级定时器驱动接口与寄存器定义，用于 FOC 控制
 *
 * 本文件提供 STM32F1 系列 MCU 的 TIM1 高级定时器底层寄存器访问定义、
 * 配置参数以及初始化和控制函数原型，支持中心对齐模式1与 PWM 模式2，
 * 并可设置死区时间以满足 FOC 算法对对称 PWM 波形的严格要求。
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
#ifndef __TIM1_FOC_PWM_REG_H
#define __TIM1_FOC_PWM_REG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/**
  * @brief  启动 TIM1，开始输出 PWM
  */
void TIM1_PWM_Start(void);

/**
  * @brief  停止 TIM1 PWM 输出
  */
void TIM1_PWM_Stop(void);

/**
  * @brief      设置 TIM1 通道 1/2/3 的比较值，并触发更新事件使新值立即生效
  */
void TIM1_Set_Compare_Value(uint32_t CH1CompareValue, uint32_t CH2CompareValue, uint32_t CH3CompareValue);

/**
  * @brief  基于寄存器初始化 TIM1 产生 3 路 PWM 互补波形
  */
void TIM1_PWM_Init(uint16_t period, uint16_t duty1, uint16_t duty2, uint16_t duty3);

#ifdef __cplusplus
}
#endif

#endif /* __TIM1_FOC_PWM_REG_H */

```

## 2.3、main.c
![[Pasted image 20250514135452.png | 1000]]

## 2.4、编译、调试
![[Pasted image 20250514135559.png | 1000]]
编译成功。
![[Pasted image 20250514140331.png | 1000]]
从示波器的波形看来，没问题。

# 三、梳理寄存器
---
## 3.1、TIMx_CR1控制寄存器1
![[Pasted image 20250514145500.png]]
```c
TIM1->CR1 |= TIM_CR1_ARPE    /* ARR 预装载 */
		   | TIM_CR1_CMS_0;  /* CMS = 01：中心对齐模式1 */
```
如上所示，通过TIMx_CR1设置计数器向上计数、中心对齐模式1、TIMx_ARR寄存器被装入缓冲器。

**为什么要将ARPE置1？**
1. 同步更新
   当 ARPE=1，所有对 ARR（或 CCRx） 的写操作都会先进入影子寄存器，只有在计数器产生“更新事件”（overflow/underflow，或写 EGR.UG）时，才一次性把新的周期和占空比值加载到实际寄存器中。这样所有参数在同一时刻生效，保证了 PWM 周期的原子性。

2. 与预装载比较寄存器配合
   我们通常也会打开各通道的 OCxPE 位，配合 ARPE，使 CCRx 同样获得预装载能力，更新时机与 ARR 保持一致，避免 DUTY 和 PERIOD 不匹配带来的输出畸变。

## 3.2、TIMx_CCMR1与TIMx_CCMR2捕获/比较寄存器1与2
![[Pasted image 20250514151348.png]]

如上所示，通过CCMR1与CCMR2设置CH1～CH3的工作模式（PWM模式2）与使能预装载功能。

## 3.3、TIMx_CCR1 ~ CCR3捕获/比较寄存器1～3
![[Pasted image 20250514151907.png]]
如上所示，设置CCR1～CCR3的值改变PWM占空比。

## 3.4、TIMx_CCER捕获/比较使能寄存器
![[Pasted image 20250514153505.png]]
如上所示，通过CCER寄存器将CH1～CH3设置有效电平是高电平（根据MOS选型来选择，大部分场景使用的MOS都是高电平导通）、开启互补输出、使能输出。

## 3.5、TIMx_BDTR刹车和死区寄存器
![[Pasted image 20250514160403.png]]
在本章节，通过TIMx_BDTR设置死区时间与控制主输出。

## 3.6、TIMx_EGR事件产生寄存器
![[Pasted image 20250514171942.png]]
如上所示，函数`TIM1_GenerateEvent_UPDATE()`的作用是将bit0置1（产生更新事件），其他bit清0。




















