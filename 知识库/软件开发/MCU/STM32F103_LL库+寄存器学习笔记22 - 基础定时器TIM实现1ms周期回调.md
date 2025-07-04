# 导言
---
![[Pasted image 20250512192708.png]]
如上所示，STM32F103有两个基本定时器TIM6与TIM7，所谓「基本定时器」，即功能最简单的定时器。

项目地址：
github:
- LL库: https://github.com/q164129345/MCU_Develop/tree/main/stm32f103_ll_library22_Basic_Timer
- 寄存器方式: https://github.com/q164129345/MCU_Develop/tree/main/stm32f103_reg_library22_Basic_Timer

gitee(国内):
- LL库: https://gitee.com/wallace89/MCU_Develop/tree/main/stm32f103_ll_library22_Basic_Timer
- 寄存器方式: https://gitee.com/wallace89/MCU_Develop/tree/main/stm32f103_reg_library22_Basic_Timer

# 一、LL库
---
## 1.1、CubeMX
![[Pasted image 20250512194649.png | 1000]]
如上所示，STM32F103 的定时器时钟频率均为 72 MHz。

![[Pasted image 20250512195212.png | 1000]]
![[Pasted image 20250512194542.png | 1000]]
![[Pasted image 20250512194608.png | 1000]]

## 1.2、tim.c
![[Pasted image 20250512201816.png | 1000]]
CubeMX 根据上述配置，自动生成了相应初始化代码。

## 1.3、main.c
![[Pasted image 20250512202008.png | 1000]]

## 1.4、stm32f1xx_it.c
![[Pasted image 20250512204855.png | 1000]]

## 1.5、编译、调试
![[Pasted image 20250512202240.png | 1000]]
![[22_Basic_Timer.gif | 1000]]
如上所示，串口疯狂打印“Hello,World\n"。

# 二、寄存器方式
---
## 2.1、myTIM6Drive.c
![[Pasted image 20250512204527.png | 1000]]
![[Pasted image 20250512204958.png | 1000]]
如上所示，基于寄存器的实现代码简洁明了。

## 2.2、编译、调试
![[Pasted image 20250512205126.png | 1000]]
![[22_Basic_Timer_Reg.gif | 1000]]
由此可见，两种实现方式在功能和效果上完全一致。

# 三、梳理寄存器
---
## 3.1、RCC_APB1ENR时钟使能寄存器
![[Pasted image 20250513101715.png]]
```c
RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; // 使能 TIM6 时钟
```

## 3.2、TIMx_PSC预分配寄存器
![[Pasted image 20250513102126.png]]
```c
TIM6->PSC = 71;      /* PSC 寄存器：计数时钟 = PCLK1/(PSC+1) */
```

## 3.3、TIMx_ARR自动重载寄存器
![[Pasted image 20250513102246.png]]
```c
TIM6->ARR = 999;     /* ARR 寄存器：计数到 ARR 后产生更新事件 */
```

## 3.4、TIMx_CR1控制寄存器1
![[Pasted image 20250513102447.png]]
```c
TIM6->CR1 |= TIM_CR1_ARPE; /* 3. 使能 ARR 预装载 */

TIM6->CR1 |= TIM_CR1_CEN;  /* 4. 启动定时器 */
```

## 3.5、TIMx_SR状态寄存器
![[Pasted image 20250513103105.png]]
```c
TIM6->SR &= ~TIM_SR_UIF; /* 1. 清除更新中断标志 */
```
在中断回调函数中，遇到 UIF 更新中断时，需要及时将其标志位清零。
`TIM_SR_UIF` 定义为 0x0001（即 bit-0）。所以，`~TIM_SR_UIF` 等于 `0xFFFE`。把 UIF 位对应的位置为 0，其它位为 1。最终，`SR &= 0xFFFE` 就是“给 UIF 写 0，给其它位写 1”——符合“写 0 清”语义。

## 3.6、TIMx_DIER中断使能寄存器
![[Pasted image 20250513103728.png]]
如上所示，将 DIER 寄存器的 UIE 位置 1，可使能 TIMx 的更新中断（计数溢出）。
```c
TIM6->DIER |= TIM_DIER_UIE; // 使能更新中断
```

# 四、细节补充
---

















