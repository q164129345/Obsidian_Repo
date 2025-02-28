# 导言
---
GPIO设置输入模式后，一般会用轮询的方式去查看GPIO的电平状态。比如，最常用的案例是用于检测按钮的当前状态（是按下还是没按下）。中断的使用一般用于计算脉冲的频率与计算脉冲的数量。

项目地址：https://github.com/q164129345/MCU_Develop/tree/main/stm32f103_ll_library05_gpio_input_interrupt

# 一、代码
---
## 1.1、main.c
![[Pasted image 20250227154032.png | 800]]
如上所示，函数`EXTI_Configure()`完成PB4的上升沿触发中断的设置。
![[Pasted image 20250227154342.png | 800]]
如上所示，一定要在GPIO设置完输入模式后，才能设置中断。

## 1.2、stm32f1xx_it.c
![[Pasted image 20250227160212.png | 800]]
如上所示，编写EXTI4中断回调函数EXTI4_IRQHandler()的内容，在函数EXTI4_IRQHandler()里一定要清除中断标志，否则下一次中断将不会被执行。
为什么EXTI4的中断函数的名字是EXTI4_IRQHandler()? 在启动文件startup_stm32f103xb.s的中断向量表里有定义，当发现EXTI4中断时，调用函数EXTI4_IRQHandler()。如下所示：
![[Pasted image 20250227160809.png | 800]]

回到EXTI4中断回调函数EXTI4_IRQHandler()，既然EXTI4捕获到中断时，会调用函数EXTI4_IRQHandler()。但是，为什么EXTI4_IRQHandler()里需要用if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_4) != RESET)再一次确认是EXTI4的中断标志位？
我认为主要有两个原因：
1. 规范中断回调函数的一致性（都要再一次通过中断标志位确认）
	- 在 STM32 中，并非所有 EXTI 线路都有独立的中断处理函数。例如，EXTI5 到 EXTI9 共享一个处理函数 EXTI9_5_IRQHandler()，在这种情况下，必须通过检查标志位来判断具体是哪条线路触发了中断。所以，不管是EXTI1（有独立的中断回调函数EXTI41_IRQHandler）还是EXTI9都要在中断回调函数里再一次检查中断标志位。
2. 确保中断源的准确性
	- 尽管 EXTI4_IRQHandler() 通常只由 EXTI4 触发，但在某些异常情况下（例如软件配置错误、中断控制器异常等），其他因素可能导致误触发。检查 LL_EXTI_LINE_4 的标志位可以确认中断确实是由 EXTI4 引发的，从而避免执行错误的处理逻辑。这种额外的验证提高了代码的健壮性。
![[Pasted image 20250227161550.png | 800]]

# 二、寄存器的梳理
---
## 2.1、中断向量表
《STM32F1参考手册》的章节9.1.2-中断与异常向量看到，EXTI中断一共有如下：
![[Pasted image 20250227163448.png | 800]]
![[Pasted image 20250227163523.png | 800]]
如上所示，只有EXTI1～4有单独的中断地址，EXTI5～EXTI9共享一个中断地址，EXTI10～EXTI15共享一个中断地址。所以，弄明白中断回调函数`EXTI4_IRQHandler()`里为什么再一次使用代码`if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_4) != RESET)`去判断中断标志位了。从中断向量表里看到，EXTI5～EXTI9确认共享一个中断地址（函数指针）。

## 2.2、为什么PB4的中断对应EXTI4？
![[Pasted image 20250227164254.png | 800]]
如上所示，根据《STM32F1参考手册》的章节9.2.5看到GPIO跟EXTI中断线的对应关系，PB4对应EXTI4。

## 2.3、外部中断配置寄存器x (AFIO_EXTICRx)
![[Pasted image 20250227174505.png]]
![[Pasted image 20250227175147.png | 800]]
如上所示，《STM32F1参考手册》的章节8.4.4，寄存器AFIO_EXTICR2的段EXTI4 = 0001时，相当于将PB4映射到EXTI4。
```c
MODIFY_REG(AFIO->EXTICR[1], 0xF << 0UL, 0x01 << 0UL); // 配置EXTI4线路映射到PB4引脚
```

## 2.4、上升沿触发选择寄存器(EXTI_RTSR)
![[Pasted image 20250227201407.png]]
寄存器EXTI_PTSR的作用设置上升沿是否触发EXTI线中断。比如，位TR4 = 1相当于EXTI4启动上升沿触发中断。
```c
EXTI->RTSR |= 0x01UL << 4UL;  // EXTI4开启上升沿触发中断
EXTI->RTSR |= 0x01UL << 10UL; // EXTI10开启上升沿触发中断

EXTI->RTSR &= ~(0x01UL << 4UL);  // EXTI4关闭上升沿触发中断
EXTI->RTSR &= ~(0x01UL << 10UL); // EXTI10关闭上升沿触发中断
```


## 2.5、下降沿触发选择寄存器(EXTI_FTSR)
![[Pasted image 20250227201627.png]]
寄存器EXTI_FTSR的作用设置下降沿是否触发EXTI线中断。比如，为TR4 = 1相当于EXTI4启动下降沿触发中断。

```c
EXTI->FTSR |= 0x01UL << 4UL; // 开启EXTI4下降沿中断
EXTI->FTSR &= ~(0x01UL << 4UL); // 关闭EXTI4下降沿中断
```

## 2.6、挂起寄存器(EXTI_PR)
![[Pasted image 20250227201806.png]]
当寄存器EXTI_PR的位PR4置1时，代表触发EXTI4中断。 往里写‘1’可以清除它。
```c
if (EXTI->PR & (0x01UL << 4UL)) { // 判断是不是EXTI4中断
	EXTI->PR |= 0x01UL << 4UL; // 清除EXTI4的中断标志
}

if (EXTI->PR & (0x01UL << 10UL)) { 判断是不是EXTI10中断
	EXTI->PR |= 0x01 << 10UL; // 清除EXTI10的中断标志
}
```

# 三、寄存是方式的实现
---
## 3.1、main.c
![[Pasted image 20250227203017.png | 800]]
如上所示，使用寄存器方式真的简洁。

## 3.2、stm32f1xx_it.c
![[Pasted image 20250227203129.png | 800]]
如上所示，通过寄存器EXTI_PR的bit4判断是否是EXTI4中断。然后往bit4写入‘1’就能清除中断标志。
![[Pasted image 20250227203519.png | 1100]]
如上所示，在debug模式看到，当PB4从低电平->高电平时，进入中断回调函数EXTI4_IRQHandler()，且寄存器EXTI_PR的bit4被置1。

# 四、细节补充
---
## 4.1、EXTI4支持同时检测PB4与PA4吗？？
![[Pasted image 20250227205331.png]]
根据《STM32F1参考手册》的章节9.2.5，根据PA0～PG0映射到EXTI0上，所以PA4～PG4是映射到EXTI4上。有一个疑问，STM32F1支持PA4、PB4一起映射到EXTI4上吗？？抱着这个疑问，我尝试用CubeMX试试，看看ST官方工具允许不允许这样做。
![[LL05_EXTI.gif | 1100]]
如上图所示，CubeMX不支持PA4与PB4一起映射到EXTI4。PA4映射到EXTI4后，如果将PAB4映射到EXTI4的话，PA4就会自动失效。**所以，每一个EXTIx只能映射某一个PAx。**

## 4.2、STM32F103一共支持多少路EXTI中断？
![[Pasted image 20250228092407.png | 800]]
如上所示，参考《STM32F1参考手册》的章节9.2.5，在STM32F103引脚资源足够的情况下，**最多支持16个GPIO口映射到EXTI外部中断（EXTI0～EXTI15）。** 另外EXTI16～EXTI19不能映射到普通GPIO，只能用于处理特定内部事件或外设的中断/事件，比如EXTI18的USB唤醒事件。











