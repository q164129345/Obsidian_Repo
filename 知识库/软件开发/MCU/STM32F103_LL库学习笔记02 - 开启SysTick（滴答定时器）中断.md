# 导言
---
[[STM32F103_LL库学习笔记01 - 梳理CubeMX生成的LL库最小的裸机系统框架]]上一章节对CubeMX生成的最小系统框架进行梳理，在此工程的基础上，梳理SysTick（滴答定时器）中断是怎样开启的。

效果如下：
![[LL02_Tick_Interrupt.gif | 1100]]
在代码stm32f1xx_it.c里的函数`SysTick_Handler()`在持续被调用，将全局变量gTickCount持续累加。

项目地址：https://github.com/q164129345/MCU_Develop/tree/main/stm32f103_ll_library02_SysTick_Interrupt

# 一、寄存器SysTick->CTRL
![[Pasted image 20250224205356.png | 800]]
如上所示，将SysTick->CTRL的段TICKINT置1就可以开启SysTick中断。
![[Pasted image 20250224210223.png | 800]]
![[Pasted image 20250224210351.png | 800]]
如上所示，


# 二、代码
---
## 2.1、main.c
![[Pasted image 20250224205732.png | 800]]
如上所示，通过或运算，将寄存器CTRL的段TICKINT置1，启动SysTick中断。
## 2.2、stm32f1xx_it.c
![[Pasted image 20250224210450.png | 800]]
在SysTick中断回调函数`SysTick_Handler()`里将全局变量gTickCount累加。

# 三、Debug调试
---
![[Pasted image 20250224210223.png | 800]]
![[Pasted image 20250224210351.png | 800]]
如上所示，从debug模式看到寄存器SysTick->CTLR = 0x00000007相当于0111(二进制)，相当于：
- CLKSOURCE = 1，SysTick的时钟源来自AHB
- TICKINT = 1，打开中断
- ENABLE = 1，启动倒数

# 四、细节补充
---
### 4.1、为什么SysTick中断时会自动调用SysTick_Handler()
假如我想将SysTick的中断回调关联到新的函数SysTick_Interrupt()而不是原来的SysTick_Handler()，该怎样做？
![[Pasted image 20250224213107.png | 800]]
![[Pasted image 20250224213148.png | 800]]
![[Pasted image 20250224213218.png | 800]]
![[Pasted image 20250224213310.png | 800]]
如上四张图所示，改完后直接编译代码。
![[Pasted image 20250224213400.png | 800]]
如上，编译成功。
![[Pasted image 20250224213450.png | 800]]
如上所示，新的中断回调函数`SysTick_Interrupt()`被正常调用。
到此，只是说明怎样改变SysTick中断时，调用其他函数而已。但是，为什么能跟.s启动文件的中断向量表里的SysTick_Interrupt关联起来呢？
![[Pasted image 20250224214032.png | 800]]
如上所示，**中断向量表里的这个位置，就是对应SysTick中断回调的。** 在 Cortex-M 的启动文件中，每一个中断/异常在向量表里都有一个固定“槽位”，而那个“DCD SysTick_Interrupt ; SysTick Handler”这一行就代表 SysTick 中断的入口地址。如果你将向量表中的 SysTick 条目写成 DCD SysTick_Interrupt，那么当 SysTick 中断发生时，CPU 就会跳转到名为 SysTick_Interrupt() 的函数入口执行。

换句话说，**向量表中的那一行就是告诉 CPU：SysTick 中断对应的中断服务函数是哪个。**




