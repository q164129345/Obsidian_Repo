# 导言
---
《[[STM32 - 在机器人领域，LL库相比HAL优势明显]]》在机器人、自动化设备领域使用MCU开发项目，必须用LL库。本系列笔记记录使用LL库的开发过程，并通过LL库梳理寄存器的使用。只要掌握寄存器（底层原），才能真正掌握MCU开发。
《STM32F1中文参考手册》、《Cortex-M3技术参考手册》一定要多看，读烂它！！
《STM32F1中文参考手册》、《Cortex-M3技术参考手册》一定要多看，读烂它！！
《STM32F1中文参考手册》、《Cortex-M3技术参考手册》一定要多看，读烂它！！

# 一、CubeMX
---
## 1.1、RCC
![[20250214-120135.jpg | 1100]]
使用CubeMX创建项目，首先要把RCC时钟配置好。如上所示，使能HSE时钟连接外部晶振。接着，单片机的PD0与PD1接口被复用成RCC_OSC_IN与RCC_OSC_OUT，用于连接外部高速晶振。下图是某块FOC控制板的原理图，从图上看到，OSC_IN与OSC_OUT连接了一个外部8M的晶振。
![[Pasted image 20250214114334.png | 800]]
## 1.2、Clock Configuration
![[Pasted image 20250213205024.png | 900]]
![[Pasted image 20250214114454.png | 1000]]
如上两图所示：
CubeMX的Clock Configuration其实就是《STM32F1中文参考手册》6.2时钟章节的时钟数。单片机的引脚OSC_OUT（PD0）与OSC_IN（PD1）连接外部晶振（一般8M），通过PLLMUL模块后将频率加倍（倍频）至72M（最大频率）。*注：STM32F407的系统时钟最大168M。*

## 1.3、SYS
![[Pasted image 20250214170722.png | 1100]]
如上所示：
- Debug(调试端口)选择Serial Wire时，相当于只开SWD，关闭JTAG。这样，最节省GPIO，只用了PA13与PA14。
- Timebase Source选择SysTick，不占用其他TIM外设。这样是节省资源的方案。

## 1.4、GPIO
![[Pasted image 20250218181941.png | 1100]]
![[Pasted image 20250218182045.png]]
从原理图看到，当GPIO拉低时，LED灯亮起来。灌电流 = 3.3V / 2000欧 = 1.65ma。

## 1.5、Project Manager
![[Pasted image 20250218180911.png | 1100]]
![[Pasted image 20250218180958.png | 1100]]
![[Pasted image 20250218181036.png | 1100]]

# 三、代码
---
让CubeMX生成项目代码，生成的主文件包括main.c和stm32f1xx.it.c。

## 3.1、main()
![[Pasted image 20250218195203.png | 1100]]
![[Pasted image 20250218195257.png | 1100]]
接下来，开始将函数main()里的每一个函数开始梳理。

### 3.1.1、LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO)
```c
__STATIC_INLINE void LL_APB2_GRP1_EnableClock(uint32_t Periphs)
{
  __IO uint32_t tmpreg;
  SET_BIT(RCC->APB2ENR, Periphs); // 让RCC的寄存器APB2ENR的某个bit置1
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = READ_BIT(RCC->APB2ENR, Periphs); // 读取寄存器APB2ENR的某个bit的状态（是0还是1）
  (void)tmpreg; // 避免编译器警告与防止优化
}
```

**tmpreg = READ_BIT(RCC->APB2ENR, Periphs)的作用是什么？** 函数确保了硬件寄存器的操作确实得到了执行，并具备了一定的延迟（尽管极小），等待MCU处理时钟使能。从代码`tmpreg = READ_BIT(RCC->APB2ENR, Periphs)`看来，变量tmpreg能获取寄存器APB2ENR的某个bit的状态，但是函数LL_APB2_GRP1_EnableClock()返回的类型是void，即并不需要返回任何变量。那么，代码`tmpreg = READ_BIT(RCC->APB2ENR, Periphs)`的目的是为什么？为了延迟一下（尽管极小），等待MCU处理时钟使能。

**(void)tmpreg的作用是什么？** 避免编译器警告与防止优化。编译器不会因为变量未使用而报警。让编译器知道这个变量确实需要被访问，以防止某些编译器优化可能导致的读操作被省略。

**LL_APB2_GRP1_PERIPH_AFIO相当于1UL**
![[Pasted image 20250218213253.png | 1100]]
**所以，LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO)的作用是将RCC的寄存器APB2ENR的bit0置1，使能AFIO时钟。**

从下图看到，进入Debug模式，查看LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO)的运行过程，看到RCC的APB2ENR寄存器的bit0位AFIOEN被置1。
![[Pasted image 20250218214653.png | 1000]]
![[Pasted image 20250218214822.png | 1000]]



