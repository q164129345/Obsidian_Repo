# 导言
---
《[[STM32 - 在机器人领域，LL库相比HAL优势明显]]》在机器人、自动化设备领域使用MCU开发项目，必须用LL库。
本系列笔记记录使用LL库的开发过程，首先通过CubeMX生成LL库代码，梳理LL库源码。通过学习LL库源码，弄清楚寄存器的使用。最后，删除LL库代码，编写自己的寄存器驱动代码验证一遍。

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

## 1.4、Project Manager
![[Pasted image 20250218180911.png | 1100]]
![[Pasted image 20250218180958.png | 1100]]
![[Pasted image 20250218181036.png | 1100]]

# 三、代码
---
让CubeMX生成项目代码，生成的主文件包括main.c和stm32f1xx.it.c。

## 3.1、main()
![[Pasted image 20250218195203.png | 1100]]
![[Pasted image 20250219144159.png | 1100]]
如上两图所示，main()函数的作用如下：
1. 初始化：
	1. 启用AFIO与PWR外设的时钟。
	2. 配置NVIC优先级分组和SysTick中断优先级。
	3. 禁用JTAG，启用SWD调试接口。
	4. 调用SystemClock_Config()配置系统时钟。
	5. 启动GPIOD组与GPIOA组的外设时钟（因为HSE与SWD接口使用了这两组GPIO）。
2. 主循环：
	1. 每隔1S，将变量cnt加1。

## 3.2、RCC时钟相关代码
**当需要使用 MCU 的某些外设功能时，必须先打开对应的时钟。** 请记住，让外设正常工作的前提是启动对应的时钟。例如，UART、SPI、I2C等外设都需要其对应的时钟启动。其中RCC的寄存器APB1ENR与寄存器APB2ENR几乎管理着所有外设的时钟，
本章节需要启动以下几个外设的时钟：
- AFIO
- PWR
- GPIOD
- GPIOA
为此，需要通过RCC的寄存器APB1ENR与寄存器APB2ENR去启用它们的时钟。

### 3.2.1、启用AFIO辅助功能IO时钟
`LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO)`的作用是启用AFIOEN辅助功能IO时钟时钟。
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

**总的来说，LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO)的作用是将RCC的寄存器APB2ENR的bit0置1，使能AFIO时钟。** 当需要使用 MCU 的某些外设功能时，必须先打开对应的时钟。AFIO 模块在 STM32F1 系列中用于管理 GPIO 的复用功能。
#### 3.2.1.1、LL_APB2_GRP1_PERIPH_AFIO相当于1UL
![[Pasted image 20250218213253.png | 1100]]
如上所示，LL_APB2_GRP1_PERIPH_AFIO = RCC_APB2ENR_AFIOEN = RCC_APB2ENR_AFIOEN_Msk = 0x01UL << 0 = 0x01UL。

![[Pasted image 20250218214653.png | 800]]
![[Pasted image 20250218214822.png |800]]
从上两张图看到，在Keil的Debug模式观察LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO)的运行过程，看到RCC的寄存器APB2ENR的bit0位AFIOEN被置1。
#### 3.2.1.2、梳理APB2 外设时钟使能寄存器(RCC_APB2ENR)
![[Pasted image 20250219111820.png | 1100]]
如上所示，在STM32F1参考手册的6.3.7章节找到APB2 外设时钟使能寄存器(RCC_APB2ENR)的详细信息，看到RCC的寄存器APB2ENR的bit0位是AFIO_EN。

![[Pasted image 20250219112041.png | 1100]]
**因为需要用到GPIO的复用功能，GPIO的复用功能由AFIO管理，所以要使用GPIO的复用功能必须先把对应的时钟（AFIO时钟）打开。** 如上所示，置1代表开启时钟。另外，RCC的寄存器APB1ENR与寄存器APB2ENR几乎管理着所有外设时钟的开启与关闭。另外值得注意的是，关闭不使用的外设的时钟可以有效降低MCU的功耗，这在低功耗场合非常关键。
- APB1ENR（APB1 Peripheral Clock enable register）:
	管理 APB1 总线上的外设时钟，如 UART2, UART3, I2C1, I2C2, SPI2, CAN, USB, DAC 等。
- APB2ENR（APB2 Peripheral Clock enable register）:
    管理 APB2 总线上的外设时钟，如 AFIO（如我们之前讨论的）、GPIOA 到 GPIOG, ADC1, ADC2, TIM1, SPI1, USART1 等。

#### 3.2.1.3、复位和时钟控制(RCC)的存储器映像（地址）
![[Pasted image 20250219115234.png | 1100]]
从STM32F1参考手册2.3章节看到，RCC在存储器的起始地址是0x40021000，这个起始地址非常重要。后续，通过这个起始地址直接控制RCC。
![[Pasted image 20250219121116.png | 1100]]
**为什么程序能通过这个内存地址来控制单片机的RCC呢？** RCC硬件的寄存器被映射到内存地址空间（0x40021000~0x400213FF)中，我们外部程序只能通过修改内存地址去间接控制RCC寄存器。当RCC硬件检测到映射的内存被修改，马上根据修改的内容去调整硬件状态。比如当发现APB2ENR的bit0被置1时，马上启动AFIO时钟。总的来说，想控制哪个外设时，应该先去找对应的内存映射地址（Memory-Mapped），即STM32F1参考手册2.3章节的内容。

#### 3.2.1.4、复位和时钟控制(RCC)的结构
![[Pasted image 20250219144714.png | 1100]]
如上所示，从STM32F1中文参考手册6.3.11章节看到RCC的结构。由如下寄存器组成：
- RCC_CR
- RCC_CFGR
- RCC_CIR
- RCC_APB2RSTR
- RCC_APB1RSTR
- RCC_AHBENR
- RCC_APB2ENR
- RCC_APB1ENR
- RCC_BDCR
- RCC_CSR
![[Pasted image 20250219145253.png]]
如上图所示，在stm32f103xb.h头文件看到RCC_TypeDef结构体跟6.3.11章节的RCC寄存器地址映像的结构一模一样。所以，就算没有stm32f103xb.h头文件，我们也能根据STM32F1参考手册，定义各个外设寄存器的结构体。

#### 3.2.1.5、有了结构体，有了起始地址
![[Pasted image 20250219145917.png | 800]]
如上所示，`RCC_BASE`是RCC寄存器的起始地址，`RCC_TypeDef`是结构体，那么`RCC_TypeDef*`是一个结构体指针。RCC通过宏定义变成一个`RCC_TypeDef`类型结构体指针。后续我们可以通过`RCC->`去访问RCC内部各个寄存器了。**这是一个非常常用的方法来抽象硬件寄存器，使得代码在可读性和维护性上更易于管理。它利用了 C 语言的结构体指针特性，让硬件寄存器的访问看起来更像是在操作一个普通的结构体变量。**

### 3.2.2、启用PWR电源时钟接口时钟
跟3.2.1章节一样，简单梳理一下。``LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR)``的作用是启用PWR电源时钟接口时钟。宏`LL_APB1_GRP1_PERIPH_PWR`相当于 1UL << 28U。
![[Pasted image 20250219180552.png | 800]]
![[Pasted image 20250219162559.png | 800]]
![[Pasted image 20250219162728.png | 800]]
如上两图所示。进入debug模式，打断点看到RCC的寄存器APB1ENR的bit28置1，启动PWR时钟。
![[Pasted image 20250219163057.png | 1100]]
如上图所示，《STM32F1参考手册》的6.3.8章节看到bit28的PWREN置1相当于启动电源接口时钟。

### 3.2.3、启用GPIOD与GPIOA时钟
![[Pasted image 20250219180518.png | 800]]
![[Pasted image 20250219180628.png | 800]]
如上图所示，`LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD)`启用了GPIOD组的时钟，`LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA)`启用了GPIOA组的时钟。其中宏`LL_APB2_GRP1_PERIPH_GPIOD`相当于1UL << 5UL，宏LL_APB2_GRP1_PERIPH_GPIOA相当于1UL << 2UL。
![[Pasted image 20250219181116.png | 1100]]
如山图所示，《STM32F1参考手册》的6.3.7章节看到位5是控制GPIOD时钟，位2是控制GPIOA时钟。

![[Pasted image 20250219181458.png | 800]]
![[Pasted image 20250219181529.png | 800]]
![[Pasted image 20250219181558.png | 800]]
如上图所示，用Debug模式观察寄存器的变化，执行完MX_GPIO_Init()之后，寄存器IOPAEN与IOPDEN都从0变成1。

### 3.2.4、编写自己的代码
![[Pasted image 20250219194209.png | 800]]
如上图所示，函数`Enable_Peripherals_Clock()`启动了这章节所用到的所有外设的时钟。
![[Pasted image 20250219194414.png | 800]]
如上图所示，把启动外设时钟的LL库注释掉。
### 3.2.5、调试自己编写的函数Enable_Peripherals_Clock()
![[Pasted image 20250219194539.png | 800]]
如上，`SET_BIT(RCC->APB2ENR, 1UL << 0UL);`，成功将寄存器APB2ENR的AFIOEN位置1，成功启动AFIO时钟。
![[Pasted image 20250219194703.png | 800]]
如上，`SET_BIT(RCC->APB1ENR, 1UL << 28UL);`，成功启动了PWR时钟。
![[Pasted image 20250219194737.png | 800]]
如上，`SET_BIT(RCC->APB2ENR, 1UL << 5UL);`，成功启动了GPIOD时钟。
![[Pasted image 20250219194804.png | 800]]
如上，`SET_BIT(RCC->APB2ENR, 1UL << 2UL);`，成功启动了GPIOA时钟。

## 3.3、配置NVIC优先级组

