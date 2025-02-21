# 导言
---
《[[STM32 - 在机器人领域，LL库相比HAL优势明显]]》在机器人、自动化设备领域使用MCU开发项目，必须用LL库。
本系列笔记记录使用LL库的开发过程，首先通过CubeMX生成LL库代码，梳理LL库源码。通过学习LL库源码，弄清楚寄存器的使用。最后，删除LL库代码，编写自己的寄存器驱动代码验证一遍。
MCU开发的精髓在于寄存器。
MCU开发的精髓在于寄存器。
MCU开发的精髓在于寄存器。
《STM32F1中文参考手册》、《Cortex-M3技术参考手册》一定要多看，读烂它！！
《STM32F1中文参考手册》、《Cortex-M3技术参考手册》一定要多看，读烂它！！
《STM32F1中文参考手册》、《Cortex-M3技术参考手册》一定要多看，读烂它！！

## 外设的内存映射
**外设的内存映射分为两个区域：常规外设（peripherals）与核心外设（core peripherals)**。梳理寄存器，要先找到它们的内存地址。
![[Pasted image 20250220111326.png | 800]]
通过查阅《STM32F1中文参考手册》的章节2.3-内存映像、《STM32F1编程手册》的章节4-Core peripherals，针对外设的内存映射分为两个区域。一个是常规外设区域，内存地址从0x40000000开始。另外一个是核心外设区域，内存地址从0xE000E010。这样做的目的是根据功能进行分离管理。
- 常规外设，内存地址在0x40000000~0x5003FFFF，如UART、SPI、GPIO等。
- 核心外设，内存地址0xE000E010 ~ 0xE000EF03，如System Timter、System control block等。


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
如上所示，在《STM32F1参考手册》章节6.3.7找到APB2 外设时钟使能寄存器(RCC_APB2ENR)的详细信息，看到RCC的寄存器APB2ENR的bit0位是AFIO_EN。

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
### 3.2.5、调试自己编写的函数
```c
static void Enable_Peripherals_Clock(void) {
    SET_BIT(RCC->APB2ENR, 1UL << 0UL);  // 启动AFIO时钟
    SET_BIT(RCC->APB1ENR, 1UL << 28UL); // 启动PWR时钟
    SET_BIT(RCC->APB2ENR, 1UL << 5UL);  // 启动GPIOD时钟
    SET_BIT(RCC->APB2ENR, 1UL << 2UL);  // 启动GPIOA时钟
    __NOP(); // 稍微延时一下下
}
```
![[Pasted image 20250219194539.png | 800]]
如上，`SET_BIT(RCC->APB2ENR, 1UL << 0UL);`，成功将寄存器APB2ENR的AFIOEN位置1，成功启动AFIO时钟。
![[Pasted image 20250219194703.png | 800]]
如上，`SET_BIT(RCC->APB1ENR, 1UL << 28UL);`，成功启动了PWR时钟。
![[Pasted image 20250219194737.png | 800]]
如上，`SET_BIT(RCC->APB2ENR, 1UL << 5UL);`，成功启动了GPIOD时钟。
![[Pasted image 20250219194804.png | 800]]
如上，`SET_BIT(RCC->APB2ENR, 1UL << 2UL);`，成功启动了GPIOA时钟。

## 3.3、配置NVIC（Nested vectored interrupt controller）的中断优先级分组
### 3.3.1、寄存器SCB_AIRCR
![[Pasted image 20250220151044.png | 1100]]
如上图，摘自《STM32F1编程手册》的章节4.4.5，介绍寄存器SCB->AIRCR的结构，函数函数`__NVIC_SetPriorityGrouping()`的主要目的是修改段PRIGROUP(bit10~bit8)。为了修改段PRIGROUP必须往段VECTKEY写入0xFA05，否则不能往寄存器SCB->AIRCR写入任何值。

![[Pasted image 20250220152750.png | 1100]]
如上所示，根据《STM32F1编程手册》的章节4.1介绍，SCB(System control block)的内存映射起始地址是0xE000ED00，跟源码定义的指针地址SCB_BASE一样。所以，通过宏`#define SCB    (SCB_Type *)SCB_BASE`之后，可以通过结构体指针SCB->去访问SCB里面的各个寄存器。

### 3.3.2、LL库源码
![[Pasted image 20250220142931.png | 800]]
如上图所示，函数NVIC_SetPriorityGrouping()的作用是设置NIVC的中断优先级分组。优先级分组一共有4种设置（如下图所示），摘自《STM32F1编程手册》4.4.5章节：
![[Pasted image 20250220142151.png | 1100]]
如上图，为什么一般工程会选择NVIC_PRIORITYGROUP_4？
1. **最大抢占优先级数量。** 删除子优先级，剩下抢占优先级，这样可以区分更多中断的紧急程度（通常 4 位可以划分 16 个级别）。这在系统中存在大量中断且各中断优先级差异较大时非常有用。
2. **简化中断配置。** 由于没有子优先级，配置时只需关注抢占优先级，减少了设置上的复杂性，有助于避免在子优先级上的混淆和错误。
3. **更明确的中断抢占关系。** 每个中断的优先级完全由抢占优先级决定，当一个中断执行时，只有比它优先级高（数值更低）的中断才会抢占，这使得系统行为更直观。
总之，大多数工程选择 NVIC_PRIORITYGROUP_4，原因在于它能提供更多的抢占级别，适合中断较多、要求严格抢占控制的系统，同时也能简化配置和设计。

![[Pasted image 20250220140420.png | 800]]
如上所示，根据`#define NVIC_SetPriorityGrouping    __NVIC_SetPriorityGrouping`得知，其实NVIC_SetPriorityGrouping()还有一个别名__NVIC_SetPriorityGrouping()。接着，看看函数__NVIC_SetPriorityGrouping()是怎样实现的。
![[Pasted image 20250220140654.png | 800]]
如上图所示，在源码core_cm3.h的第1480行找到函数`__NVIC_SetPriorityGrouping()`的实现。可见，这个函数并不是LL库的，属于内核级别的源码。

![[Pasted image 20250220145017.png | 800]]
如上图，函数__NVIC_SetPriorityGrouping()的目的是保留其他位的内容，只更改VECTKEY段(bit31~bit16)与PRIGROUP段(bit10~bit8)的内容。**这是一种非常值得学习的标准的“读-修改-写”的标准写法。**
如果在设置NVIC的中断优先级分组时，确认过寄存器的其他位都是0的话，可以使用更加简洁的代码来实现：
```c
SCB->AIRCR = (0x5FA << SCB_AIRCR_VECTKEY_Pos) | (NVIC_PRIORITYGROUP_4 << SCB_AIRCR_PRIGROUP_Pos); // 0x5FA是寄存器AIRCR的写入钥匙，没有它，这个寄存器写不进去。 
```

### 3.3.3、debug模式查看寄存器SCB->AIRCR
![[Pasted image 20250220153304.png | 1100]]
![[Pasted image 20250220153341.png | 1100]]
从上两图看到，寄存器AIRCR的值从0xFA050000变成0xFA050300。
![[Pasted image 20250220153739.png]]
![[Pasted image 20250220153825.png]]
### 3.3.4、编写自己的函数
```c
static void Set_NVIC_PriorityGrouping(uint32_t group_numer) {
    SCB->AIRCR = (0x5FA << SCB_AIRCR_VECTKEY_Pos) | (group_numer << SCB_AIRCR_PRIGROUP_Pos);
}
```
![[Pasted image 20250220154454.png | 1100]]
![[Pasted image 20250220154524.png | 1100]]
![[Pasted image 20250220154601.png | 1100]]
如上两图，SCB->AIRCR从0xFA050000变成0xFA050300。

## 3.4、禁用JTAG，启用SWD调试接口
### 3.4.1、寄存器AFIO_MAPR
![[Pasted image 20250220195839.png | 1100]]
如上图，通过修改寄存器AFIO_MAPR的段SWJ_CFG(bit26~bit24)设置debug口。本例程只使用SWD接口，所以需要将段SWJ_CFG设置为010。

### 3.4.2、LL库源码
![[Pasted image 20250220193115.png | 800]]
函数`LL_GPIO_AF_Remap_SWJ_NOJTAG()`的作用是关闭JTAG接口，仅启动SWD接口。这个方案是最节省GPIO口的。
![[Pasted image 20250220195008.png | 800]]
函数`LL_GPIO_AF_Remap_SWJ_NOJTAG()`里只有一句代码，使用宏`MODIFY_REG()`去修改AFIO的寄存器MAPR。
![[Pasted image 20250220200414.png | 800]]
如上所示，宏`MODIFY_REG()`的作用是**先将寄存器中第二个参数所指定位清零，然后再将第三个参数所指定位设置为1。** 具体过程如下：
1. 读取原寄存器值：先通过 READ_REG(REG) 获取寄存器当前值。
2. 清零操作：使用 ~(CLEARMASK) 将第二个参数对应的位取反，再与原值进行按位与运算，从而将这些位清0。
3. 置位操作：将上一步的结果与第三个参数（SETMASK）进行按位或运算，将其中为1的位设置为1。
4. 写回寄存器：最终通过 WRITE_REG 把更新后的值写回寄存器。
所以，第二个参数确定了哪些位被清0，第三个参数则指定了哪些位被置1。

如果不考虑原子性较高的场合时，宏`MODIFY_REG()`相当于`READ_REG()`+`CLEAR_BIT()`+`SET_BIT()`的组合。
>Note：“原子性”是指一个操作在执行过程中不可被中断或分割，要么完全执行完毕，要么根本不执行，保证在整个过程中数据的一致性。

### 3.4.3、debug模式查看寄存器AFIO->MAPR
![[Pasted image 20250220204218.png | 1100]]
![[Pasted image 20250220204452.png | 1100]]
![[Pasted image 20250220204556.png]]
如上所示，执行代码`MODIFY_REG(AFIO->MAPR, AFIO_MAPR_SWJ_CFG, AFIO_MAPR_SWJ_CFG_JTAGDISABLE);`之后，段SWJ_CFG变成010了。

### 3.4.4、调试自己编写的函数
```c
__STATIC_INLINE void Set_USE_SWD_NOT_JTAG(void) {
    __IO uint32_t reg = READ_REG(AFIO->MAPR);
    CLEAR_BIT(reg, AFIO_MAPR_SWJ_CFG);  // AFIO_MAPR_SWJ_CFG = 0x07 << 24UL
    SET_BIT(reg, AFIO_MAPR_SWJ_CFG_JTAGDISABLE); // AFIO_MAPR_SWJ_CFG_JTAGDISABLE = 1UL << 25UL
    AFIO->MAPR = reg; // 设置AFIO
}
```
![[Pasted image 20250220205620.png | 1100]]
如上所示，AFIO_MAPR同样变成0x02000000。**另外，还是使用宏`MODIFY_REG`比较完美啊，又精简又保证原子性。**

## 3.5、配置系统时钟
![[Pasted image 20250221093058.png | 800]]
`SystemClock_Config()` 函数完成了以下任务：
1. 设置 Flash 等待周期为 2，以适应 72MHz 的系统时钟。
2. 启用 HSE 时钟（外部晶振，假设 8MHz），并等待稳定。
3. 配置 PLL 使用 HSE 作为输入，倍频 9 倍，输出 72MHz。
4. 启用 PLL 并等待锁定。
5. 设置 AHB、APB1 和 APB2 的时钟分频器：
	- AHB 时钟 = 72MHz。
	- APB1 时钟 = 36MHz。
	- APB2 时钟 = 72MHz。
6. 将系统时钟源切换到 PLL 输出（72MHz）。
7. 初始化 SysTick 定时器（1ms 中断），并设置系统核心时钟为 72MHz。
通过这些步骤，系统时钟被成功配置为 72MHz，AHB 时钟为 72MHz，APB1 时钟为 36MHz，APB2 时钟为 72MHz，SysTick 定时器每 1 毫秒中断一次。

### 3.5.1、设置 Flash 等待周期为 2，以适应 72MHz 的系统时钟
![[Pasted image 20250221102604.png | 800]]
![[Pasted image 20250221102959.png | 800]]
如上所示，设置FLASH的等待周期，需要了解外设FLASH的寄存器ACR。宏`FLASH_ACR_LATENCY`从代码上看到是0x07UL << 0UL。代码`MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, Latency)`的意思是先读取寄存器ACR原来的值，接着清除`FLASH_ACR_LATENCY`指定的位，然后按照变量`Latency`指定的位来置1。**这是一种非常值得学习的标准的“读-修改-写”的标准写法。**
![[Pasted image 20250221103152.png | 800]]
![[Pasted image 20250221103321.png | 800]]
![[Pasted image 20250221103408.png | 800]]
如上图，从《STM32F1参考手册》的章节2.3-存储器映像找到外设FLASH在内存上的起始地址是0x40022000。从代码看到，宏FLASH指向的内存地址确实是0x40022000。
![[Pasted image 20250221104157.png | 800]]
![[Pasted image 20250221104224.png | 800]]
从《STM32F1闪存编程手册》的章节3.1找到FLASH_ACR的内容，段LATENCY(bit2~bit0)的作用是配置FLASH等待周期。当前的工程使用系统时钟72M，所以段LATENCY需要设置010（相当于0x02)。
![[Pasted image 20250221105207.png | 800]]
debug模式调试寄存器状态看来，确实是0x02。
### 3.5.2、启用 HSE 时钟（外部晶振，假设 8MHz）
![[Pasted image 20250221105808.png | 800]]
![[Pasted image 20250221105848.png | 800]]
如上所示，启动HSE时钟，需了解外设RCC的寄存器CR。宏`RCC_CR_HSEON`，从代码上看到是0x01UL << 16UL。代码`SET_BIT(RCC->CR, RCC_CR_HSEON)`的意思是将宏`RCC_CR_HSEON`指定的位来置1。

![[Pasted image 20250221110124.png | 800]]
![[Pasted image 20250221110205.png | 800]]
如上所示，寄存器RCC_CR的位16是控制HSE振荡器开启或者关闭。
![[Pasted image 20250221110744.png | 800]]
![[Pasted image 20250221110818.png | 800]]
如上所示，寄存器CR的HSEON启动了。
### 3.5.3、配置 PLL 使用 HSE 作为输入，倍频 9 倍，输出 72MHz
![[Pasted image 20250221112235.png | 800]]
![[Pasted image 20250221113235.png | 800]]
如上图，源码分析：
1. 函数参数。
	- `LL_RCC_PLLSOURCE_HSE_DIV_1` = `(RCC_CFGR_PLLSRC | 0x00000000U)`。接着，`RCC_CFGR_PLLSRC` = 0x01UL << 16UL。
	- `LL_RCC_PLL_MUL_9` = `RCC_CFGR_PLLMULL9` = 0x07UL << 18UL。
2. 计算需要清除的位。
	- `RCC_CFGR_PLLSRC` = 0x01UL << 16UL。
	- `RCC_CFGR_PLLXTPRE` = 0x01UL << 17UL。
	- `RCC_CFGR_PLLMULL` = 0x01UL << 18UL。
	- `RCC_CFGR_PLLSRC` | `RCC_CFGR_PLLXTPRE` | `RCC_CFGR_PLLMULL`后相当于要清除位16、位17、位18。
![[Pasted image 20250221120145.png | 800]]
3. 计算需要置1的位。
	- 局部变量`Source` = `LL_RCC_PLLSOURCE_HSE_DIV_1` = 0x01UL << 16UL。
	- `RCC_CFGR_PLLSRC` = 0x01UL << 16UL。
	- `RCC_CFGR_PLLXTPRE` = 0x01UL << 17UL。
	- `Source` & (`RCC_CFGR_PLLSRC` | `RCC_CFGR_PLLXTPRE`) = 0x01UL << 16UL（即0x00010000）。
	- 局部变量`PLLMul` = `LL_RCC_PLL_MUL_9` = 0x07UL << 18UL(即0x001C0000)。
	- (`Source` & (`RCC_CFGR_PLLSRC` | `RCC_CFGR_PLLXTPRE`)) | PLLMul = 0x01UL << 16UL | 0x07UL << 18UL = 0x00010000 | 0x001C0000 = 0x001D0000，相当于位16、位18、位19、位20需要置1。
![[Pasted image 20250221120657.png | 800]]
 接着，进入debug模式调试看看。
 ![[Pasted image 20250221140732.png | 800]]![[Pasted image 20250221140914.png | 800]]
 如上两图所示，寄存器CFGR里的PLLSRC置1与PLLMUL的值变成0x07（二进制0111）。
 ![[Pasted image 20250221141054.png | 800]]
 如上所示：
 1. PLLSRC置1 = HSE时钟作为PLL输入时钟。
 2. PLLMUL的值是0x07(二进制0111) = PLL 9倍频输出。
![[Pasted image 20250221141335.png | 800]]
如上所示，CubeMX的Clock Configuration看到，8M晶振从HSE输入，接着通过PLLMUL的9倍频变成72M。

### 3.5.4、启动PLL，并等待锁定
![[Pasted image 20250221142226.png | 800]]
![[Pasted image 20250221142344.png | 800]]
如上图所示，通过外设RCC的寄存器CR来启动PLL。宏`RCC_CR_PLLON`= 0x01UL << 24UL。

![[Pasted image 20250221142636.png | 800]]
如上所示，RCC_CR的PLLON在bit14，当它置1时，PLL使能。

![[Pasted image 20250221142829.png | 800]]
![[Pasted image 20250221142955.png | 800]]
从debug调试看来，寄存器CR的PLLON被置1，PLL使能。从下面的PLLRDY看到，PLL已经完成使能。

![[Pasted image 20250221143201.png | 800]]
![[Pasted image 20250221143243.png | 800]]
如上所示，了解PLL是否已经完成锁定，还是从RCC_CR里看。宏`RCC_CR_PLLRDY` = 0x01UL << 25UL。

![[Pasted image 20250221143526.png | 800]]
如上图，《STM32F1参考手册》章节6.3.1，当RCC_CR的bit25变成1时，表示PLL已经锁定。

### 3.5.5、设置 AHB、APB1 和 APB2 的时钟分频器
![[Pasted image 20250221144114.png | 800]]
![[Pasted image 20250221144203.png | 800]]
如上图所示，设置AHB、APB1、APB2的分频都是在外设RCC的寄存器CFGR。其中，宏`RCC_CFGR_HPRE` = 0xF << 4UL,宏`RCC_CFGR_PPRE1` = 0x07 << 8UL，宏`RCC_CFGR_PPRE2` = 0x07 << 11UL。

![[Pasted image 20250221145224.png | 800]]
如上图所示:
- 设置AHB分频的是寄存器CFGR的段HPRE，一共4个bit（bit4～bit7）。
- 设置APB1分频的是寄存器CFGR的段PPRE1，一共3个bit（bit8～bit10）。
- 设置APB2分频的是寄存器CFGR的段PPRE2，一共3个bit（bit11～bit13）。

![[Pasted image 20250221145828.png | 800]]
从代码看来，APB1要2分频，其他两个不需要分频。按理来说，在保证外设能稳定工作的话，时钟频率越高越好。为什么APB1要2分频？接着分析。

![[Pasted image 20250221150222.png | 800]]
如上所示，《STM32F1参考手册》章节6.3.2有说明，APB1的时钟频率不能超过36MHz。因为之前PLL时钟频率设置了72MHz，所以APB1要在PLL时钟的前提下2分频得到时钟频率36MHz。

![[Pasted image 20250221150609.png | 800]]
如上所示：
- 段HPRE要设置二进制的000(相当于0x00000000 = 宏`LL_RCC_SYSCLK_DIV_1`)。
- 段PPRE1要设置二进制的100(相当于0x01UL << 10UL = 0x00000400 = 宏`RCC_CFGR_PPRE1_DIV2`)。
- 段PPRE2要设置二进制的0000(相当于0x00000000 = 宏`LL_RCC_APB2_DIV_1`)。

![[Pasted image 20250221151347.png | 800]]
进入debug模式看来，以上的分析是正确的。最终AHB的频率 = 72M = ARB2，只有APB1 = 72M / 2 = 36M。

### 3.5.6、将系统时钟源切换到 PLL 输出（72MHz）
![[Pasted image 20250221151753.png | 800]]
![[Pasted image 20250221151839.png | 800]]
如上所示，还是外设RCC的寄存器CFGR。

![[Pasted image 20250221151957.png | 800]]
如上所示，将寄存器CFGR的段SW设置为10（二进制）就可以将系统时钟设置为PLL输出。

![[Pasted image 20250221152226.png | 800]]
从debug模式观察到，SW确实被设置成10（十六进制0x02）。

![[Pasted image 20250221152408.png | 800]]
如上所示，记得检查系统时钟是不是已经切换到PLL时钟。

![[Pasted image 20250221152738.png | 800]]
如上所示，宏`RCC_CFGR_SWS` = 0x03 << 2UL，`READ_BIT(RCC->CFGR, RCC_CFGR_SWS)`相当于读取寄存器CFGR的位2～位3。

![[Pasted image 20250221152532.png | 800]]
如上所示，寄存器CFGR的位2～位3等于10(二进制)时，PLL输出已经作为系统时钟。

![[Pasted image 20250221153130.png | 800]]
如上所示，LL库源码确实是等待位2～位3等于10(二进制)。

### 3.5.7、初始化 SysTick 定时器（1ms 中断）
![[Pasted image 20250221153554.png | 800]]
![[Pasted image 20250221153640.png | 800]]
![[Pasted image 20250221161731.png | 800]]
如上所示，嵌套一套一套解开，最后是函数`LL_InitTick(72000000,1000U)`。

![[Pasted image 20250221170224.png | 800]]
1. 代码`SysTick->LOAD  = (uint32_t)((HCLKFrequency / Ticks) - 1UL)`
	- 将720000000代入HCLKFrequency，将1000代入Ticks后，SysTick->LOAD = 72000000 / 1000 - 1 = 72000 - 1 = 71999（十六进制0x0001193F）。
2. 代码`SysTick->VAL   = 0UL`
	- 将VAL计时器清0，准备开始计时。
3. 代码`SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk`
	- 宏`SysTick_CTRL_CLKSOURCE_Msk` = 0x01UL << 2UL
	- 宏`SysTick_CTRL_ENABLE_Msk` = 0x01UL
	- 所以`SysTick_CTRL_CLKSOURCE_Msk` | `SysTick_CTRL_ENABLE_Msk` = 0x05UL
![[Pasted image 20250221172330.png | 800]]
如上图所示，debug模式看到SysTick->CTRL确实等于0x05，还有SysTick->LOAD确认等于0x0001193F(十进制是71999)。

![[Pasted image 20250221172950.png | 800]]
如上所示，《STM32F1编程手册》的章节4.5.1 - SysTick control and status register (STK_CTRL)看到，外设SysTick的bit2-CLKSOURCE的作用是选择分频，然后bit0-ENABLE的作用是启动计数器，SysTick开始计数。
SysTick->CTRL = 0x05，相当于bit2与bit0置1。

![[Pasted image 20250221173326.png | 800]]
如上所示，SysTick->CTRL的设置相当于Clock Configuration图的后半段。
