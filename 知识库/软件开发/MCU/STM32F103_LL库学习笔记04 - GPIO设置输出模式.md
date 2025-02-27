# 导言
---
[[STM32F103_LL库学习笔记03 - GPIO设置输入模式，并轮询GPIO的电平状态]]，上一章节完成GPIO输入模式的设置。接下來梳理GPIO的输出模式配置，并且控制GPIO输出高电平或低电平。
首先，先梳理一下LL库怎样去设置GPIO的输出模式与控制GPIO输出高电平或低电平。

项目地址：https://github.com/q164129345/MCU_Develop/tree/main/stm32f103_ll_library04_gpio_output
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
![[LL04,GPIO_Output.gif | 800]]
如上所示，Watch1的pinStatus每隔1S变化一次状态（1->0->1）。while(1)死循环的程序在控制PB5一秒高电平，一秒低电平，一直循环。


## 2.4、通过LL库设置GPIO输出模式的另外一种方法
![[Pasted image 20250227151738.png | 800]]
如上图所示，通过函数`LL_GPIO_ResetOutputPin()`、`LL_GPIO_SetPinMode()`、`LL_GPIO_SetPinOutputType()`就能完成GPIO输出模式的配置。

# 三、寄存器的梳理
---
## 3.1、RCC时钟
![[Pasted image 20250226101728.png | 800]]
如上所示，因为PB5与PB4都属于GPIOB，所以时钟不用再增加。

## 3.2、GPIO相关寄存器
### 3.2.1、GPIOx_CRL
![[Pasted image 20250226102038.png | 800]]
```c
GPIOB->CRL &= ~(0xF << 20UL); // 清除段CNF5与MODE5
GPIOB->CRL |= (0x02 << 20UL); // 设置段CNF = 00,段MODE5 = 10
// 或者使用LL库提供的宏MODIFY_REG(),考虑到原子性的话，优先使用MODIFY_REG() 
// MODIFY_REG(GPIOB->CRL, 0x0F << 20UL, 0x02 << 20UL);
```
![[Pasted image 20250226102716.png | 800]]
如上是最终的结果。

### 3.2.2、GPIOx_ODR与GPIOx_BRR与GPIOx_BSRR的区别
在STM32F103中，GPIO的电平控制主要涉及以下几个寄存器：
1. `ODR`（输出数据寄存器）：直接控制GPIO引脚的输出电平状态。
2. `BSRR`（位设置/复位寄存器）：用于原子化地设置或复位引脚电平。
3. `BRR`（位复位寄存器）：专门用于原子化地将引脚电平复位（拉低）。

**总的来说，ODR不建议使用，不利于构建复杂的程序，原因是它不具备原子性。应该学习LL库，使用BSRR与BRR寄存器来控制GPIO，操作是原子性的。**

>**为什么原子性重要？**
>在嵌入式系统中，程序可能同时处理多个任务（比如主循环、中断、定时器等），这就可能导致竞争条件（Race Condition）。如果一个操作不是原子的，就可能在执行过程中被打断，造成不可预期的结果。


#### 3.2.2.1、ODR（输出数据寄存器）
![[Pasted image 20250226114522.png]]
- **功能**：是直接反映和控制GPIO端口的所有引脚输出状态。
- **特点**：读写操作是非原子的，可能影响整个端口的所有引脚。
- **优点**：简单直观，适合单次操作或调试。
- **缺点**：在多任务或中断环境下，读-改-写可能导致竞争条件（race condition），因为操作不是原子的。
```c
GPIOB->ODR |= (1 << 5);  // PB5置高
GPIOB->ODR &= ~(1 << 5); // PB5置低
```

#### 3.2.2.2、BSRR（位设置/复位寄存器）
![[Pasted image 20250226115335.png]]
- **功能**：通过单次写操作，原子化地设置或复位引脚电平。
- **特点**：
	- 原子操作，不会干扰其他引脚状态。
	- 单次写操作即可完成设置或复位。
- **优点**：线程安全，适合多任务或中断环境。
- **缺点**：需要分别处理置高和置低，代码稍显复杂。
```c
GPIOB->BSRR = (1 << 5);  // PB5置高
GPIOB->BSRR = (1 << (5 + 16)); // PB5置低（位21）
```

#### 3.2.2.3、BRR（位复位寄存器）
![[Pasted image 20250226115358.png]]
- **功能**：专门用于原子化地将引脚复位（拉低）。
- **特点**：
	- 仅用于复位（拉低），没有置高功能。
	- 与 BSRR 的高16位功能类似，但更简洁。
- **优点**：专为复位设计，操作简单，原子性保证。
- **缺点**：功能单一，只能拉低电平。
```c
GPIOB->BRR = (1 << 5); // PB5电平拉低
```


# 四、寄存器方式的实现
----
![[Pasted image 20250226181421.png | 800]]
![[Pasted image 20250226181454.png | 800]]
如上所示，在函数`GPIO_Configure()`设置GPIO输出推挽模式，在`main()`的死循环`while(1){}`里控制PB5每1S切换一次电平状态（高电平->低电平->高电平）。

![[LL04,GPIO_Output1.gif | 800]]
如上所示，从debug模式的Watch1看到全局变量pinStatus每隔1S切换一次电平状态。
![[Pasted image 20250226182232.png | 800]]
如上所示，从debug模式的寄存器窗口看到GPIB的寄存器CRL的段MODE5 = 0x02(输出模式，最大速度2MHz)、段CNF5 = 0x00(通用推挽输出模式)。
所以，寄存器方式的实现是没有问题的。由此看到，寄存器方式实现的代码量真的很简洁！！

# 五、细节补充
---
## 5.1、GPIO最后一个寄存器GPIOx_LCKR
![[Pasted image 20250226182920.png | 800]]
GPIOx_LCKR（Lock Configuration Register，端口配置锁定寄存器）的主要功能是锁定GPIO引脚的配置，防止意外修改。简单来说，它就像给GPIO配置（比如输入/输出模式、上下拉状态等）加了一把“锁”，**一旦锁定，除非复位芯片，否则无法再更改这些配置。**

**锁定指的是对寄存器GPIOx_CRL与GPIOx_CRH的锁定，其他寄存器ODR、IDR、BSRR、BRR都能正常使用。** 换句话说，锁定只冻结了“引脚的功能设置”，而引脚的实际操作（输入读取或输出控制）依然正常工作。

GPIOx_LCKR的设计初衷是为了提高系统的安全性和稳定性，尤其是在以下场景中非常有用：
1. **防止关键引脚配置被意外修改**
	- 假设你用PB5控制一个关键设备（比如电源开关或安全阀），一旦配置为输出模式，你不希望程序运行中被其他代码（比如调试代码或错误逻辑）意外改成输入模式，导致设备失控。
2. **嵌入式系统的高可靠性需求**
	- 在工业控制或汽车电子中，GPIO可能连接到传感器或执行器，配置错误可能导致严重后果（如生产线停机或安全事故）。锁定后，即使程序出现bug，也无法破坏GPIO配置。
3. **调试或开发中的保护**
	- 开发过程中，你可能频繁修改代码，有时会不小心覆盖某些引脚的配置（比如把输出改成输入）。如果某个引脚连接了外部硬件，误操作可能损坏硬件。
4. **防止软件攻击或干扰**
	- 在一些安全敏感的应用中（比如加密设备），攻击者可能通过软件漏洞试图修改GPIO配置，干扰硬件行为。锁定后，即使软件被攻破，也无法改变引脚功能。

示例代码：
```c
#include "stm32f1xx_ll_gpio.h"

void lock_PB5(void) {
    // 假设PB5已配置为输出模式
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
    GPIOB->CRL &= ~(0xF << 20UL); // 清除PB5配置
    GPIOB->CRL |= (0x02 << 20UL); // 推挽输出，2MHz

    // 锁定PB5配置（这个流程在参考手册有说明）
    GPIOB->LCKR = (1 << 5) | (1 << 16); // LCK[5] = 1, LCKK = 1
    GPIOB->LCKR = (1 << 5);             // LCK[5] = 1, LCKK = 0
    GPIOB->LCKR = (1 << 5) | (1 << 16); // LCK[5] = 1, LCKK = 1
    // 读取确认锁定
    if (GPIOB->LCKR & (1 << 16)) {
        // 锁定成功
    }
}

int main(void) {
    lock_PB5();
    while (1) {
        // PB5的配置已锁定，后续无法修改CRL/CRH
        GPIOB->BSRR = (1 << 5);  // 仍然可以控制电平
        for (volatile int i = 0; i < 100000; i++);
        GPIOB->BRR = (1 << 5);
        for (volatile int i = 0; i < 100000; i++);
    }
}
```

