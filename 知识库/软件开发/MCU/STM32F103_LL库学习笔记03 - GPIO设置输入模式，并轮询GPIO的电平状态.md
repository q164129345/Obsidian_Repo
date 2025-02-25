# 导言
---
[[STM32F103_LL库学习笔记02 - 开启SysTick（滴答定时器）中断]]上一章节完成SysTick中断。接着，开始梳理大家肯定逃不过的外设GPIO。
首先，先梳理一下LL库怎样去设置GPIO的模式，读取GPIO的电平的状态。
# 一、CubeMX
---
![[Pasted image 20250225114450.png | 1100]]
![[Pasted image 20250225114540.png | 1100]]
如上所示，使用CubeMX设置PB4为输入模式，Pull-up（上拉）。

# 二、代码
---
## 2.1、MX_GPIO_Init()
![[Pasted image 20250225142118.png | 1100]]
![[Pasted image 20250225150607.png | 1100]]
如上所示，**使用外设之前记得先打开对应的时钟。** LL库函数`LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB)`的目的是打开GPIOB的时钟。然后，使用结构体`LL_GPIO_InitTypeDef`来填写GPIO的功能，然后调用`LL_GPIO_Init()`进行GPIO初始化。
![[Pasted image 20250225145841.png | 1100]]
如上所示，结构体`LL_GPIO_InitTypeDef`定义了5个结构体成员，通过这5个成员的组合，配置GPIO的工作模式。
为了实现抽象，方便使用结构体来设置GPIO的各个模式，函数`LL_GPIO_Init()`的定义相当复杂，如下所示：
```c
ErrorStatus LL_GPIO_Init(GPIO_TypeDef *GPIOx, LL_GPIO_InitTypeDef *GPIO_InitStruct)
{
  uint32_t pinmask;
  uint32_t pinpos;
  uint32_t currentpin;

  /* Check the parameters */
  assert_param(IS_GPIO_ALL_INSTANCE(GPIOx));
  assert_param(IS_LL_GPIO_PIN(GPIO_InitStruct->Pin));

  /* ------------------------- Configure the port pins ---------------- */
  /* Initialize  pinpos on first pin set */

  pinmask = ((GPIO_InitStruct->Pin) << GPIO_PIN_MASK_POS) >> GPIO_PIN_NB;
  pinpos = POSITION_VAL(pinmask);

  /* Configure the port pins */
  while ((pinmask  >> pinpos) != 0u)
  {
    /* skip if bit is not set */
    if ((pinmask & (1u << pinpos)) != 0u)
    {
      /* Get current io position */
      if (pinpos < GPIO_PIN_MASK_POS)
      {
        currentpin = (0x00000101uL << pinpos);
      }
      else
      {
        currentpin = ((0x00010001u << (pinpos - GPIO_PIN_MASK_POS)) | 0x04000000u);
      }

      if (GPIO_InitStruct->Mode == LL_GPIO_MODE_INPUT)
      {
        /* Check The Pull parameter */
        assert_param(IS_LL_GPIO_PULL(GPIO_InitStruct->Pull));

        /* Pull-up Pull-down resistor configuration*/
        LL_GPIO_SetPinPull(GPIOx, currentpin, GPIO_InitStruct->Pull);
      }
      
      /* Check Pin Mode parameters */
      assert_param(IS_LL_GPIO_MODE(GPIO_InitStruct->Mode));
      
      /* Pin Mode configuration */
      LL_GPIO_SetPinMode(GPIOx, currentpin, GPIO_InitStruct->Mode);

      if ((GPIO_InitStruct->Mode == LL_GPIO_MODE_OUTPUT) || (GPIO_InitStruct->Mode == LL_GPIO_MODE_ALTERNATE))
      {
        /* Check speed and Output mode parameters */
        assert_param(IS_LL_GPIO_SPEED(GPIO_InitStruct->Speed));
        assert_param(IS_LL_GPIO_OUTPUT_TYPE(GPIO_InitStruct->OutputType));

        /* Speed mode configuration */
        LL_GPIO_SetPinSpeed(GPIOx, currentpin, GPIO_InitStruct->Speed);

        /* Output mode configuration*/
        LL_GPIO_SetPinOutputType(GPIOx, currentpin, GPIO_InitStruct->OutputType);
      }
    }
    pinpos++;
  }
  return (SUCCESS);
}
```

## 2.2、读取GPIO的电平状态LL_GPIO_IsInputPinSet()
通过2.1章节的函数`MX_GPIO_Init()`对PB4进行初始化后，接着可以通过函数`LL_GPIO_IsInputPinSet()`来获取当前PB4是高电平还是低电平。
![[Pasted image 20250225154413.png | 1100]]
![[LL03,gpio_input.gif | 1100]]
如上图所示，pinStatus刚开始等于1，因为PB4的初始状态是上拉。当我将PB4连接到GND，pinStatus变成0。当我再一次将PB4与GND断开，pinStatus又变回1。

# 三、寄存器梳理
---
## 3.1、GPIOB的RCC时钟
![[Pasted image 20250225162005.png]]
如上所示，从《STM32F1参考手册》的章节2.1-系统结构的系统结构看到，外设GPIOB的时钟源来自APB2。
![[Pasted image 20250225162557.png]]
![[Pasted image 20250225162636.png]]
如上所示，《STM32F1参考手册》的章节6.3.7，将外设RCC的寄存器ARB2ENR的bit3置1，即打开GPIOB的时钟。

```c
RCC->APB2ENR |= 0x01UL << 3UL; // 使能GPIOB的时钟
// RCC->APB2ENR |= RCC_APB2ENR_IOPBEN; //两种方法等效的
```


## 3.2、GPIO相关寄存器
### 3.2.1、GPIOB_CRL
![[Pasted image 20250225163638.png]]
如上所示，《STM32F1参考手册》的章节8.2.1，首先要配置GPIO的寄存器CRL（PB0～PB7）或者CRH（PB8～PB15）。比如PB4要设置段MODE4 = 00（输入模式），段CNF4 = 10（上拉/下拉输入模式）。
```c
GPIOB->CRL &= ~(0xF << 16UL); // 清除段CNF4与段MODE4
GPIOB->CRL |= 0x08 << 16UL;   // 设置段CNF4 = 10，段MODE4 = 00
// 或者使用LL库提供的宏MODIFY_REG(),考虑到原子性的话，优先使用MODIFY_REG()
// MODIFY_REG(GPIOB->CRL, 0x0F << 16UL, 0x08 << 16UL); 
```

### 3.2.2、GPIOB_ODR
![[Pasted image 20250225170233.png]]
如上所示，《STM32F1参考手册》的章节8.2.4，寄存器ODR对应的位置1相当于上拉，置0相当于下拉。
```c
GPIOB->ODR |= (0x01 << 4UL); // 置1相当于上拉
GPIOB->ODR &= ~(0x01 << 4UL); // 置0相当于下拉
```

### 3.2.3、PB4输入模式，上拉
```c
// 假设已经使能GPIOB的时钟
MODIFY_REG(GPIOB->CRL, 0x0F << 16UL, 0x08 << 16UL); // PB4输入模式、上拉/下拉输入模式
SET_BIT(GPIOB->ODR, 0x01 << 4UL); // 上拉 , 等效GPIOB->ODR |= (0x01 << 4UL)
```
通过上面两句代码，让PB4设置输入模式，且上拉。

### 3.2.4、读取PB4的电平状态(GPIOB_IDR)
![[Pasted image 20250225171505.png]]
如上所示，《STM32F1中文参考手册》的章节8.2.3，寄存器IDR4对应的是PB4的电平状态。IDR4 = 1相当于PB4高电平，IDR4 = 0相当于PB4低电平。
```c
if(GPIOB->IDR & (0x01 << 4UL)) {
	// PB4为高电平
} else {
	// PB4为低电平
}

// 等效实现方式
if (READ_BIT(GPIOB->IDR, 0x01 << 4UL)) {
    // PB4为高电平
} else {
	// PB4为低电平
}

```

# 四、寄存器方式实现
![[Pasted image 20250225173944.png | 800]]
![[Pasted image 20250225174110.png | 800]]
![[LL03,read_gpio.gif| 800]]
如上所示，测试效果一直。
![[Pasted image 20250225174520.png | 800]]
如上所示，通过debug模式的寄存器状态看到，寄存器GPIOB_CRL的MODE4 = 0x00(输入模式）与CNF4 = 0x02（上拉/下拉的输入模式）。



