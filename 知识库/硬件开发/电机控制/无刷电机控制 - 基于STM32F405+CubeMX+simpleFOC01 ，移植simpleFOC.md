# 导言
---
![[Pasted image 20250704155102.png]]
如上所示，本次实验使用中空三相无刷电机 + STM32F405的FOC电机控制板。

年前完成STM32F103的simpleFOC的全部开发与实战：
[SimpleFOC STM32教程01｜基于STM32F103+HAL库，移植核心的common代码](https://blog.csdn.net/wallace89/article/details/145150130?sharetype=blogdetail&sharerId=145150130&sharerefer=PC&sharesource=wallace89&spm=1011.2480.3001.8118)
[SimpleFOC STM32教程02｜基于STM32F103+HAL库，通过AS5600编码器读取电机的角度](https://blog.csdn.net/wallace89/article/details/145150362?sharetype=blogdetail&sharerId=145150362&sharerefer=PC&sharesource=wallace89&spm=1011.2480.3001.8118)
[SimpleFOC STM32教程03｜基于STM32F103+HAL库，通过AS5600编码器计算电机的转速](https://blog.csdn.net/wallace89/article/details/145150472?sharetype=blogdetail&sharerId=145150472&sharerefer=PC&sharesource=wallace89&spm=1011.2480.3001.8118)
[SimpleFOC STM32教程04｜基于STM32F103+HAL库，完成三相半桥电路的驱动程序](https://blog.csdn.net/wallace89/article/details/145241260?sharetype=blogdetail&sharerId=145241260&sharerefer=PC&sharesource=wallace89&spm=1011.2480.3001.8118)
[SimpleFOC STM32教程05｜基于STM32F103+HAL库，实现速度开环控制](https://blog.csdn.net/wallace89/article/details/145244826?spm=1001.2014.3001.5502)
[SimpleFOC STM32教程06｜基于STM32F103+HAL库，编码器（位置传感器）的校准](https://blog.csdn.net/wallace89/article/details/145245312?sharetype=blogdetail&sharerId=145245312&sharerefer=PC&sharesource=wallace89&spm=1011.2480.3001.8118)
[SimpleFOC STM32教程07｜基于STM32F103+HAL库，位置闭环控制（位置、速度闭环、没有电流环）](https://blog.csdn.net/wallace89/article/details/145245459?sharetype=blogdetail&sharerId=145245459&sharerefer=PC&sharesource=wallace89&spm=1011.2480.3001.8118)
[SimpleFOC STM32教程08｜基于STM32F103+HAL库，速度闭环控制（没电流环）](https://blog.csdn.net/wallace89/article/details/145245644?sharetype=blogdetail&sharerId=145245644&sharerefer=PC&sharesource=wallace89&spm=1011.2480.3001.8118)
[SimpleFOC STM32教程09｜基于STM32F103+HAL库，ADC采样相电流](https://blog.csdn.net/wallace89/article/details/145311415?sharetype=blogdetail&sharerId=145311415&sharerefer=PC&sharesource=wallace89&spm=1011.2480.3001.8118)
[SimpleFOC STM32教程10｜基于STM32F103+HAL库，速度闭环控制（有电流环）](https://blog.csdn.net/wallace89/article/details/145382892?sharetype=blogdetail&sharerId=145382892&sharerefer=PC&sharesource=wallace89&spm=1011.2480.3001.8118)
[SimpleFOC STM32教程11｜基于STM32F103+HAL库，位置闭环控制（速度、电流闭环）](https://blog.csdn.net/wallace89/article/details/145536238?sharetype=blogdetail&sharerId=145536238&sharerefer=PC&sharesource=wallace89&spm=1011.2480.3001.8118)
[SimpleFOC STM32教程12｜基于STM32F103+HAL库，实战项目 - 棘轮手感的实现](https://blog.csdn.net/wallace89/article/details/145536446?spm=1001.2014.3001.5501)
[SimpleFOC STM32教程13｜基于STM32F103+HAL库，实战项目 - 挡块手感的实现](https://blog.csdn.net/wallace89/article/details/145536526?sharetype=blogdetail&sharerId=145536526&sharerefer=PC&sharesource=wallace89&spm=1011.2480.3001.8118)

接着，开始STM32F405的simpleFOC开发与实战。STM32F405 与 STM32F103 在 FOC（Field-Oriented Control，磁场定向控制）电机控制上的主要区别可以从性能、资源、外设支持、运算能力等多个角度来看。以下是它们的核心差异：

## 1. 处理器性能

| 对比项 | STM32F103 | STM32F405 |
|--------|-----------|-----------|
| CPU    | Cortex-M3，72MHz | Cortex-M4，168MHz（含FPU） |
| FPU（浮点运算） | ❌ 无 | ✅ 有（硬件FPU） |
| DSP指令集 | ❌ 无 | ✅ 有 |

> STM32F405 带有 FPU 和 DSP 指令集，能显著提升 FOC 中的变换计算、PI 控制和 SVPWM 计算效率。

## 2. 内存资源

| 对比项 | STM32F103 | STM32F405 |
|--------|-----------|-----------|
| SRAM   | 20 KB     | 112 KB    |
| Flash  | 64~512 KB | 512~1024 KB |

> F405 拥有更大的内存资源，支持更复杂的控制算法和更多缓存数据。

## 3. 定时器和 PWM 输出

| 对比项 | STM32F103 | STM32F405 |
|--------|-----------|-----------|
| 高级定时器 | TIM1、TIM8 | TIM1、TIM8 |
| 定时器分辨率 | 16位 | 16位 |
| 死区时间控制 | ✅ 有 | ✅ 有 |

> 两者都支持互补PWM和死区时间，但 F405 的定时器功能更灵活，适合更复杂控制需求。

## 4. ADC 特性

| 对比项 | STM32F103 | STM32F405 |
|--------|-----------|-----------|
| ADC分辨率 | 12-bit | 12-bit |
| ADC数量 | 2个 | 3个 |
| 同时采样通道数 | 2路 | 3路 |
| 转换速度 | 1Msps | 2.4Msps |

> F405 支持更高采样速度和三相电流同时采样，提升电流环的控制带宽和精度。

## 5. 运算能力对 FOC 的意义

| 运算任务 | STM32F103 | STM32F405 |
|----------|-----------|-----------|
| Clarke、Park变换 | 软件实现（慢） | 硬件FPU加速（快） |
| PI调节器 | 固定点软件实现 | 浮点快速计算 |
| SVPWM生成 | 算法需优化 | 算力足，浮点稳定实现 |

> F405 能支持更高频率控制循环（如20kHz），大幅提升 FOC 响应速度和控制精度。

## 6. 软件支持

| 对比项 | STM32F103 | STM32F405 |
|--------|-----------|-----------|
| Cube 支持 | STM32CubeF1 | STM32CubeF4 |
| Motor Control SDK | v5.x 及以下 | v6.x / v7.x |
| 高级控制支持 | ❌ 有限 | ✅ 支持 FOC + PFC |

> F405 支持最新的 ST 电机控制库，适合开发工业级应用。


## 7. 总结对比（适用于 FOC）

| 特性 | STM32F103 | STM32F405 |
|------|-----------|-----------|
| 控制精度 | 一般 | 高精度 |
| 采样速度 | 中等 | 高速 |
| 电流环频率 | ~10kHz | 可高至20kHz以上 |
| 适合场景 | 简单低速FOC、开源学习项目 | 高速、高精度、工业伺服 |

> ✅ **结论：**
> - STM32F103：适合低成本、入门 FOC 项目；
> - STM32F405：适合高性能、高响应、高精度的电机控制场景。

**项目地址：**
*   **Gitee (国内)**: https://gitee.com/wallace89/MCU_Develop/tree/main/simplefoc14_stm32f405_transplant_simplefoc
*   **GitHub**: https://github.com/q164129345/MCU_Develop/tree/main/simplefoc14_stm32f405_transplant_simplefoc

<br>
# 二、基于之前的STM32F103工程快速移植simpleFOC
---
![[Pasted image 20250703202101.png]]
如上所示，将components文件夹、SEGGER_RTT文件夹、simplefoc文件夹、user文件夹直接copy过去即可。

# 三、Keil
----
## 3.1、工程设置
![[Pasted image 20250703201618.png]]

## 3.2、创建工程文件夹
![[Pasted image 20250704112059.png]]

## 3.3、往项目文件夹添加代码
![[Pasted image 20250704154009.png]]
如上所示，对照之前的STM32F103工程，将代码都加入到新的STM32F405工程里。

细心的同学会发现，我没有移植以下的代码：
- AS5600_I2C（原因是本次项目的三相无刷电机的编码器不是AS5600，而是霍尔传感器）
- BLDCDriver3PWM（STM32F405的FOC控制板的三相逆变电路用6PWM，另外，之前STM32F103的FOC控制板的三相逆变电路是3PWM。后续会讲解怎样编写BLDCDriver6PWM的代码）
- InlineCurrentSense（采样电路后续再看）

![[Pasted image 20250704114922.png | 1100]]
添加完代码后，点击编译代码肯定会报错。原因是还没来得及配置头文件的搜索路径。

![[Pasted image 20250704143231.png]]
如上所示，按照之前的STM32F103项目的头文件路径来设置。

# 四、CubeMX
---
## 4.1、RCC
![[Pasted image 20250704165746.png | 1100]]
## 4.2、Clock Configuration
![[Pasted image 20250704165601.png | 1100]]

## 4.3、TIM4
![[Pasted image 20250704170438.png | 1100]]

## 4.4、TIM1
![[Pasted image 20250704170644.png]]

《[STM32F405 + CubeMX - 产生互补PWM波，中心对齐模式1 + PWM模式2（FOC专用）](https://blog.csdn.net/wallace89/article/details/144520720?spm=1001.2014.3001.5501)》，TIM1的CubeMX怎样配置6PWM的三相逆变电路驱动，参考之前的笔记即可。

# 五、代码
---
## 5.1、main.h
![[Pasted image 20250704163615.png]]

## 5.2、main.c
![[Pasted image 20250704164146.png]]
![[Pasted image 20250704164230.png | 1100]]

## 5.3、main.cpp
![[Pasted image 20250704164453.png | 1100]]
![[Pasted image 20250704164706.png | 1100]]
![[Pasted image 20250704173637.png | 1100]]

## 5.4、编译代码
![[Pasted image 20250704165233.png | 1100]]

## 5.5、烧录程序
![[Pasted image 20250704174153.png | 1100]]

## 5.6、RTT
![[Run_LED.gif]]
如上所示，从RTT打印的滴答定时器的计数看来，DWT的延迟精度不错。




