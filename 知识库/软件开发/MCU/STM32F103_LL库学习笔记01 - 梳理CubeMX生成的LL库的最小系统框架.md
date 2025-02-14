# 导言
---
《[[STM32 - 在机器人领域，LL库相比HAL优势明显]]》
在机器人、自动化设备领域使用MCU开发项目，必须用LL库。本系列笔记记录使用LL库的开发过程，并且对照《STM32F1中文参考手册》，梳理寄存器的使用。
《STM32F1中文参考手册》一定要多看，读烂它！！
《STM32F1中文参考手册》一定要多看，读烂它！！
《STM32F1中文参考手册》一定要多看，读烂它！！

最小系统框架包括RCC Mode与SYS Mode的设置，如下所示：
![[Pasted image 20250214141349.png | 500]]
生成LL库的方法，如下所示：
![[Pasted image 20250214121252.png | 1100]]


# 一、RCC时钟，对应《STM32F1中文参考手册-6.2时钟》
---
## 1.1、CubeMX - RCC
![[20250214-120135.jpg | 1100]]
使用CubeMX创建项目，首先要把RCC时钟配置好。如上所示，使能HSE时钟连接外部晶振。接着，单片机的PD0与PD1接口被复用成RCC_OSC_IN与RCC_OSC_OUT，用于连接外部高速晶振。下图是某块FOC控制板的原理图，从图上看到，OSC_IN与OSC_OUT连接了一个外部8M的晶振。
![[Pasted image 20250214114334.png | 800]]


## 1.2、CubeMX - Clock Configuration
![[Pasted image 20250213205024.png | 900]]
![[Pasted image 20250214114454.png | 1000]]
如上两图所示：
CubeMX的Clock Configuration其实就是《STM32F1中文参考手册》6.2时钟章节的时钟数。单片机的引脚OSC_OUT（PD0）与OSC_IN（PD1）连接外部晶振（一般8M），通过PLLMUL模块后将频率加倍（倍频）至72M（最大频率）。*注：STM32F407的系统时钟最大168M。*

# 二、CubeMX - SYS
---
![[Pasted image 20250214170722.png | 1100]]
如上所示：
- Debug(调试端口)选择Serial Wire时，相当于只开SWD，关闭JTAG。这样，最节省GPIO，只用了PA13与PA14。
- Timebase Source选择SysTick，不占用其他TIM外设。这样是节省资源的方案。







# 三、代码
---



