### 一、扎实基础（经常被忘记，多复习）
---
[[STM32内存管理 - 堆栈（栈）与堆的区别]]
[[STM32内存管理 - ROM与 RAM由哪些部分组成]]
[[STM32内存管理 -  程序保存在flash，为什么STM32的程序可以小块提取，再小块运行？]]
[[STM32内存管理 - .map文件的结构]]
[[STM32程序优化 - volatile的正确理解与使用]]
[[STM32时间戳 -  计算64位的上电运行总时间（RTX5）]]
[[STM32异常 - HardFault_Handler处理方法]]

### 二、HAL库
----
[[STM32外设 - HAL库 - USART01,STM32F407,DMA接收空闲中断与DMA发送]]
[[STM32外设 - HAL库 - USART02,使用C语言面向对象的方式改写上一篇代码]]
[[STM32外设 - HAL库 - CAN,bsp_can.c与bsp_can.h备忘录]]
[[STM32外设 - HAL库 - TIM，产生互补PWM波，中心对齐模式1 + PWM模式2（FOC算法专用）]]

### 三、开发工具
---
[[STM32开发工具 - vscode插件Embedded IDE 导入Keil工程，开发编译、下载程序]]
[[STM32开发工具 - vscode插件 Cortex-Debug + ST-LINK与DAP-LINK与J-LINK调试MCU程序]]
[[STM32开发工具 - vscode插件Embedded IDE 使用OpenOCD在STM32F103芯片上下载失败]]
[[STM32开发工具 - vscode + Embedded 移植SEGGER RTT，打印log]]
[[STM32 - Embedded IDE01 - vscode搭建Embedded IDE开发环境（支持JLINK、STLINK、DAPLINK）]]
[[STM32 - Embedded IDE02 - 调试、断点、观察全局变量（支持JLINK、STLINK、DAPLINK）]]

[[STM32 - RTT组件01 - 移植]]
[[STM32 - RTT组件02 - RTT Viewer替代串口调试，实时打印调试log]]
[[STM32 - RTT组件03 - RTT Viewer测试回环打印]]
[[STM32 - RTT组件04 - RTT Viewer接入rt_thread console]]
[[STM32 - RTT组件05 - J-Scope数据可视化，使用HSS方式]]
[[STM32 - RTT组件06 - J-Scope数据可视化，使用RTT方式]]
[[STM32 - RTT组件07 - J-Scope数据可视化，RTT方式+DWT相对时间戳]]

### 四、开源库
---
1. [Ringbuffer](https://github.com/xiaoxinpro/QueueForMcu)
2. [letter_shell](https://github.com/NevermindZZT/letter-shell)
3. [CmBacktrack](https://github.com/armink/CmBacktrace)

### 五、RT_Thread
----
[[STM32系统 - RT_Thread系统移植]]
[[STM32系统 - RT_Thread移植控制台FinSH（串行方式）]]
[[STM32系统 - RT_Thread,MSH_CMD_EXPORT()注册控制台字符串指令，并传递参数]]
[[STM32系统 - RT_Thread,注册控制台字符串指令，并传递参数]]

### 六、LL库
[[STM32 - 在机器人领域，LL库相比HAL优势明显]]
[[STM32F103_LL库+寄存器学习笔记01 - 梳理CubeMX生成的LL库最小的裸机系统框架]]
[[STM32F103_LL库+寄存器学习笔记02 - 开启SysTick（滴答定时器）中断]]

**GPIO专题**
[[STM32F103_LL库+寄存器学习笔记03 - GPIO设置输入模式，并轮询GPIO的电平状态]]
[[STM32F103_LL库+寄存器学习笔记04 - GPIO设置输出模式]]
[[STM32F103_LL库+寄存器学习笔记05 - GPIO输入模式，捕获上升沿进入中断回调]]

**串口专题**
[[STM32F103_LL库+寄存器学习笔记06 - 梳理串口与串行发送“Hello,World"]]
[[STM32F103_LL库+寄存器学习笔记07 - 串口接收缓冲区非空中断]]
[[STM32F103_LL库+寄存器学习笔记08 - DMA串口发送，开启DMA传输完成中断]]
[[STM32F103_LL库+寄存器学习笔记09 - DMA串口接收与DMA串口发送，串口接收空闲中断]]
[[STM32F103_LL库+寄存器学习笔记10 - DMA传输过半+DMA传输完成中断实现DMA串口接收"双缓冲"]]
[[STM32F103_LL库+寄存器学习笔记11 - 串口收发的中断优先级梳理]]
[[STM32F103_LL库+寄存器学习笔记12 - 提高串口通讯程序的健壮性：异常监控 + 超时保护机制]]
[[STM32F103_LL库+寄存器学习笔记12.1 - 串口DMA高效收发实战：引入ringbuffer结构]]
[[STM32F103_LL库+寄存器学习笔记12.2 - 串口DMA高效收发实战2：进一步提高串口接收的效率]]

**CAN专题**
[[STM32F103_HAL库+寄存器学习笔记13 - 梳理外设CAN与如何发送CAN报文（串行发送）]]
[[STM32F103_HAL库+寄存器学习笔记14 - CAN发送完成中断]]
[[STM32F103_HAL库+寄存器学习笔记15 - 梳理CAN发送失败时，涉及哪些寄存器]]
[[STM32F103_HAL库+寄存器学习笔记16 - 监控CAN发送失败（轮询方式）]]
[[STM32F103_HAL库+寄存器学习笔记16.1 - 监控CAN发送失败（轮询+中断方式），未能调试成功！！]]
[[STM32F103_HAL库+寄存器学习笔记17 - CAN中断接收 + 接收CAN总线所有报文]]
[[STM32F103_HAL库+寄存器学习笔记18 - CAN接收溢出中断]]
[[STM32F103_HAL库+寄存器学习笔记19 - CAN发送中断+CAN接收中断+接收所有CAN报文+ringbuffer数据结构]]
[[STM32F103_HAL库+寄存器学习笔记20 - CAN发送中断+ringbuffer + CAN空闲接收中断+接收所有CAN报文+ringbuffer]]
[[STM32F103_HAL库+寄存器学习笔记21 - CAN接收过滤器：CPU减负神器，提升系统效率的第一道防线]]

**定时器专题**
[[STM32F103_LL库+寄存器学习笔记22 - 基础定时器TIM实现1ms周期回调]]
[[STM32F103_LL库+寄存器学习笔记23 - PWM波形输出及软件方式调整周期与占空比]]
[[STM32F103_LL库+寄存器学习笔记24 - TIM产生中心PWM波，中心对齐模式1 + PWM模式2（FOC算法专用）]]

**bootloader专题**
[[STM32F103_Bootloader程序开发01 - 什么是IAP？跟OTA有什么关系？]]
[[STM32F103_Bootloader程序开发02 - Bootloader程序架构与STM32F103的Flash内存规划]]
[[STM32F103_Bootloader程序开发03 - 启动入口与升级模式判断(boot_entry.c与boot_entry.h)]]
[[STM32F103_Bootloader程序开发04 - App跳转模块(app_jump.c与app_jump.h)]]
[[STM32F103_Bootloader程序开发05 - Keil修改生成文件的路径与文件名，自动生成bin格式文件]]
[[STM32F103_Bootloader程序开发06 - IAP升级用的App.bin增加CRC32校验码，确保固件完整性，防止“变砖”]]
[[STM32F103_Bootloader程序开发07 - 使用J-Flash将App_crc.bin烧录到App下载缓存区，再校验CRC32，确认固件完整性]]
[[STM32F103_Bootloader程序开发08 - 将App下载缓存区的固件搬运到App区，运行新的App程序(op_flash.c与op_flash.h)]]
[[STM32F103_Bootloader程序开发09 - 恰到好处的Ymodem协议]]
[[STM32F103_Bootloader程序开发10 - 实现IAP通讯看门狗与提升“跳转状态机”的健壮性]]
[[STM32F103_Bootloader程序开发11 - 实现 App 安全跳转至 Bootloader]]
[[STM32F103_Bootloader程序开发12 - IAP升级全流程]]






