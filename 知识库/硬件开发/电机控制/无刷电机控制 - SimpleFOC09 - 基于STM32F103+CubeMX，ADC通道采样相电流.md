# 导言
---
开始移植SimpleFOC的电流环代码之前，先实现ADC通道采样相电流的功能。
![[Pasted image 20250120204223.png]]
本章节主要目的是完成函数_readADCVoltageInline(pinx)，它会被InlineCurrentSensor类使用，获取相电流。

## 为什么是InlineCurrentSense类，而不是LowsideCurrentSense类？
![[Pasted image 20250120204045.png | 1100]]
从SimpleFOC的源码看到，current_snese里一共有三个类：
1. GenericCurrentSense（这个不清楚，暂时不管）
2. InlineCurrentSense（采样电阻在上桥臂与下桥臂之间）
3. LowsideCurrentSense（采样电阻在下桥臂下方且与GND连接。*多数大功率且高电压的FOC驱动板使用这个方案*）

![[Pasted image 20250120204425.png | 1100]]
如上图所示，选择InlineCurrentSense的原因是电路板的采样电路设计。

# 一、CubeMX
---
## 1.1、ADC1
参考以下博主的文章：
《[STM32CubeMX | STM32使用HAL库的ADC多通道数据采集（DMA+非DMA方式）+ 读取内部传感器温度](https://blog.51cto.com/u_15950551/6031866)》
《[STM32CUBEMX配置教程（十一）STM32的ADC轮询模式扫描多个通道](https://blog.csdn.net/weixin_44584198/article/details/119442880)》
![[Pasted image 20250120205111.png]]
### 1.1.1、Scan Conversion Mode（扫描模式）
目的：让 ADC 能依次转换多个规则通道（Rank1 → Rank2 → …）。
配置：在 CubeMX 中设置“Number of Conversion”为你要采样的通道数，比如 2 个。然后分别把你要采的通道（如 Channel3、Channel4）放到合适的顺位（Rank1、Rank2）。
效果：每次触发（软件触发或硬件触发）时，ADC 会顺序地把这 2 个通道都转换完，*实现了“一次触发，两通道采样”*。
### 1.1.2、Continuous Conversion Mode（连续模式）
目的：让 ADC 在完成一轮“扫描序列”后，自动立即再来下一轮，不停地转换。
配置：*如果只想“一次触发就采完全部通道并停下”，就关闭它（Disabled）*；如果要 ADC 不停地采样，则开启（Enabled）。

### 1.1.3、Discontinuous Conversion Mode（不连续模式）
目的：把本来一次要连着转换的全部通道分成若干小批次，每次只转换指定个数通道，然后等待下一个触发再转换后面的通道。
“Number of Discontinuous Conversions” 决定了“每次触发转换多少个通道”。
例如：假设总共要扫描 4 个通道，Number of Discontinuous Conversions=2，则每次触发会转换 2 个通道，真正完整采完 4 个通道需要触发 2 次。*这一次采样只有两个通道，触发一次即可。所以Disabled。*

### 1.1.4、总结
Scan Conversion Mode = Enabled
Continuous Mode = Disabled（若你只想单次采样）
Discontinuous Mode = Disabled
Number of Conversions = 2
Rank1 = Channel 3，Rank2 = Channel 4
然后 HAL_ADC_Start(&hadc1) 一次，就能顺序完成对 Channel 3、Channel 4 的转换。

# 1.2、ADC时钟频率
![[Pasted image 20250120210035.png | 1100]]


# 二、代码
---
## 2.1、adc.c
![[Pasted image 20250120210115.png | 1100]]



# 三、细节补充
----
## 3.1、ADC采样时间与转换时间

### 3.1.1、ADC采样时间
来自bilibili：《[[STM32 HAL库][ADC]采样时间和转换时间，最佳教程，没有之一](https://www.bilibili.com/video/BV1XM4m1y7eP/?spm_id_from=333.1391.0.0&vd_source=5a3da2d3c2504fa7535978c724745a9e)》的视频6：45分。
![[Pasted image 20250120174234.png]]
![[Pasted image 20250120174418.png]]
![[Pasted image 20250120174601.png]]
![[Pasted image 20250120174734.png]]
所谓采样时间，就是开关闭合，让模拟信号跑进电容Cadc，接着再断开开关，这个流程的时间。

### 3.1.2、采样时间怎样计算?
![[Pasted image 20250120175057.png]]
其中Radc=1k是MCU的内阻，它是固定的。电容Cadc也是固定的8pF。MCU外部的电阻有锂电池的内阻 + 电机相电阻 + 导线电阻等等。
![[Pasted image 20250120175416.png]]
如上图所示，博主计算了一番：
- 当MCU外面的电阻 = 400欧姆时，如果ADC运行频率 = 14MHz时，那么采样时间至少 = 1.54cycles
- 当MCU外面的电阻 = 10k欧姆时，如果ADC运算频率 = 14MHz时，那么采样时间至少 = 11.9cycles

>一般锂电池的电芯都是m欧姆级别，电机的相电阻一般也才几个欧姆，导线的电阻也很低。所示，就算我们当400欧姆来计算，1.54cycles也有一定的冗余。大部分的人都建议至少用7.5 cycles。
>如果ADC频率是12MHz，采样时间等于0.625us。1 / 12MHz = 83.3333ns、7.5 cycles * 83.333ns = 0.625us。

### 3.1.3、转换时间
![[Pasted image 20250120180154.png | 1100]]
如上所示，STM32F103的中文参考手册的161页有说明，转换时间是固定的12.5cycle。
>注：如果ADC频率是12MHz，转换时间等于1.0417us。 1 / 12MHz = 83.333ns、12.5cycle * 83.333ns = 1.0317us。

### 3.1.4、总时间
如果ADC频率是12MHz时，总时间 = 采样时间（7cycles = 0.625us) + 转换时间（12.5cycles = 1.0317us) = 1.6567us

