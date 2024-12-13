# 导言
---
本实验参考安富莱教程《[【专题教程第6期】SEGGER的J-Scope波形上位机软件，RTT模式波形上传速度可狂飙到500KB/S左右](https://www.armbbs.cn/forum.php?mod=viewthread&tid=86881)》
还有参考J-Scope的官方教程:https://kb.segger.com/UM08028_J-Scope
![[J-Scope_rtt.gif | 1100]]
如上图所示，正弦波跟PWM波的周期都是1ms，定时器的回调周期是100us。
项目源码:https://github.com/q164129345/MCU_Develop/tree/main/jlink_scope_rtt

# 一、CubeMX
---
## 1.1、TIM
![[Pasted image 20241212170823.png | 900]]
TIM14的中断周期从1000us改为100us，数据的发送频率从1KHz变成10KHz。

# 二、代码
---
## 1.1、main.c
![[Pasted image 20241212191143.png | 900]]
![[Pasted image 20241212191241.png | 900]]
![[Pasted image 20241212191408.png | 900]]![[Pasted image 20241212193845.png | 900]]
编译OK，下载程序。

# 三、J-Scope
---
![[Pasted image 20241212194125.png]]
![[Pasted image 20241212194401.png | 900]]
![[Pasted image 20241212194611.png | 900]]
# 四、细节补充
---
## 4.1、J-Scope支持哪些数据类型？
从官方文档看到，现在8.10d版本支持的类型如下：
- bool
- float
- uint8_t、uint16_t、uint32_t
- int8_t、int16_t、int32_t
![[Pasted image 20241212195020.png | 1100]]

## 4.2、通道的命名格式
![[Pasted image 20241212195316.png | 1000]]
![[Pasted image 20241212195619.png | 1000]]
比如我希望发送的消息是如下的结构体或者单个变量:
```c
// "JScope_t4f4u4"
typedef struct {
	uint32_t timestamp;
	float    temp;
	uint32_t distance;
}

// "JScope_u4i4"
typedef struct {
	uint32_t distance;
	int32_t value;
}

// "JScope_u4"
uint32_t value; 

// "JScope_f4"
float temp;
```

## 4.3、时间戳问题
![[Pasted image 20241213113304.png]]
大概的意思是：
RTT 通道未设置提供时间戳！  
没有时间戳，J-Scope 将假设数据点是以 100 微秒的间隔均匀分布的。




