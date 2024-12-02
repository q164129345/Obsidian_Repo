# 导言
---
![[1551001463-1-16 1.mp4 ]]
![[Pasted image 20241202135439.png | 800]]

在FOC算法里，SVPWM用于产生三相PWM波给电机。为了生成SVPWM波形，STM32的高级定时器TIM使用互补PWM的中心对齐模式1可以很好地实现。
<br><br>

![[Pasted image 20241127155014.png]]
如上图所示，按照后面的笔记来配置TIM1后，可以产生的互补PWM波形。
1. 我们期望的SVPWM，需要每一个PWM周期里，高电平在中间，低电平在两侧。所以，选择==PWM_Mode2==。 PWM_Mode1是中间低电平，两侧高电平。
2. 每一个PWM周期都要通过ADC采样一次相电流。==当RCR设置1时== ，ARR寄存器的值倒数到0时会触发一次Update Event。从上图看到，Update Event时，在下半桥高电平（导通）的中间位置。我使用的FOC电路板，电流采样电阻在下半桥上，所以采样电流时，必须在下半桥导通的时候，如下图所示。
![[Pasted image 20241127161327.png]]
<br><br>
![[Pasted image 20241127161717.png]]
如上图所示，使用示波器监控出来的PWM互补波形。

# 一、CubeMX
---
## 1.1、Clock Configuration
![[Pasted image 20241127153711.png | 1000]]

## 1.2、TIM
### 1.2.1、TIM1 Mode and Configuration
![[Pasted image 20241127153939.png | 800]]
如上图所示，配置三路PWM互补输出，时钟一定要选择内部时钟。

### 1.2.2、Parameter Settings
![[Pasted image 20241127162033.png | 800]]
剩下的PWM Channel2与PWM Channel3都跟PWM Channel1的设置一样。

### 1.2.3、GPIO Settings
![[Pasted image 20241127162319.png | 800]]
如上图所示，最后检查一下GPIO Speed是不是都设置High。

# 二、代码
---
## 2.1、main.c
![[Pasted image 20241127162603.png | 800]]
如上所示，启动3路PWM互补PWM波形，并设置对应的占空比，4200相当于占空比50%。

# 三、细节补充
---
## 3.1、PWM Mode1 与 PWM Mode2的区别，为什么PWM Mode1不能使用
- PWM Mode1，向上计数时，PWM信号从有效电平变成无效电平（假如有效电平是高电平时，PWM信号从高电平变成低电平）；
- PWM Mode2，向上计数时，PWM信号从无效电平变成有效电平（假如有效电平是高电平时，PWM信号从低电平变成高电平）；

![[Pasted image 20241127181055.png]]
如上图所示：
1. 中心对齐模式1 + PWM Mode1的话，触发Update Event时，下半桥是断开的，电流没办法经过采样电阻。如果此时启动ADC采样的话，电流采样会出问题。
2. 我们期望的SVPWM，需要每一个PWM周期里，高电平在中间，低电平在两侧。所以PWM Mode1也不符合SVPWM的期望。
3. 综上所述，采样电阻在下桥的话，不能使用中心对齐模式1 + PWM Mode1。

## 3.2、为什么RCR设置1
![[Pasted image 20241127181838.png | 800]]
![[Pasted image 20241127184052.png | 800]]
如上图所示，中心对齐模式下，当RCR=1时，每一次ARR寄存器从最大值倒数至0时，都会触发一次Update_Event。又因为在PWM Mode2模式下，此时下桥导通，可以采样电流了。如下图所示：
![[Pasted image 20241127185013.png]]

<div class="warning">
<strong>警告</strong><br><br>如果采样电阻在上桥，就要使用中心对齐模式1 + PWM_Mode1 + RCR=1
</div>

## 3.3、TIM1触发Upate Event时，下桥真的已经导通吗？
### 3.3.1、CubeMX
![[Pasted image 20241127192502.png | 800]]

如上所示，使能TIM1的update interrupt。
### 3.3.2、tim.c
![[Pasted image 20241127192701.png | 800]]
如上所示，在tim.c代码里启动TIM1的中断。
### 3.3.3、main.c
![[Pasted image 20241127192806.png | 800]]
如上图所示，在Update Event中断时，翻转GPIO的电平。

### 3.3.4、示波器观察
![[Pasted image 20241128102034.png]]



