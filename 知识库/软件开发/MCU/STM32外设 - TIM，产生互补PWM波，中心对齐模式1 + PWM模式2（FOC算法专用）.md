# 导言
---
![[1551001463-1-16 1.mp4 ]]

![[Pasted image 20241127152901.png]]
在FOC算法里，SVPWM用于产生三相PWM波给电机。为了更好地生成SVPWM波形，STM32的高级定时器TIM使用互补PWM的中心对齐模式1可以很好地实现。
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
![[Pasted image 20241127153711.png]]

## 1.2、TIM
### 1.2.1、TIM1 Mode and Configuration
![[Pasted image 20241127153939.png]]
如上图所示，配置三路PWM互补输出，时钟一定要选择内部时钟。

### 1.2.2、Parameter Settings
![[Pasted image 20241127162033.png]]
剩下的PWM Channel2与PWM Channel3都跟PWM Channel1的设置一样。

### 1.2.3、GPIO Settings
![[Pasted image 20241127162319.png]]
如上图所示，最后检查一下GPIO Speed是不是都设置High。

# 二、代码
---
## 2.1、main.c
![[Pasted image 20241127162603.png]]
如上所示，启动3路PWM互补PWM波形，并设置对应的占空比，4200相当于占空比50%。

# 三、细节补充
---
## 3.1、
