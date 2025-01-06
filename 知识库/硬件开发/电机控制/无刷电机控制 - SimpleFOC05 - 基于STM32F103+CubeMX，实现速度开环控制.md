# 导言
---
![[Pasted image 20241231160807.png]]
如上所示，要让三相电机转起来，实现速度开环控制，需要完成FOC框架如下部分：
1. Inverse Park Transform(反帕克变换)
2. SVPWM Generator（SVPWM算法，或者克拉克逆变换）

效果如下，电机终于开始旋转起来了：
![[20250102-204654.mp4]]

RTT Viewer的打印，虽然是速度开环控制，但速度也接近控制的4rad/s：
![[rtt_open_vel.gif]]
项目源码:https://github.com/q164129345/MCU_Develop/tree/main/simplefoc05_stm32f103_open_vel_ctrl

# 一、CubeMX
---
## 1.1、TIM4
**TIM4的作用是周期性地执行FOC运算，周期暂时设置500us，即2KHz。**
![[Pasted image 20241227164949.png]]

![[Pasted image 20241227165025.png]]

# 二、simpleFOC源码
---
## 2.1、BLDCMotor类
![[Pasted image 20241231180512.png]]
![[Pasted image 20250102205835.png]]
![[Pasted image 20250102210033.png]]
这样修改后，程序就能编译成功。

## 2.2、simpleFOC官方的速度开环控制例子
![[Pasted image 20250103105844.png]]
![[Pasted image 20250103105936.png]]
如上所示，找到simpleFOC官方编写的速度开环控制例子。接下来，简单解释一下这个开环控制例程。本例程也是参考这个官方例子修改过来的。

### 2.2.1、open_loop_velocity_example
![[Pasted image 20250103110739.png]]
![[Pasted image 20250103111848.png]]
![[Pasted image 20250103173817.png]]

# 三、代码
---
## 3.1、user_main.cpp
![[Pasted image 20250103165007.png]]
如上图所示，按照simpleFOC官方的速度开环控制示例代码编写。
![[Pasted image 20250103170830.png]]
编译代码OK，下载运行即可。

# 四、细节补充
---
## 4.1、程序流程图
![[Pasted image 20250103180401.png]]
如上图所示，使用simpleFOC库，只需简单配置BLDCDriver类与BLDCMotor类就能实现开环速度控制。

## 4.2、J-Scope观察三相占空比的波形（漂亮的正弦波）
![[Pasted image 20250106192851.png]]
关于J-Scope的RTT方式观察波形的方法，参考笔记：《[[STM32 - RTT组件06 - J-Scope数据可视化，使用RTT方式]]》
如上图所示，通过将占空比放大10倍，在J-Scoppe上观察。

![[j-scope_open_vel.gif]]
如上图所示，`motor.move()`生成非常漂亮的三相正弦波。所以，只要发送相隔120度的三相正弦波给三相无刷电机，电机就能旋转起来！

## 4.3、性感的马鞍波
![[Pasted image 20250106194124.png]]
![[j-scope_maanbo.gif]]
如上图所示，切换控制算法后，正弦波变成马鞍波。simpleFOC支持两种算法，随意我们切换来做实验。





