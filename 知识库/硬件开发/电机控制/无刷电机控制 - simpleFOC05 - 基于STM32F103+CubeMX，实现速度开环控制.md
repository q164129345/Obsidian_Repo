# 导言
---
![[Pasted image 20250106201758.png]]
如上所示，构建速度开环控制的框架，需要继续完成如下模块：
1. Inverse Park Transform(反帕克变换)
2. SVPWM Generator（SVPWM算法，或者克拉克逆变换）

效果如下，电机终于开始旋转起来了：
![[20250102-204654.mp4]]

RTT Viewer的打印，虽然是速度开环控制，也能比较接近期望的转速4rad/s：
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

## 4.4、BLDCMotor motor(7)发生了什么？
![[Pasted image 20250106211707.png]]
如上图所示，实例化一个BLDCMotor对象，最简单的方式是填入电机的极对数，实验用的云台无刷电机是7极多。接下来，稍微深入研究一下代码`BLDCMotor motor(7)`的意义。

![[Pasted image 20250107191354.png]]
BLDCMotor类的父类是FOCMotor类，BLDCMotor实例化对象motor时会先调用FOCMotor类的构造函数FOCMotor()，接着再调用自己的构造函数BLDCMotor()。

![[Pasted image 20250106212503.png | 800]]
如上图所示，BLDCMotor的父类是FOCMotor。

![[Pasted image 20250106213045.png]]
如上图所示，FOCMotor类的的构造函数初始化一堆软件上的参数。

![[Pasted image 20250107192031.png]]
如上图所示，BLDCMotor类的构造函数的目的是初始化电机的硬件参数，极对数必须填写。

![[Pasted image 20250107194657.png]]
为什么在BLDCMotor()的构造函数末尾要增加:FOCMotor()？
1. 如果BLDCMotor的父类FOCMotor类只有一个默认的无参数的构造函数时，末尾的:FOCMotor()可以删除掉，不影响。
2. 如果父类FOCMotor有两个或者以上的构造函数（包含带参数的构造函数），必须显式调用FOCMotor(...)来调用指定的构造函数，否则C++编译器会尝试调用无参数的构造函数FOCMotor()。如果没有这个无参数构造函数FOCMotor()的话，编译会失败。
综上所述，从simpleFOC的源码看到，FOCMotor类只有一个默认的无参数的构造函数。所以，BLDCMotor类的构造函数BLDCMotor()结尾的:FOCMotor()可以有，可以无。只能说simpleFOC的作者有一个好习惯！


