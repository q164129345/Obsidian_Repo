# 导言
---
在几家公司做过一些机器人项目，通讯协议（消息包）都不一样。更加恐怖的，我在丰疆做了两个机器人项目，分别是清洗机器人与割草机器人。它们的VCU与Liunx算法板之间的电池包通讯协议都不一样。

怪不得ROS2提供了一些标准的消息包，如果大家都按照ros的标准来开发机器人的话，就不会出现之前的电池包通讯协议问题了。以后，做其他机器人项目，打包电池包的通讯协议时，严格按照ros2的`sensor_msgs/msg/BatteryState`来格式来。

# 一、ros2 interface
---
![[Pasted image 20241009075023.png | 800]]
在终端输入`ros2 interface`之后，弹出5个命令选项。
## 1.1、ros2 interface list
![[Pasted image 20241009075312.png | 800]]
![[Pasted image 20241009075404.png| 800]]
![[Pasted image 20241009075429.png | 800]]
指令`ros2 interface list`的回复太长了，截取3张图之后可以大概看到，消息包一共分了三大类：Messages（话题）、Services（服务）、Actions（动作），它们都是节点间通讯的媒介。

## 1.2、ros2 interface packages
---
![[Pasted image 20241009081143.png | 800]]
如上图所示，ROS2官方提供了众多消息包，比如我暂时最关注的是sensor_msgs。越复杂的机器人会有越多的传感器，那么sensor_msgs的消息包就会很多。
## 1.3、ros2 interface package 
比如我想进一步查找sensor_msgs里都有哪些传感器的消息包时，使用`ros2 interface package sensor_msgs`就能快速查找。
![[Pasted image 20241009080735.png | 800]]
## 1.4 ros2 interface prototype
![[Pasted image 20241009081758.png | 800]]
当我们需要查看一个消息包的原型时，使用prototype可以看到消息包里面装了什么东西。

## 1.5、ros2 interface show 
想进一步细化消息包的内容时，可以使用show。
![[Pasted image 20241009082056.png | 800]]
如上图所示，BatteryState的消息包内容很长。例如，voltage有了类型float32，且还有注释了，跟源码很接近了。

## 1.6、消息包的源码在哪里？
![[Pasted image 20241009083200.png | 800]]
如图所示，在include/文件夹。
![[Pasted image 20241009083347.png]]
如上所示，找到`battery_state_struct.hpp`定义了电池包消息的结构体。

![[Pasted image 20241009083718.png]]
上面就是源码。


