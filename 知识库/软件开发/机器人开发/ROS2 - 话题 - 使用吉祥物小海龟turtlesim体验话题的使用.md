# 导言
---
在终端输入：`ros2 run turtlesim turtlesim_node`，启动小海龟模拟器。
![[Pasted image 20241010075826.png | 800]]

# 一、查看turtlesim_node的话题
---
## 1.1、CLI
![[Pasted image 20241010080629.png | 800]]
终端输入`ros2 topic list -t`，查看到当前turtlesim_node节点的话题与类型。
- /turtle1/cmd_vel 用于控制乌龟运动
- /turtle1/color 乌龟变颜色
- /turtle1/pose 发布当前乌龟的位置与姿态信息

## 1.2、rqt
可视化工具rqt也能查看话题信息。
![[Pasted image 20241010081255.png | 800]]
![[Pasted image 20241010081349.png | 800]]
如图所示，找到话题列表(topic)与话题对应的类型(type)。

# 二、使用话题控制小海龟运动起来
---
CLI与可视化工具rqt都可以让小海龟动起来。CLI使用命令行比较麻烦（可以去问Chatgpt），这里只演示可视化工具rqt。
如下图所示，找到Message Publisher.
![[Pasted image 20241010081744.png]]
<br>
如下图所示，设置linear.x = 0.5，设置angular.z = 0.2，并勾选主题之后，小海龟开始动起来了。
![[Pasted image 20241010082014.png]]
<br>
此时，可以通过CLI查看主题的发送频率与内容，在终端输入`ros2 topic echo /turtle1/cmd_vel`。
终端显示linear.x = 0.5，linear.z = 0.2，跟rqt设置对应上了。
![[Pasted image 20241010082409.png | 800]]
查看频率在终端输入`ros2 topic hz /turtle1/cmd_vel`。如下图所示，测量出来的频率是1.0。
![[Pasted image 20241010082951.png | 800]]
