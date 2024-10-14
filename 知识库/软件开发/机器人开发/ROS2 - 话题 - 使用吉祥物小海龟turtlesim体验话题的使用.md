# 导言
---
**话题（Topic）**
- **发布/订阅模型**：话题采用发布者和订阅者的通信模型。一个节点可以发布消息到一个特定的话题上，其他订阅该话题的节点会接收到这些消息。
- **异步通信**：发布者和订阅者之间的通信是异步的，彼此不需要同时存在。发布者无需知道订阅者的存在，反之亦然。
- **多对多关系**：一个话题可以有多个发布者和多个订阅者，这使得消息传递非常灵活。
- **适用场景**：适用于需要持续传输数据的情况，例如传感器数据（激光扫描、图像流）、状态更新等。
**示例**：
- 一个相机节点持续发布图像数据到`/camera/image`话题，多个图像处理节点订阅该话题以处理实时图像。

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
