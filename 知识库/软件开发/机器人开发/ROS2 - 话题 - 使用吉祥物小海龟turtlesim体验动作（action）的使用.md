# 导言
---
官方文档：https://fishros.org/doc/ros2/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html
![[Action-SingleActionClient.gif]]
### **动作与话题、服务的区别**
- **话题（Topic）**：用于发布/订阅模式的异步通信，适合持续的数据流，没有反馈或结果机制。
- **服务（Service）**：基于请求/响应的同步通信，适合短时间、即时的请求，不支持任务取消或进度反馈。
- **动作（Action）**：适用于需要长时间执行的任务，提供了异步通信、实时反馈、任务取消和状态跟踪等功能。

### **动作通信的适用场景**
- **机器人导航**：机器人移动到指定位置，需要实时监控导航进度。
- **机械臂操作**：执行抓取、放置等复杂动作，需要反馈操作进度和结果。
- **路径规划**：计算复杂的路径可能需要较长时间，客户端需要知道进度和最终结果。
<br>
# 一、启动节点
---
启动小乌龟节点：
```shell
ros2 run turtlesim turtlesim_node
```
启动检测键盘按键，控制小乌龟的节点：
```shell
ros2 run turtlesim turtle_teleop_key
```
![[Pasted image 20241015232343.png]]

如下图所示，按下键盘可以控制小乌龟到指定的角度。但是，到底哪个是服务端，哪个是客户端呢？？
![[Pasted image 20241015232920.png| 800]]

# 二、节点信息ros2 node info
---
为了了解到底哪个节点是动作服务端，哪个节点是动作客户端，可以使用`ros2 node info 节点名称`。
首先，使用`ros2 node list`查看节点名称。
![[Pasted image 20241015233519.png | 600]]
<br>
查看节点`/turtlesim`的节点信息，如下图所示，找到Action Servers看到接口`/turtle1/rorate_absolute`，它的类型是`turtlesim/action/RotateAbsolute`。
![[Pasted image 20241015233752.png | 800]]
<br>
接着，查看节点`/teleop_turtle`的节点信息，如下图所示，找到Action Clients看到接口`/turtle1/rorate_absolute`，它的类型是`turtlesim/action/RotateAbsolute`。
![[Pasted image 20241015234310.png | 800]]
<br>
总结一下，所以节点`/turtlesim`是动作服务器，`/turleop_turtle`是动作客户端。

# 三、查看当前的动作列表ros2 action list -t
---
跟其他指令一样，都有list -t的用法。
![[Pasted image 20241015235655.png | 800]]

# 四、查看动作接口被调用的情况ros2 action info 
---
![[Pasted image 20241016000250.png | 800]]

# 五、查看接口类型的细节ros2 interface show 
---
![[Pasted image 20241016000837.png | 800]]

内容如下，翻译一下中文。根据官方文档的介绍，第一个`---`上面的是请求的结构（数据类型和名称）。接着，第二个`---`上面的是反馈的结果，最后的是反馈的过程数据，就是说`remaining`会持续反馈。
```text
# The desired heading in radians（以弧度为单位的所需航向）
float32 theta
---

# The angular displacement in radians to the starting position（以弧度为单位的到起始位置的角位移）
float32 delta
---

# The remaining rotation in radians（以弧度为单位的剩余旋转）
float32 remaining

```

# 六、发送一个动作目标ros2 action send_goal
---
跟话题、服务、参数一样，动作也可以通过命令行直接控制，使用公式：`ros2 action send_goal <action_name> <action_type> <values>`。
一开始，乌龟的头向左边，如下图所示：
![[Pasted image 20241016075118.png | 500]]
<br>
接着，使用如下命令：
```shell
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"
```
可以看到，小乌龟开始转动，直到头往上。从命令行上看也有结果反馈。
![[Pasted image 20241016075507.png]]
<br>
## 在命令行增加 --feedback
通过增加--feedback，可以让动作服务端持续发送运动过程信息，直到动作结束。
```shell
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: -1.57}" --feedback
```
![[Pasted image 20241016080049.png | 800]]

<br>

# 七、总结
---
动作类似于服务，允许您执行长时间运行的任务，提供定期反馈，并可取消。
一个机器人系统可能会使用动作进行导航。一个动作目标可以告诉机器人前往一个位置。当机器人导航到该位置时，它可以沿途发送更新（即反馈），然后在到达目的地后发送最终结果消息。
Turtlesim具有一个动作服务器，动作客户端可以向其发送旋转乌龟的目标。在本教程中，您可以内省该动作``/turtle1/rotate_absolute``，以更好地了解动作是什么以及它们是如何工作的。
