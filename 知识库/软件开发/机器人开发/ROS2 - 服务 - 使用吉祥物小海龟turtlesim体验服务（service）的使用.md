# 导言
---
![[Service-SingleServiceClient.gif]]
![[Service-MultipleServiceClient.gif]]

**服务（Service）**
- **请求/响应模型**：服务采用客户端和服务器的通信模型。客户端发送请求，服务器处理后返回响应。
- **同步通信**：客户端在发送请求后，会等待服务器的响应。这意味着通信是同步的，客户端需要等待操作完成。
- **一对一关系**：一次服务调用涉及一个客户端和一个服务器。
- **适用场景**：适用于需要即时响应的操作，例如参数查询、一次性命令执行（启动、停止某个功能）等。
**示例**：
- 一个机器人需要获取当前的位置坐标，客户端发送请求到位置服务，服务器处理后返回位置信息。
<br>

话题（Topic）与服务（Service）的区别： 
**通信模式**：
- **话题**：发布/订阅，异步，多对多。
- **服务**：请求/响应，同步，一对一。
**使用场景**：
- **话题**：连续的数据流，需要持续更新的信息。
- **服务**：一次性的请求，需要立即处理和响应的操作。
**依赖关系**：
- **话题**：发布者和订阅者彼此独立，不要求同时在线。
- **服务**：客户端和服务器需要同时在线，客户端需要等待服务器的响应。

# 一、启动
---
官方文档：https://fishros.org/doc/ros2/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html

官方文档使用CLI的方式讲解了话题的使用，我决定在这个备忘录使用rqt做一遍。

在终端运行：
```shell
ros2 run turtlesim turtlesim_node
```
在另一个终端运行：
```shell
ros2 run turtlesim turtle_teleop_key
```
第三个终端运行：
```shell
rqt
```
分别启动以上三个ROS2进程，如下图所示，将弹出两个GUI窗口。
![[Pasted image 20241014160252.png | 1000]]
<br>
# 二、服务
---
## 2.1、查看当前的服务 ros2 service list -t
在终端输入之后，显出当前所有ros2进程的话题，如下图所示：
```shell
ros2 service list -t
```
![[Pasted image 20241014161342.png |600]]
使用强大的可视化工具rqt也可以查看当前所有的服务，如下图所示：
![[Pasted image 20241014161553.png | 600]]
![[Pasted image 20241014161720.png | 600]]
比如服务`/clear`，它的话题类型是`std_srv/srv/Empty`。
![[Pasted image 20241014162035.png | 600]]
<br>
## 2.2、话题的参数（请求、回复）
如下图所示，话题`/spawm`有4个请求参数：分别是x、y、theta、name。
![[Pasted image 20241014162404.png | 600]]
然而，话题`/clear`没有请求参数，如下图所示。
![[Pasted image 20241014162724.png | 600]]
总结，服务的请求不一定需要参数。实际使用时，需要参看接口。
<br>
## 2.3、服务的调用
服务`/clear`的作用是清除小乌龟的行走轨迹。如图所示，小乌龟移动之后会流下轨迹。
![[Pasted image 20241014163039.png]]
接着，尝试使用rqt调用/clear服务看看效果吧。
![[Pasted image 20241014163159.png | 900]]
如下图所示，点击call按钮之后，小乌龟的轨迹消失了，在Response对话框里弹出服务的答复。
![[Pasted image 20241014163237.png | 900]]
接着，我们使用服务`/spawn`创造一只新乌龟出来看看。如下所示出现了第二只乌龟，在服务的回复上看到它的名字是turtle2。
![[Pasted image 20241014163506.png | 900]]





