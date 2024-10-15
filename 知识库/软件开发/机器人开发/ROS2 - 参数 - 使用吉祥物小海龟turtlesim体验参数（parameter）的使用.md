# 导言
---
官方文档：https://fishros.org/doc/ros2/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html

在ROS2（Robot Operating System 2）中，**参数（Parameters）** 是一种用于配置节点行为的机制。参数的引入旨在提高系统的灵活性和可配置性，使得节点的行为可以在运行时动态调整，而无需修改代码或重新编译。

**参数的目的**
- **动态配置节点行为**：参数允许在节点启动时或运行时设置变量，这些变量可以影响节点的运行逻辑。例如，调整算法的阈值、设置话题名称等。
- **提高系统的灵活性**：通过参数，开发者可以为同一节点提供不同的配置，以适应不同的运行环境或需求，而无需更改代码。
- **支持运行时调整**：参数可以在节点运行时被动态更改，节点可以响应这些变化，实时调整其行为。
- **增强可维护性**：将配置与代码分离，便于维护和管理。参数的使用使得配置管理更加系统化、规范化。

**参数的使用场景**
1. **配置算法参数**：例如，在SLAM（同步定位与地图构建）算法中，可以通过参数设置激光扫描的最小和最大范围、地图分辨率等。
2. **设置话题和服务名称**：允许节点根据参数订阅或发布到不同的话题，或提供不同的服务。
3. **启用或禁用功能**：使用布尔类型的参数，可以控制节点的某些功能是否启用，如日志记录、调试模式等。
4. **调整运行参数**：如控制机器人的速度、加速度限制、安全距离等。
<br>
# 一、参数
---
还是启动之前的两个小海龟ROS节点。
```shell
ros2 run turtlesim turtlesim_node
```
```shell
ros2 run turtlesim turtle_teleop_key
```
![[Pasted image 20241014203021.png | 800]]
## 1.1、查看参数列表ros2 param list
如下图所示，通过`ros2 param list`查看参数列表。每个节点都有参数``use_sim_time``；它不是 turtlesim 特有的。
<div class="tip">
<strong>use_sim_time是什么？</strong><br><br>当`use_sim_time`被设置为`true`时，节点会订阅`/clock`话题，并使用从该话题接收到的时间作为当前时间。这意味着节点的时间将由仿真器或时间发布者控制，而不是系统的墙钟时间。
</div>

![[Pasted image 20241014203254.png]]

## 1.2、获取参数的内容ros2 param get
使用ros2 param get 节点名 参数名就可以获取参数的内容。如下所示，获取`/turtlesim`的参数background_r的内容是69。
![[Pasted image 20241014204600.png]]

## 1.3、设置参数ros2 param set
将节点/turtlesim的参数background_r设置为150，接着得到节点的答复"Set parameter successful"。
![[Pasted image 20241014204903.png]]
小乌龟活动空间的背景颜色发生变化了。
![[Pasted image 20241014205045.png]]

<div class="tip">
<strong>提示</strong><br><br>使用 `set` 命令设置参数只会在当前会话中生效，而不是永久生效。然而，你可以保存你的设置，并在下次启动节点时重新加载。
</div>

## 1.4、导出参数的模板文件yaml
在终端输入`ros2 param dump /turtlesim > turtlesim.yaml`之后，当前文件夹就会出现一个文件turtlesim.yaml。
![[Pasted image 20241014205547.png]]
turtlesim.yaml的内容如下图：
![[Pasted image 20241014205733.png]]

## 1.5、导入参数模板文件yaml
在终端重新启动节点turtlesim，背景颜色回到原来的颜色。
![[Pasted image 20241014205926.png | 700]]
接着，通过导入刚才导出来的turtlesim.yaml。最后，背景颜色变化了，在终端上看到很多`successful`的字眼，证明导入参数成功了。
![[Pasted image 20241014210349.png]]


