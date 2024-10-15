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













