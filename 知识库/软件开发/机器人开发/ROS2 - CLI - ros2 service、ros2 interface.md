# 导言
---
`ros2 service`是一个很方便的工具，用于在命令行中与服务节点进行交互，有助于开发和调试ROS2的服务节点程序。

通过使用 `ros2 service` 命令，你可以：
- **调试服务：** 快速验证服务是否正常工作。
- **测试功能：** 在不编写额外代码的情况下测试服务的功能。
- **了解系统：** 查看当前有哪些服务在运行，以及它们的接口类型。

接下来，看看怎样使用`ros2 service`与`ros2 interface`去验证一个服务节点。


# 一、开启一个简单的服务节点
---
在命令行里执行如下命令，执行ros官方提供的一个服务节点。
```shell
ros2 run examples_rclpy_minimal_service service
```

# 二、查看service node的服务与该服务的类型：ros2 service list -t
---
![[Pasted image 20241005092508.png]]
如上图所示，`ros2 service list -t`可以查看到服务节点提供的`服务`与`类型`。



# 三、查看service node的服务的接口：ros2 interface show
---
![[Pasted image 20241005093645.png]]
如上图所示，通过`ros2 interface show` + 服务的类型（通过`ros2 service list -t`找到），可以查看到服务的类型的接口的详细内容了。

![[Pasted image 20241005094708.png]]
通过接口的内容，我们了解到输入a与b将得到sum。

# 四、调用服务的类型的接口 ros2 service call
---
![[Pasted image 20241005095842.png]]
如上图所示，输入命令：`ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 10,b: 21}"`，想服务节点请求服务request，并得到答复response。









