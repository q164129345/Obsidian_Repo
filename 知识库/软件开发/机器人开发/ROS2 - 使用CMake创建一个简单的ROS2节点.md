# 导言
---
在ROS2中，colcon 和 CMake 是两种不同的工具，分别用于不同的目的。
#### Colcon：
- 用途：colcon 是一个构建工具，专门用于构建 ROS 2 工作空间。它可以处理多个包的构建、测试和安装。
- 功能：colcon 可以自动处理包之间的依赖关系，确保按正确的顺序构建包。它还支持并行构建，提高构建速度。
- 使用场景：当你有一个包含多个 ROS 2 包的工作空间时，使用 colcon 是最方便的选择。
#### CMake：
- 用途：CMake 是一个跨平台的构建系统生成器。它用于生成本地构建系统（如 Makefile 或 Visual Studio 项目）。
- 功能：CMake 负责配置和生成构建系统，但不处理包之间的依赖关系。
- 使用场景：CMake 通常用于单个包或项目的构建配置。每个 ROS 2 包都需要一个 CMakeLists.txt 文件来定义其构建规则。

# 一、创建一个CMake工程
---
如下图所示，一个简单的CMake工程包含如下：
- build文件夹
- CMakeLists.txt
- main.cpp
---
![[Pasted image 20241023003534.png]]

## 2.1、main.cpp
---
```cpp
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)

{
	// 调用rclcpp的初始化函数
	rclcpp::init(argc, argv);	
	// 调用rclcpp的循环运行我们创建的first_node节点
	rclcpp::spin(std::make_shared<rclcpp::Node>("first_node"));
	rclcpp::shutdown();
	return 0;
}
```

如下图所示，`#include "rclcpp/rclcpp.hpp"`会出现红色波浪线。
![[Pasted image 20241023004106.png]]

鼠标指向它，然后点击**快速修复**。
![[Pasted image 20241023004244.png]]

选择，**编辑“includePath“设置**。
![[Pasted image 20241023004334.png]]

![[Pasted image 20241023004512.png]]

如下图所示，在includePath段添加`/opt/ros/humble/include/**`。这个目录到底有什么？去探一个究竟。
![[Pasted image 20241023004657.png]]

如下图所示，`/opt/ros/humble/include/`这里有一个rclcpp文件夹。
![[Pasted image 20241023004921.png]]

再往里面找，找到rclcpp.hpp了。
![[Pasted image 20241023005112.png]]

如下图所示，红色波浪线消失了。说白了，就是设置一下inlcude目录而已。
![[Pasted image 20241023005228.png]]
<div class="tip">
<strong>提示</strong><br><br>include 路径是包括源文件中随附的头文件(如 `#include "myHeaderFile.h"`)的文件夹。指定 IntelliSense 引擎在搜索包含的头文件时要使用的列表路径。对这些路径进行的搜索不是递归搜索。指定 `**` 可指示递归搜索。例如，${workspaceFolder}/** 将搜索所有子目录，而 ${workspaceFolder} 则不会。如果在安装了 Visual Studio 的 Windows 上，或者在 `compilerPath` 设置中指定了编译器，则无需在此列表中列出系统 include 路径。
</div>

## 2.2、CMakeLists.txt
```cmake
# 这行代码指定了 CMake 的最低版本要求。这里要求使用至少 3.22 版本的 CMake。
# 原因：确保使用的 CMake 版本支持所有需要的功能和语法。
cmake_minimum_required(VERSION 3.22)

# 定义了项目的名称，这里是 basic_cpp
# 原因：项目名称用于标识和组织构建过程中的文件和目标
project(basic_cpp)

# 查找并加载 rclcpp 包，这是 ROS 2 的 C++ 客户端库。
# 原因：rclcpp 提供了创建 ROS 2 节点所需的功能和类。REQUIRED 表示如果找不到这个包，CMake 将会报错并停止。
find_package(rclcpp REQUIRED)

# 定义一个可执行文件 basic_cpp，其源文件是 main.cpp。
# 原因：告诉 CMake 需要编译 main.cpp 并生成一个名为 basic_cpp 的可执行程序。
add_executable(basic_cpp main.cpp)

# 将 rclcpp 库链接到 first_node 可执行文件
# 原因：确保在编译 first_node 时，链接 rclcpp 库，以便使用其提供的功能
target_link_libraries(basic_cpp rclcpp::rclcpp)
```

如下图所示：
1、首先，进入build文件夹
2、cmake ..
3、make
![[Pasted image 20241023011545.png]]

编译成功，可执行文件basic_cpp出来了。
![[Pasted image 20241023011714.png]]

如下图所示，通过ros2命令查询到，有ros2节点在运行了，节点的名字：first_node。
![[Pasted image 20241023011950.png]]

![[Pasted image 20241023012121.png]]

## 2.2.1、find_package的原理
以下摘自鱼香ROS2教程:https://fishros.com/d2lros2/#/humble/chapt2/basic/4.CMake%E4%BE%9D%E8%B5%96%E6%9F%A5%E6%89%BE%E6%B5%81%E7%A8%8B
![[Pasted image 20241023012327.png]]

总的来说，CMake的find_package依赖系统环境变量，需要确保系统环境变量有相应的库路径才行。

