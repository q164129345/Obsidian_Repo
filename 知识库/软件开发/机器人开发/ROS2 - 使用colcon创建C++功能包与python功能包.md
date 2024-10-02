# 导言
---

# 一、创建工作区
---
![[Pasted image 20241002153334.png]]
如上图所示，工作区trainning_01其实就是一个文件夹。当前，工作区trainning_01里只有一个src/。

# 二、创建C++功能包与ros2节点
---
![[Pasted image 20241002153801.png]]
```bash
cd training_01/src/
ros2 pkg create training_cpp --build-type ament_cmake --dependencies rclcpp
```
通过上述命令，创建training_cpp功能包。

![[Pasted image 20241002154406.png]]
如上图所示，成功创建traning_cpp功能包。

![[Pasted image 20241002154924.png]]
如上图所示，在文件夹traning_cpp/src/文件夹下创建node_01.cpp。
```cpp
#include "rclcpp/rclcpp.hpp"
int main(int argc, char **argv)
{
    /* 初始化rclcpp  */
    rclcpp::init(argc, argv);
    /*产生一个node_01的节点*/
    auto node = std::make_shared<rclcpp::Node>("node_01");
    // 打印一句自我介绍
    RCLCPP_INFO(node->get_logger(), "node_01节点已经启动.");
    /* 运行节点，并检测退出信号 Ctrl+C*/
    rclcpp::spin(node);
    /* 停止运行 */
    rclcpp::shutdown();
    return 0;
}
```

![[Pasted image 20241002155742.png]]
接着，在CMakeLists.txt里添加如上图代码，告诉编译器编译node_01.cpp代码。
```cmake
add_executable(node_01 src/node_01.cpp)
ament_target_dependencies(node_01 rclcpp)

install(TARGETS
	node_01
	DESTINATION lib/${PROJECT_NAME}
)

```


![[Pasted image 20241002160207.png]]

使用colcon工具编译node_01.cpp代码。
```bash
colcon build
```

接着，source一下install/文件夹的setup.zsh。
```bash
source install/setup.zsh
```

最后，使用ros2 run运行指定功能包的某个节点。
```bash
ros2 run traning_cpp node_01
```


# 三、创建python功能包与ros2节点
---
```shell
cd training_01/src/
ros2 pkg create traning_py --build-type ament_python --dependencies rclpy
```
通过上述指令，创建traning_py功能包。

![[Pasted image 20241002161241.png]]
如上图所示，创建后traning_py功能包后，src/文件夹里有两个功能包了。

![[Pasted image 20241002161646.png]]
然后，在traning_py/traning_py/文件夹下创建node_02.py。
并输入代码：
```python
import rclpy
from rclpy.node import Node

def main(args=None):
    """
    ros2运行该节点的入口函数
    编写ROS2节点的一般步骤
    1. 导入库文件
    2. 初始化客户端库
    3. 新建节点对象
    4. spin循环节点
    5. 关闭客户端库
    """
    rclpy.init(args=args) # 初始化rclpy
    node = Node("node_02")  # 新建一个节点
    node.get_logger().info("大家好，我是node_02.")
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy
```

![[Pasted image 20241002162550.png]]
接着，在setup.py里，添加"node_02 = traning_py.node_02:main"。告诉编译器需要编译node_02.py节点。

![[Pasted image 20241002161943.png]]
使用colcon工具，重新编译工作区的代码。如上图所示，traning_py与traning_cpp的节点都编译成功。
```shell
colcon build
```

![[Pasted image 20241002162655.png]]
如上图所示，顺利执行node_02节点。
