# 导言
---
[[ROS2 - 用C++创建发布者节点与订阅者节点]]上一节，用C++语言创建了发布者与订阅者。接着，用python语言创建发布者与订阅者，对比一下两种语言的差异。

![[Pasted image 20241003221153.png]]

如上图所示，先在setup.py里添加如下两行。目的是告诉编译器需要编译节点topic_publisher_02.py与topic_subscirbe_02.py。
```python
"topic_publisher_02 = example_topic_rclpy.topic_publisher_02:main",
"topic_subscribe_02 = example_topic_rclpy.topic_subscribe_02:main"
```

# 一、发布者
---
```python
# /usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class NodePublisher02(Node):
	def __init__(self, name):
	super().__init__(name)
	self.get_logger().info("大家好，我是%s!" % name)
	self.command_publisher_ = self.create_publisher(String, "command", 10)
	self.timer = self.create_timer(0.5, self.timer_callback)

	# 定时器回调函数
	def timer_callback(self):
		msg = String()
		msg.data = 'backup'
		self.command_publisher_.publish(msg)
		self.get_logger().info(f'发布了指令：{msg.data}') # 打印发布的数据

def main(args = None):
	rclpy.init(args = args)
	node = NodePublisher02("topic_publisher_02")
	rclpy.spin(node)
	rclpy.shutdown()
```


# 二、订阅者
---
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

  
class NodeSubscribe02(Node):
	def __init__(self,name):
	super().__init__(name)
	self.get_logger().info("大家好，我是%s!" % name)
	# 创建订阅者
	self.command_subscribe_ = self.create_subscription(String, "command", self.command_callback ,10)

	def command_callback(self, msg):
		speed = 0.0
		if msg.data == 'backup':
		speed = -0.2
		self.get_logger().info(f'收到[{msg.data}]命令，发送速度{speed}')

  
def main(args=None):
	rclpy.init(args=args) # 初始化rclpy
	node = NodeSubscribe02("topic_subscribe_02") # 新建一个节点
	rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
	rclpy.shutdown() # 关闭rclpy
```


# 三、编译、运行节点
---
![[Pasted image 20241003223816.png]]

![[Pasted image 20241003223906.png]]
如上图所示，现在命令行输入`source install/setup.zsh`，接着`ros2 run example_topic_rclpy topic_subscribe_02`运行订阅者节点。

![[Pasted image 20241003224133.png]]
如上图所示，同样输入`source install/setup.zsh`，接着`ros2 run example_topic_rclpy topic_publisher_02`运行发布者节点。
最后，发布者节点与订阅者节点开始了通讯。



