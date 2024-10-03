# 导言
---
上一章节[[ROS2 - 使用colcon创建C++功能包与python功能包]]学会了创建C++节点与python节点。接着，使用RO2的新模块std_msgs来创建发布者节点与订阅者节点。

![[Pasted image 20241003113515.png]]
如上图所示，节点topic_publisher_01与节点topic_subscribe_01都在CMakeLists.txt里添加rclcpp、std_msgs模块。

![[Pasted image 20241003114332.png]]
接着，package.xml文件也要添加模块std_msgs。

# 一、节点topic_publisher_01
---
```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class TopicPublisher01 : public rclcpp::Node
{
public:
	// 构造函数,有一个参数为节点名称
	TopicPublisher01(std::string name) : Node(name)
	{
		RCLCPP_INFO(this->get_logger(), "大家好，我是%s.\n", name.c_str());
		// 创建发布者
		command_publisher_ = this->create_publisher<std_msgs::msg::String>("command", 10);
		// 创建定时器
		timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&TopicPublisher01::timer_callback, this));
	}

private:
	void timer_callback(void) {
	// 定义消息
	std_msgs::msg::String message;
	message.data = "forward\n";
	// 打印log
	RCLCPP_INFO(this->get_logger(), "Publishing:'%s'", message.data.c_str());
	// 发布消息	
	command_publisher_->publish(message);
	}

	// 声明话题发布者
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_publisher_;
	// 声明定时器
	rclcpp::TimerBase::SharedPtr timer_;
};

  

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);	
	/*创建对应节点的共享指针对象*/
	auto node = std::make_shared<TopicPublisher01>("topic_publisher_01");
	/* 运行节点，并检测退出信号*/
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
```





# 二、节点topic_subscribe_01
```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class TopicSubscribe01 : public rclcpp::Node
{
public:
	// 构造函数,有一个参数为节点名称
	TopicSubscribe01(std::string name) : Node(name)
	{
		RCLCPP_INFO(this->get_logger(), "大家好，我是%s.", name.c_str());
		// 创建一个订阅者订阅消息	
		command_Subscribe_ = this->create_subscription<std_msgs::msg::String>("command", 10, std::bind(&TopicSubscribe01::command_callback, this, std::placeholders::_1));
	}

private:

	// 声明一个订阅者
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_Subscribe_;
	// 创建回调函数
	void command_callback(const std_msgs::msg::String::SharedPtr msg) {
		double speed = 0.0f;
		if (msg->data != "forward") {
			speed = 0.2f;
		}
		RCLCPP_INFO(this->get_logger(), "收到[%s]指令,发送速度%f", msg->data.c_str(), speed);
	}
};

  
int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);	
	/*创建对应节点的共享指针对象*/
	auto node = std::make_shared<TopicSubscribe01>("topic_subscribe_01");
	/* 运行节点，并检测退出信号*/
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
```

# 三、运行节点
---
![[Pasted image 20241003114551.png]]
如上图所示，使用命令colcon build编译功能包里的节点代码。

![[Pasted image 20241003114732.png]]
如上图所示，运行了订阅者节点。

![[Pasted image 20241003114846.png]]
如上图所示，运行了发布者节点之后，发布者与订阅者开始了通讯。

![[Pasted image 20241003115507.png]]
使用ros2的强大工具rqt，使用Node Graph插件可以查看节点之间的通讯关系。如上图所示，节点topic_publisher_01发布主题command。节点topic_subscribe_01订阅了主题command。
