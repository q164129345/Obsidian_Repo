# 导言
---
![[Pasted image 20241003231204.png | 900]]
如上图所示，输入`ros2 topic`查看详细的参数。

# 一、ros2 topic list
---
![[Pasted image 20241003231833.png]]
如上图所示，输入`ros2 topic list`后，可以查看到当前的主题列表，其中`/command`是节点topic_publisher_02.py发布的。因为这个节点还在运行，所以能看到`/command`主题。

# 二、ros2 topic echo 
---
如果想查看`/command`主题的信息内容，可以使用`ros2 topic echo /command`。我认为这个是调试topic发布者的一个非常非常有用的方法。

![[Pasted image 20241003232336.png]]
如上图所示，主题`/command`的内容是字符串“backup"，这个正式发布者topic_publisher_02.py通过主题command发布出来的消息内容。

# 三、ros2 topic info
---
![[Pasted image 20241003232723.png]]
如上图所示，`ros2 topic info`可以查看到主题command的消息类型std_msgs/msg/String，还有订阅者与发布者的数量。
