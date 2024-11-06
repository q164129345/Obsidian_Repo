# 导言
---
![[152526716-1-208.mp4]]

# 一、Qt Creator配置虚拟环境
---
![[Pasted image 20241107011012.png]]
在左侧找到项目，构建设置是配置python3解析器的位置。



![[Pasted image 20241107011111.png]]
此时，有两种选择：
1. 在MacOS系统上的python3
2. 在Qt项目里的python3(虚拟环境里)，刚才安装了paho-mqtt第三方库

![[Pasted image 20241107011331.png]]
居然还可以创建一个新的python3虚拟环境。

![[Pasted image 20241107011804.png]]
在Qt Creator里编译项目时，可以选择用系统里的python还是项目里python虚拟环境。