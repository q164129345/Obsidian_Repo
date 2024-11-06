
# 导言
---
视频里讲了怎样创建python虚拟环境，怎样激活等。接着，在Qt Creator实践一遍。
![[152526696-1-16.mp4]]

# 一、激活虚拟环境
---
![[Pasted image 20241107003233.png]]
如上图所示，Qt Creator创建Pyside6项目时，可以选择创建一个python虚拟环境。

![[Pasted image 20241107003611.png]]
如上图所示，构建、运行项目时选择虚拟环境，可以将基础工程跑起来。

![[Pasted image 20241107003813.png]]
如上图所示，打开Qt Creator的终端。当然，MacOS的终端也可以。

![[Pasted image 20241107004139.png]]
如上图所示，一步一步找到bin/下面的activate。

![[Pasted image 20241107004359.png]]
如上所示，激活了这个项目的虚拟环境。

![[Pasted image 20241107004502.png]]
如上所示，使用`which python3`指令，查到此时python3解析器的位置在刚才创建的项目里。

![[Pasted image 20241107004811.png]]
如上图所示，在MacOS终端上，输入`which python3`查询解析器的位置，跟Qt项目的不一样。隔离了！！！！

![[Pasted image 20241107005122.png]]
通过`pip3 list`看到，第三方库的内容也不一样。隔离了！！！

# 二、去激活虚拟环境
---
![[Pasted image 20241107010334.png]]
如上图所示，使用`deactivate`指令可以退出虚拟环境（去激活）。




