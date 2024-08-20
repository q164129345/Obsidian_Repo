# 一、前言
----
[官方文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-nano/finsh-port/an0045-finsh-port)写得很详细，操作过一遍真的很简单。

# 二、CubeMX
---
![[Pasted image 20240820202945.png]]
![[Pasted image 20240820203006.png]]
波特率随便设，RT_Thread里会重新设置的，开启中断接收。

# 三、rtconfig.h
---
将备注解除。
![[Pasted image 20240820203058.png]]
![[Pasted image 20240820203121.png]]

# 四、main.c
---
首先，CubeMX生成的USART1初始化备注掉，因为要留给RT_Thread系统来初始化。
![[Pasted image 20240820203201.png]]

# 五、board.c
---
![[Pasted image 20240820203226.png]]
将官方写的函数uart_init()里的Instance改为USART1，跟前面一致。好了，编译，下载程序。

# 六、MobaXterm
---
使用这个串口工具，可以很好地跟FinSH控制台交互。
![[Pasted image 20240820203311.png]]
