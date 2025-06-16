# 导言
根据[官方文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-nano/an0038-nano-introduction)，按照步骤来就可以，真的很简单。
![[Pasted image 20240820202609.png]]



# 一、代码处理
---
## 1.1、rtconfig.h
![[Pasted image 20241210142542.png]]

## 1.2、main.c
在while(1)循环，记得增加rt_thread_delay()，让系统调度起来，不要卡死的while()，否则console会不接收任何指令。
![[Pasted image 20241210142613.png]]

# 二、细节补充
---
## 2.1、移植完成后，main()函数里的心跳灯，居然跑不起来
不要让CubeMX生成`SystemClock_Config()`就能解决。
![[Pasted image 20250616192019.png]]
![[Pasted image 20250616192343.png]]