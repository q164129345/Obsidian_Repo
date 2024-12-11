# 导言
----
参考安富莱教程：《[【专题教程第4期】SEGGER的J-Scope波形上位机软件，HSS模式简单易用，无需额外资源，也不需要写目标板代码](https://www.armbbs.cn/forum.php?mod=viewthread&tid=83097)》
我发现RTT Viewer可以跟J-Scope一起开起来监控，一边检测log，另外一边检测波形，真的很爽。
![[rtt_viewer_scope.gif]]
如上所示，左边RTT Viewer与右边的J-Scope都是在监控同一个全局变量sineValue。

# 一、代码
---
## 1.1、main.h
![[Pasted image 20241210205934.png]]
## 1.2、main.c
![[Pasted image 20241210210045.png]]
如上图所示，全局变量sineValue就是在RTT Viewer打印出来，并且在J-Scope上可视化出来的。<br>

![[Pasted image 20241210210159.png]]
![[Pasted image 20241210210349.png]]
# 二、J-Scope
---
![[Pasted image 20241210210537.png]]
![[Pasted image 20241210210614.png]]


