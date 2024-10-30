# 一、前言
---
![[Pasted image 20241030113010.png]]
按照正点原子的教程编写launch.json之后，发现没办法启动gdb调试，如上图所示。

# 二、官方文档
---
官方文档:https://github.com/espressif/vscode-esp-idf-extension/blob/master/docs/DEBUGGING.md

![[Pasted image 20241030113348.png]]

如上图所示，找到**Using the Eclipse CDT GDB Debug Adapter**。

![[Pasted image 20241030113550.png]]
如上所示，点击复制到我们的ESP-IDF工程的launch.json文件。

![[Pasted image 20241030113817.png]]
如上图所示，复制到launch.json。

![[Pasted image 20241030113919.png]]
如上图所示，成功启动GDB调试，可以打断点，观察变量等。
