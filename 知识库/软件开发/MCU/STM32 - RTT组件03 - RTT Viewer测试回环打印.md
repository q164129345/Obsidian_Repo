# 导言
---
![[RTT_Input.gif]]从上gif图看到，RTT Viewer不仅可以像串口一样打印log，也可以像串口一样，接收字符。
开发板 + JLink：
![[Pasted image 20241209161818.png | 800]]
![[Pasted image 20241209191423.png | 800]]
本次实验，使用了`SEGGER_RTT_GetKey()`与`SEGGER_RTT_HasKey()`。
项目源码:https://github.com/q164129345/MCU_Develop/tree/main/jlink_rtt_viewer_input

# 一、代码
---
## 1.1、main.c
![[Pasted image 20241209192305.png]]
![[Pasted image 20241209192409.png]]
![[Pasted image 20241209192513.png]]
从上面的gif图看到，当输入了“Hello,World”后，将每隔100ms打印一个字符到RTT Viewer上，说明SEGGER_RTT_GetKey()每执行一次，从缓存里获取一个字符。

# 二、细节补充
---
## 2.1、末尾自动加入'\n'字符
**RTT Viewer的输入框会自动在结尾增加多一个'\n'字符。** 字符串“Hello,World”只有11个字符。但是，RTT Viewer输入了12个字符。RTT Viewer的输入框优雅地帮我们加入‘\n’字符，帮我们隔开每一组命令。
![[Pasted image 20241209193351.png]]


![[Pasted image 20241209194909.png | 800]]
如上所示，RTT Viewer的input->End of Line...设置Unix format(LF)，意思是在输入的最后会加入字符‘\n'。

![[Pasted image 20241209194651.png | 900]]
如上所示，修改一下代码，用if语法判断一下字符'\n'试试。
![[Pasted image 20241209194502.png | 800]]
编译，下载代码。在终端输入字符串“Hello,World”后，RTT Viewer的打印如上所示，说明字符串“Hello,World“的最后会加入字符‘\n'。
