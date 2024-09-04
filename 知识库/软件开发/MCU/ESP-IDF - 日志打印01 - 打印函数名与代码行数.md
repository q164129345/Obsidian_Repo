## 导言
---
使用`esp_log_write()`可以方便地打印log到串口上。EPS-IDF已经帮我们弄好printf重定向，UART驱动等一系列配置了。

我们只需要包含头文件`esp_log.h`，然后直接调用`esp_log_write()`即可。接着，加入预定宏`__FUNCTION__`与`__LINE__`，打印当前log的函数名与代码行数之后，调试程序非常方便。
![[Pasted image 20240904204023.png]]
效果如下：
![[Pasted image 20240904204034.png]]