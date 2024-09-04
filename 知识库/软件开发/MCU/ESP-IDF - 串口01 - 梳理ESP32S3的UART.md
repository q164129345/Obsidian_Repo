## 导言
---
首先，ESP32S3的串口数量不算多，只有三个串口。一般情况下不建议使用UART0，它被默认用于打印调试Log。
![[Pasted image 20240904204119.png]]
摘自《esp32-s3_technical_reference_manual_cn》
## 一、UART FIFO
---
![[Pasted image 20240904204142.png]]

简单来说，UART0接收到的数据先进入Rx_FIFO，此时软件可能忙着干别的事情。等有空闲时，软件通过APB总线从Rx_FIFO拿数据，也可以借助GDMA帮它拿。
发送数据同理，软件通过APB总线，将需要发送的数据放入Tx_FIFO里，UART0检测到Tx_FIFO有数据时，会将数据发送出去。软件可以委托GDMA将数据搬运到Tx_FIFO，这样软件就有更多的时间干别的事情了。
所以，要编写高效的UART程序，理解FIFO的内存分配与GDMA的应用非常关键。
![[Pasted image 20240904204207.png]]

三个UART共用1024 * 8bit的内容空间，而且首地址是固定的。怎样理解？
- 当我们需要用到三个串口时，那么每一个串口的Tx_FIFO与Rx_FIFO的大小只能设置128bytes，否则会影响别的串口功能。
- 当我们只使用UART0、UART1时，那么UART1的Tx_FIFO与Rx_FIFO的大小可以设置256bytes，将UART2的RAM空间也用上。。