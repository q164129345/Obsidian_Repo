## 导言
---
![[Pasted image 20240904202617.png]]
如上图说是，ESP32模组里包含一个外部flash。分区表用于管理这片外部Flash，方便开发者管理一些数据。
![[Pasted image 20240904202638.png]]
![[Pasted image 20240904202657.png]]

如上图所示，ESP32-S3-WROOM-2模组通过Octal SPI接口连接了一个外部flash。ESP32-S3最大支持32M的外部FLASH。

<aside> 💡 Octal SPI（Octal Serial Peripheral Interface）是一种串行通信协议，专为高速数据传输设计，通常用于存储器（如闪存）和微控制器之间的通信。它是标准SPI协议的一种扩展，允许更高的数据传输速率。

</aside>

## 一、配置分区表
---
![[Pasted image 20240904202716.png]]
接着，通过Ctrl + Shift + P开启Open Partition Table Editor UI（打开分区表编辑器），如下图所示：
![[Pasted image 20240904202729.png]]
根据正点原子的教程添加分区表即可。
![[Pasted image 20240904202741.png]]
正点原子的ESP32模组的外部flash尺寸是16M，上图所示如何分配16M的外部Flash。