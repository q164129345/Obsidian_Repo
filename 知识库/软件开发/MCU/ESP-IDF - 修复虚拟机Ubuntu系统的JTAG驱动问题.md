## 导言
---
![[Pasted image 20240904194738.png]]
如上图所示，因为ESP32-S3内置了JATG，当我们使用USB/SLAVE接口时，Windows系统显示的接口应该是`USB JTAG / serial debug unit`
当Windows的驱动程序有问题时，虚拟机Ubuntu也不能正常使用的。

## 解决驱动问题
---
首先，官方也有具体的教程：
[配置 ESP-WROVER-KIT 上的 JTAG 接口 - ESP32 - — ESP-IDF 编程指南 v5.2.1 文档](https://docs.espressif.com/projects/esp-idf/zh_CN/stable/esp32/api-guides/jtag-debugging/configure-ft2232h-jtag.html)
总的来说，就是使用Zadig软件来修复USB驱动问题。
![[Pasted image 20240904194824.png]]