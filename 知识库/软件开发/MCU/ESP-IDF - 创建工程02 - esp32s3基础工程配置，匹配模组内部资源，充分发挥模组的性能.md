## 导言
---
最近购买一块正点原子的ESP32开发板，这块开发板基于ATK-MW32S模组开发（相当于官方的模组ESP32-S3-WROOM-1），如下图所示：
![[Pasted image 20240904202022.png]]
在乐鑫官网能找到它。
![[Pasted image 20240904202042.png]]
从规格书了解到，ESP32-S3-WROOM-1能进一步细分。正点原子使用后缀N16R8的这一款模组。
从型号对比看来，N16代表lash 16M，R8代表PSRAM 8MB。
![[Pasted image 20240904202109.png]]


## 一、根据模组的型号，配置基础工程
---
使用ESP-IDF的simple_project模板创建的工程是一个通用的工程，这个工程并不知道我们实际使用哪一个模组开发项目。所以，需要进一步设置工程的sdkconfig。如下图所示：
![[Pasted image 20240904202154.png]]

### 1.1、Boot ROM Behavior
![[Pasted image 20240904202211.png]]
![[Pasted image 20240904202228.png]]

### 1.2、ESP PSRAM
![[Pasted image 20240904202248.png]]
![[Pasted image 20240904202302.png]]
![[Pasted image 20240904202315.png]]

### 1.3、ESP System Settings
![[Pasted image 20240904202332.png]]

### 1.4、FreeRTOS Kernel频率
![[Pasted image 20240904202348.png]]

### 1.5、分区
![[Pasted image 20240904202403.png]]
接着，通过Ctrl + Shift + P开启Open Partition Table Editor UI（打开分区表编辑器），如下图所示：
![[Pasted image 20240904202426.png]]
根据正点原子的教程添加分区表即可。
![[Pasted image 20240904202440.png]]
### 1.6、TWAI（需要使用CAN总线时）
![[Pasted image 20240904202455.png]]
将ISR中断回调函数放到IRAM内存里，可以提高CAN通讯接收消息的效率。