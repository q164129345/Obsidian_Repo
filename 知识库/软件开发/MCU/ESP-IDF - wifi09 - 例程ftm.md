# 一、前言
---
- `espnow` 是一种无连接的 Wi-Fi 通信协议。
- `fast_scan` 演示了如何在连接到 AP 时使用快速扫描。
- `ftm` 介绍了如何使用 FTM（精确计时测量）功能来计算设备之间的距离。
- `getting_started` 包含了使用 Wi-Fi STA 模式和 AP 模式的基本示例。
- `iperf` 展示了两个运行该示例的 ESP 的常见性能测量。
- `power_save` 介绍了如何使用 Wi-Fi 的省电模式。
- `roaming` 包含了使用 802.11k 和 802.11v API 的示例。
- `scan` 演示了如何扫描所有可用的 AP。
- `smart_config` 演示了 ESP 如何使用 ESPTOUCH 连接到目标 AP。
- `wifi_eap_fast` 演示了 ESP 如何使用 EAP-FAST 方法连接到具有企业加密的 AP。
- `wifi_easy_connect` 演示了如何使用设备配置协议（DPP）将 ESP 设备配置为入网设备。
- `wpa2_enterprise` 演示了 ESP 如何连接到具有 WPA2 企业加密的 AP。
- `wps` 介绍了如何使用 Wi-Fi 保护设置协议。
今天尝试一下例程ftm。例程的位置如下：
![[Pasted image 20240920154034.png]]
加入到vscode工作区：
![[Pasted image 20240920154047.png]]
设置好烧录的端口，芯片的型号，还有烧录的方式。
![[Pasted image 20240920154113.png]]
![[Pasted image 20240920154122.png]]
![[Pasted image 20240920154133.png]]
![[Pasted image 20240920154142.png]]
接着，编译、烧录代码即可。
![[Pasted image 20240920154159.png]]

# 二、调试程序
---
## 2.1、更换接口（如果是UART口烧录的话，可以忽略）
如下图所示，当前我程序使用USB/JTAG口。烧录完程序之后，需要将USB线插回UART口。因为例程ftm使用了终端交互功能，需要使用串口工具给esp32发送字符串。esp32根据字符串去执行相应的程序。
![[Pasted image 20240920154217.png]]
如下图所示，使用字符串指令，可以请求esp32扫描一边周边的WIFI热点。
![[Pasted image 20240920154226.png]]
## 2.2、串口工具的选择
### 2.2.1、SSCOM
选择正确的串口通道，然后波特率选择115200。如下图所示，按下esp32的复位按钮后，弹出很多初始化的消息。
![[Pasted image 20240920154259.png]]
![[Pasted image 20240920154310.png]]
![[Pasted image 20240920154327.png]]

### 2.2.2、MobaXterm（强烈推荐）
点击session
![[Pasted image 20240920154348.png]]
![[Pasted image 20240920154401.png]]
点击OK，连接之后。按下esp32的reset按钮，让程序复位。窗口就会弹出一些消息。
![[Pasted image 20240920154415.png]]
如下图所示，MobaXterm的显示效果远比sscom要好得多。我估计原因是MobaXterm使用的是Putty的格式，然而SSCOM却不是。
![[Pasted image 20240920154506.png]]

## 2.3、其他字符串指令
### 2.3.1、指令scan
如下图所示，使用scan指令之后可以扫描周边的WIFI热点。
![[Pasted image 20240920154529.png]]
scan指令可以扫描特定的wifi热点。当输入scan之后，会有补全提示。
![[Pasted image 20240920154539.png]]
如下所示，输入指令”scan baijiang”，提示成功搜索到wifi热点baijiang。
![[Pasted image 20240920154552.png]]
### 2.3.2、指令sta
如下图所示，输入字符串”sta baijiang z82693173”后，esp32进入wifi的station模式，并连接wifi热点”baijiang”。
![[Pasted image 20240920154610.png]]
当输入字符串“sta”时，也有指令提示。
![[Pasted image 20240920154621.png]]
### 2.3.3、指令ap