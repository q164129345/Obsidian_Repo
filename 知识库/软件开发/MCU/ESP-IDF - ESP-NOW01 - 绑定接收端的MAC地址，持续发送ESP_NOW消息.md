# 导言
---
![[Pasted image 20240918195607.png]]
![[Pasted image 20240918195624.png]]

本实验的目的是：
使用ESP-NOW协议，通过WIFI。让一个ESP32S3作为发送端（预先知道对方WIFI MAC地址），每间隔2S发送消息“Hello,ESP NOW”给另外一个ESP32S3接收端。
![[Pasted image 20240918195644.png]]

程序源码：[https://github.com/q164129345/esp32_Learning/tree/main/code_ESP_NOW_Comm](https://github.com/q164129345/esp32_Learning/tree/main/code_ESP_NOW_Comm)

发送端：
![[Pasted image 20240918195714.png]]
接收端：
![[Pasted image 20240918195730.png]]

# 一、Sender（发送端）
---
### 1.1、sender.c
![[Pasted image 20240918195752.png]]

### 1.2、main.c
![[Pasted image 20240918195838.png]]
# 二、Receiver（接收端）
---
### 2.1、receiver.c
![[Pasted image 20240918195907.png]]

### 2.2、main.c
![[Pasted image 20240918195940.png]]

# 三、ESP_NOW关键函数
---![[Pasted image 20240918200022.png]]
### 3.1、nvs_flash_init()
`nvs_flash_init()` 函数的主要作用是初始化 NVS 闪存分区。具体来说：
1. 初始化 NVS：
    - 函数会初始化 NVS 子系统，并准备好 NVS 分区，以便后续的读写操作。
    - 如果 NVS 闪存分区未被格式化，函数会自动格式化该分区。
2. 检查和恢复：
    - 函数会检查 NVS 分区的状态，并在必要时尝试恢复。如果分区损坏或没有初始化，函数会返回一个错误。

**为什么要运行 `nvs_flash_init()`？**
1. 存储 Wi-Fi 配置信息：
    - 在许多 ESP32 应用中，Wi-Fi 配置信息（例如 SSID 和密码）会被存储在 NVS 中。通过初始化 NVS，可以确保这些配置信息能够被正确读取和写入。
2. 数据持久化：
    - 应用程序可能需要在设备重启后保留某些数据，例如用户设置、状态信息、计数器等。通过 NVS，可以实现数据的持久化。
3. ESP-NOW 配置：
    - 在使用 ESP-NOW 时，有时需要保存配置信息到 NVS 中，例如对等设备的 MAC 地址和加密密钥等。

### 3.2、esp_netif_init()
`esp_netif` 是 ESP-IDF 中用于管理网络接口的模块。它提供了一组 API 来配置和控制各种类型的网络接口，包括 Wi-Fi 和以太网。通过 `esp_netif`，可以方便地管理和操作网络连接的生命周期。

**`esp_netif_init()` 的作用？**

`esp_netif_init()` 函数的主要作用是初始化 `esp_netif` 模块。这包括设置必要的内部数据结构和状态，以便后续的网络操作能够正常进行。

**为什么要运行 `esp_netif_init()`？**

1. 初始化网络堆栈：
    - `esp_netif_init()` 初始化网络堆栈，使其能够处理网络接口的创建、配置和操作。
2. 配置网络接口：
    - 在使用 Wi-Fi 或以太网时，需要配置相关的网络接口（如 IP 地址、网关、DNS 等）。`esp_netif_init()` 为这些配置操作打下基础。
3. 事件处理：
    - `esp_netif` 模块与事件系统集成，能够处理网络接口的各种事件（如连接、断开连接、IP 地址分配等）。初始化 `esp_netif` 后，可以注册和处理这些事件。

### 3.3、esp_event_loop_create_default()
**什么是 `esp_event_loop_create_default()`？**

`esp_event_loop_create_default()` 函数创建并初始化一个默认的事件循环，这个循环用来处理系统事件和用户事件。事件循环是一个异步机制，它允许不同的模块和组件通过事件通知和处理机制进行通信和协调。

**为什么需要运行 `esp_event_loop_create_default()`？**

1. 处理系统事件：
    - ESP-IDF 框架中许多系统组件，如 Wi-Fi、蓝牙和电源管理等，都会生成事件。例如，Wi-Fi 连接状态的变化（连接、断开等）会生成事件。
    - `esp_event_loop_create_default()` 函数确保这些系统事件能够被正确地分派和处理。
2. 用户事件处理：
    - 开发者可以定义和发布自定义事件，用于应用程序中的模块之间进行通信和协调。例如，一个传感器模块可以生成事件通知主应用处理新数据。
    - 创建默认事件循环后，可以注册用户定义的事件处理程序。
3. 异步任务协调：
    - 事件循环允许异步任务之间进行协调，而不需要阻塞操作。这样可以提高系统的响应速度和效率。

**在 `sender.c` 和 `receiver.c` 中，调用 `esp_event_loop_create_default()` 的具体作用如下：**

1. 初始化默认事件循环：
    - 创建一个默认的事件循环，使系统和用户事件能够被正确地处理。
2. 支持 Wi-Fi 相关事件处理：
    - 无论是作为发送端还是接收端，ESP32 的 Wi-Fi 模块都会生成各种事件（例如，连接、断开连接、数据发送和接收等）。默认事件循环确保这些事件能够被处理，从而使 ESP-NOW 通信能够正常工作。

### 3.4、esp_wifi_init()、esp_wifi_set_mode()、esp_wifi_start()
它们用于初始化WIFI。