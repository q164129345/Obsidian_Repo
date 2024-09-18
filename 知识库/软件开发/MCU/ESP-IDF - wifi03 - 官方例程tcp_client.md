## 一、前言
---
[https://blog.csdn.net/qq_45803449/article/details/134255011](https://blog.csdn.net/qq_45803449/article/details/134255011)
跟着这位CSDN博主学习如何运行官方的WIFI例程。
官方文档：[https://docs.espressif.com/projects/esp-idf/zh_CN/v5.1/esp32s3/api-guides/wifi.html#esp32-s3-wi-fi-ap](https://docs.espressif.com/projects/esp-idf/zh_CN/v5.1/esp32s3/api-guides/wifi.html#esp32-s3-wi-fi-ap)
Tcp工作流程
![[Pasted image 20240918202908.png]]


## 二、vscode
---
### 2.1、将例程添加到vscode的工作区
例程在esp仓库的如下位置
![[Pasted image 20240918202927.png]]
添加到vscode.
![[Pasted image 20240918202942.png]]
继续让ChatGPT帮我将代码添加中文注释，tcp_client_v4.c

```c
/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include "sdkconfig.h"
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <errno.h>
#include <netdb.h>            // struct addrinfo
#include <arpa/inet.h>
#include "esp_netif.h"
#include "esp_log.h"
#if defined(CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN)
#include "addr_from_stdin.h"
#endif

#if defined(CONFIG_EXAMPLE_IPV4)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR  // 定义主机IP地址
#elif defined(CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN)
#define HOST_IP_ADDR ""
#endif

#define PORT CONFIG_EXAMPLE_PORT  // 定义端口号

static const char *TAG = "example";  // 日志标签
static const char *payload = "Message from ESP32 ";  // 要发送的消息

// TCP客户端任务函数
void tcp_client(void)
{
    char rx_buffer[128];  // 接收缓冲区
    char host_ip[] = HOST_IP_ADDR;  // 主机IP地址
    int addr_family = 0;  // 地址族
    int ip_protocol = 0;  // IP协议

    while (1) {
#if defined(CONFIG_EXAMPLE_IPV4)
        struct sockaddr_in dest_addr;  // 定义目标地址结构体
        inet_pton(AF_INET, host_ip, &dest_addr.sin_addr);  // 将点分十进制IP地址转换为二进制形式
        dest_addr.sin_family = AF_INET;  // 设置地址族为IPv4
        dest_addr.sin_port = htons(PORT);  // 设置端口号
        addr_family = AF_INET;  // 设置地址族
        ip_protocol = IPPROTO_IP;  // 设置IP协议
#elif defined(CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN)
        struct sockaddr_storage dest_addr = { 0 };  // 定义通用目标地址结构体
        ESP_ERROR_CHECK(get_addr_from_stdin(PORT, SOCK_STREAM, &ip_protocol, &addr_family, &dest_addr));  // 从标准输入获取IP地址
#endif

        int sock =  socket(addr_family, SOCK_STREAM, ip_protocol);  // 创建TCP套接字
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);  // 打印创建套接字失败的错误信息
            break;
        }
        ESP_LOGI(TAG, "Socket created, connecting to %s:%d", host_ip, PORT);  // 打印创建套接字成功信息

        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));  // 连接到目标地址
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);  // 打印连接失败的错误信息
            break;
        }
        ESP_LOGI(TAG, "Successfully connected");  // 打印连接成功信息

        while (1) {
            int err = send(sock, payload, strlen(payload), 0);  // 发送消息
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);  // 打印发送失败的错误信息
                break;
            }

            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);  // 接收消息
            // 接收过程中出错
            if (len < 0) {
                ESP_LOGE(TAG, "recv failed: errno %d", errno);  // 打印接收失败的错误信息
                break;
            }
            // 接收到数据
            else {
                rx_buffer[len] = 0; // 将接收到的数据添加字符串结束符
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);  // 打印接收到的数据长度和来源
                ESP_LOGI(TAG, "%s", rx_buffer);  // 打印接收到的数据
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");  // 打印关闭套接字并重启的信息
            shutdown(sock, 0);  // 关闭套接字的发送和接收
            close(sock);  // 关闭套接字
        }
    }
}
```

接着是tcp_client_main.c

```c
/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "esp_event.h"

extern void tcp_client(void);

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    tcp_client();
}

```

### 2.2、编译、下载
![[Pasted image 20240918203015.png]]
![[Pasted image 20240918203027.png]]
![[Pasted image 20240918203043.png]]
![[Pasted image 20240918203056.png]]
![[Pasted image 20240918203109.png]]
![[Pasted image 20240918203122.png]]
配置好WIFI热点与Tcp Server的IP地址、Port端口号。接着，开始编译工程。
![[Pasted image 20240918203138.png]]
### 2.3、监控、调试
笔记本电脑端（Tcp Server）：
![[Pasted image 20240918203156.png]]
ESP32S3端（Tcp Client）：
![[Pasted image 20240918203302.png]]
## 三、细节补充
---
### 3.1、在代码上修改目标Tcp Server的IP地址与Port端口
![[Pasted image 20240918203321.png]]
如下图所示，实际的效果是一样的。
![[Pasted image 20240918203335.png]]