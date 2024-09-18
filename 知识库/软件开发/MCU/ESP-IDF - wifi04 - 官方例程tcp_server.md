## 一、前言
---
[https://blog.csdn.net/qq_45803449/article/details/134255011](https://blog.csdn.net/qq_45803449/article/details/134255011)
跟着这位CSDN博主学习如何运行官方的WIFI例程。
官方文档：[https://docs.espressif.com/projects/esp-idf/zh_CN/v5.1/esp32s3/api-guides/wifi.html#esp32-s3-wi-fi-ap](https://docs.espressif.com/projects/esp-idf/zh_CN/v5.1/esp32s3/api-guides/wifi.html#esp32-s3-wi-fi-ap)
Tcp 工作流程
![[Pasted image 20240918203609.png]]

## 二、vscode
---
### 2.1、添加例程到vscode的工作区
![[Pasted image 20240918205423.png]]
![[Pasted image 20240918205435.png]]

还是请ChatGPT帮我注释一下例程的代码：
```c
/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#define PORT                        CONFIG_EXAMPLE_PORT  // 定义端口号
#define KEEPALIVE_IDLE              CONFIG_EXAMPLE_KEEPALIVE_IDLE  // 保持活动的空闲时间
#define KEEPALIVE_INTERVAL          CONFIG_EXAMPLE_KEEPALIVE_INTERVAL  // 保持活动的间隔时间
#define KEEPALIVE_COUNT             CONFIG_EXAMPLE_KEEPALIVE_COUNT  // 保持活动的计数

static const char *TAG = "example";  // 日志标签

// 数据重新传输函数
static void do_retransmit(const int sock)
{
    int len;
    char rx_buffer[128];  // 接收缓冲区

    do {
        len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);  // 接收数据
        if (len < 0) {
            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);  // 打印接收错误信息
        } else if (len == 0) {
            ESP_LOGW(TAG, "Connection closed");  // 打印连接关闭信息
        } else {
            rx_buffer[len] = 0; // 将接收到的数据添加字符串结束符
            ESP_LOGI(TAG, "Received %d bytes: %s", len, rx_buffer);  // 打印接收到的数据

            // send() 可以返回小于提供的长度的字节数。
            // 通过循环发送来实现鲁棒性
            int to_write = len;
            while (to_write > 0) {
                int written = send(sock, rx_buffer + (len - to_write), to_write, 0);  // 发送数据
                if (written < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);  // 打印发送错误信息
                    // 发送失败，放弃
                    return;
                }
                to_write -= written;  // 更新剩余要发送的字节数
            }
        }
    } while (len > 0);
}

// TCP服务器任务函数
static void tcp_server_task(void *pvParameters)
{
    char addr_str[128];  // 地址字符串
    int addr_family = (int)pvParameters;  // 地址族
    int ip_protocol = 0;  // IP协议
    int keepAlive = 1;  // 保持活动选项
    int keepIdle = KEEPALIVE_IDLE;  // 保持活动的空闲时间
    int keepInterval = KEEPALIVE_INTERVAL;  // 保持活动的间隔时间
    int keepCount = KEEPALIVE_COUNT;  // 保持活动的计数
    struct sockaddr_storage dest_addr;  // 目标地址结构体

#ifdef CONFIG_EXAMPLE_IPV4
    if (addr_family == AF_INET) {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;  // 定义IPv4目标地址
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);  // 设置目标IP地址为任何地址
        dest_addr_ip4->sin_family = AF_INET;  // 设置地址族为IPv4
        dest_addr_ip4->sin_port = htons(PORT);  // 设置端口号
        ip_protocol = IPPROTO_IP;  // 设置IP协议
    }
#endif
#ifdef CONFIG_EXAMPLE_IPV6
    if (addr_family == AF_INET6) {
        struct sockaddr_in6 *dest_addr_ip6 = (struct sockaddr_in6 *)&dest_addr;  // 定义IPv6目标地址
        bzero(&dest_addr_ip6->sin6_addr.un, sizeof(dest_addr_ip6->sin6_addr.un));  // 清零目标地址
        dest_addr_ip6->sin6_family = AF_INET6;  // 设置地址族为IPv6
        dest_addr_ip6->sin6_port = htons(PORT);  // 设置端口号
        ip_protocol = IPPROTO_IPV6;  // 设置IP协议
    }
#endif

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);  // 创建监听套接字
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);  // 打印创建套接字错误信息
        vTaskDelete(NULL);  // 删除任务
        return;
    }
    int opt = 1;  // 设置选项值
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));  // 设置套接字选项
#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
    // 注意，默认情况下，IPv6绑定到两个协议，必须禁用
    // 如果同时使用两个协议（在CI中使用）
    setsockopt(listen_sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));  // 设置IPv6选项
#endif

    ESP_LOGI(TAG, "Socket created");  // 打印创建套接字信息

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));  // 绑定套接字到目标地址
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);  // 打印绑定错误信息
        ESP_LOGE(TAG, "IPPROTO: %d", addr_family);  // 打印协议类型
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT);  // 打印绑定端口信息

    err = listen(listen_sock, 1);  // 监听套接字
    if (err != 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);  // 打印监听错误信息
        goto CLEAN_UP;
    }

    while (1) {
        ESP_LOGI(TAG, "Socket listening");  // 打印套接字监听信息

        struct sockaddr_storage source_addr;  // 源地址结构体（足够大，适用于IPv4或IPv6）
        socklen_t addr_len = sizeof(source_addr);  // 地址长度
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);  // 接受连接
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);  // 打印接受连接错误信息
            break;
        }

        // 设置TCP保持活动选项
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
        // 将IP地址转换为字符串
#ifdef CONFIG_EXAMPLE_IPV4
        if (source_addr.ss_family == PF_INET) {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        }
#endif
#ifdef CONFIG_EXAMPLE_IPV6
        if (source_addr.ss_family == PF_INET6) {
            inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
        }
#endif
        ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);  // 打印接受连接的IP地址

        do_retransmit(sock);  // 调用数据重新传输函数

        shutdown(sock, 0);  // 关闭套接字的发送和接收
        close(sock);  // 关闭套接字
    }

CLEAN_UP:
    close(listen_sock);  // 关闭监听套接字
    vTaskDelete(NULL);  // 删除任务
}

// 应用主函数
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());  // 初始化NVS
    ESP_ERROR_CHECK(esp_netif_init());  // 初始化网络接口
    ESP_ERROR_CHECK(esp_event_loop_create_default());  // 创建默认事件循环

    /* 这个辅助函数配置了Wi-Fi或以太网，具体取决于在menuconfig中的选择。
     * 阅读 examples/protocols/README.md 中的 "建立Wi-Fi或以太网连接" 部分，了解更多关于该函数的信息。
     */
    ESP_ERROR_CHECK(example_connect());

#ifdef CONFIG_EXAMPLE_IPV4
    xTaskCreate(tcp_server_task, "tcp_server", 4096, (void*)AF_INET, 5, NULL);  // 创建TCP服务器任务（IPv4）
#endif
#ifdef CONFIG_EXAMPLE_IPV6
    xTaskCreate(tcp_server_task, "tcp_server", 4096, (void*)AF_INET6, 5, NULL);  // 创建TCP服务器任务（IPv6）
#endif
}

```

### 2.2、编译、下载
![[Pasted image 20240918205508.png]]
![[Pasted image 20240918205525.png]]
![[Pasted image 20240918205548.png]]
![[Pasted image 20240918205559.png]]
![[Pasted image 20240918205611.png]]
![[Pasted image 20240918205623.png]]
设置好WIFI、Tcp Server的配置。接着，编译代码。
![[Pasted image 20240918205642.png]]

### 2.3、监控、调试程序
如下图所示，连接WIFI热点Wallace89成功，被分配的IP地址是192.168.8.12。接着，初始化Tcp Server成功，端口号是3333。
![[Pasted image 20240918205700.png]]
然后，笔记本电脑使用Tcp Client的身份，连接到ESP32的这个Tcp Server上。
![[Pasted image 20240918205717.png]]
ESP32S3：
![[Pasted image 20240918205728.png]]