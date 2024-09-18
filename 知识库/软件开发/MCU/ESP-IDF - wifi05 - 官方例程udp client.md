## 一、前言
---
[https://blog.csdn.net/qq_45803449/article/details/134255011](https://blog.csdn.net/qq_45803449/article/details/134255011)
跟着这位CSDN博主学习如何运行官方的WIFI例程。
官方文档：[https://docs.espressif.com/projects/esp-idf/zh_CN/v5.1/esp32s3/api-guides/wifi.html#esp32-s3-wi-fi-ap](https://docs.espressif.com/projects/esp-idf/zh_CN/v5.1/esp32s3/api-guides/wifi.html#esp32-s3-wi-fi-ap)
udp工作流程
![[Pasted image 20240918205834.png]]
- 应用方面
![[Pasted image 20240918205908.png]]



## 二、vscode
---
### 2.1、将例程代码添加至vscode的工作区
![[Pasted image 20240918205926.png]]
![[Pasted image 20240918205946.png]]
继续让ChatGPT帮我注释一下代码：
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
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#ifdef CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN
#include "addr_from_stdin.h"
#endif

#if defined(CONFIG_EXAMPLE_IPV4)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR  // 定义主机IP地址（IPv4）
#elif defined(CONFIG_EXAMPLE_IPV6)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR  // 定义主机IP地址（IPv6）
#else
#define HOST_IP_ADDR ""
#endif

#define PORT CONFIG_EXAMPLE_PORT  // 定义端口号

static const char *TAG = "example";  // 日志标签
static const char *payload = "Message from ESP32 ";  // 要发送的消息

// UDP客户端任务函数
static void udp_client_task(void *pvParameters)
{
    char rx_buffer[128];  // 接收缓冲区
    char host_ip[] = HOST_IP_ADDR;  // 主机IP地址
    int addr_family = 0;  // 地址族
    int ip_protocol = 0;  // IP协议

    while (1) {

#if defined(CONFIG_EXAMPLE_IPV4)
        struct sockaddr_in dest_addr;  // 定义目标地址结构体（IPv4）
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);  // 将点分十进制IP地址转换为二进制形式
        dest_addr.sin_family = AF_INET;  // 设置地址族为IPv4
        dest_addr.sin_port = htons(PORT);  // 设置端口号
        addr_family = AF_INET;  // 设置地址族
        ip_protocol = IPPROTO_IP;  // 设置IP协议
#elif defined(CONFIG_EXAMPLE_IPV6)
        struct sockaddr_in6 dest_addr = { 0 };  // 定义目标地址结构体（IPv6）
        inet6_aton(HOST_IP_ADDR, &dest_addr.sin6_addr);  // 将字符串形式的IPv6地址转换为二进制形式
        dest_addr.sin6_family = AF_INET6;  // 设置地址族为IPv6
        dest_addr.sin6_port = htons(PORT);  // 设置端口号
        dest_addr.sin6_scope_id = esp_netif_get_netif_impl_index(EXAMPLE_INTERFACE);  // 获取网络接口索引
        addr_family = AF_INET6;  // 设置地址族
        ip_protocol = IPPROTO_IPV6;  // 设置IP协议
#elif defined(CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN)
        struct sockaddr_storage dest_addr = { 0 };  // 定义通用目标地址结构体
        ESP_ERROR_CHECK(get_addr_from_stdin(PORT, SOCK_DGRAM, &ip_protocol, &addr_family, &dest_addr));  // 从标准输入获取IP地址
#endif

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);  // 创建UDP套接字
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);  // 打印创建套接字错误信息
            break;
        }

        // 设置接收超时时间
        struct timeval timeout;
        timeout.tv_sec = 10;
        timeout.tv_usec = 0;
        setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

        ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);  // 打印创建套接字成功信息

        while (1) {

            int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));  // 发送消息
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);  // 打印发送错误信息
                break;
            }
            ESP_LOGI(TAG, "Message sent");  // 打印消息发送成功信息

            struct sockaddr_storage source_addr;  // 源地址结构体（足够大，适用于IPv4或IPv6）
            socklen_t socklen = sizeof(source_addr);  // 源地址长度
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);  // 接收消息

            // 接收过程中出错
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);  // 打印接收错误信息
                break;
            }
            // 接收到数据
            else {
                rx_buffer[len] = 0;  // 将接收到的数据添加字符串结束符
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);  // 打印接收到的数据长度和来源
                ESP_LOGI(TAG, "%s", rx_buffer);  // 打印接收到的数据
                if (strncmp(rx_buffer, "OK: ", 4) == 0) {  // 判断是否接收到预期消息
                    ESP_LOGI(TAG, "Received expected message, reconnecting");  // 打印接收到预期消息信息
                    break;
                }
            }

            vTaskDelay(2000 / portTICK_PERIOD_MS);  // 延时2秒
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");  // 打印关闭套接字并重启的信息
            shutdown(sock, 0);  // 关闭套接字的发送和接收
            close(sock);  // 关闭套接字
        }
    }
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
    ESP_ERROR_CHECK(example_connect());  // 连接Wi-Fi或以太网

    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);  // 创建UDP客户端任务
}

```

### 2.2、设置、编译、下载程序
![[Pasted image 20240918210029.png]]
![[Pasted image 20240918210040.png]]
![[Pasted image 20240918210053.png]]
![[Pasted image 20240918210108.png]]
![[Pasted image 20240918210123.png]]
![[Pasted image 20240918210136.png]]
![[Pasted image 20240918210148.png]]

### 2.3、监控、调试程序
调试程序发现，当udp_client发送消息到udp_server后，超过一段时间（比如5S），如果udp_server不答复udp_client的话，udp_client将关闭udp socket，重新创建socket，重新用新的端口号再一次发送消息给udp_server。为什么？
<aside> 💡 UDP协议本质上是无连接的，客户端和服务器之间没有持久的连接状态。每次发送和接收数据包都是独立的操作。

</aside>
![[Pasted image 20240918210206.png]]
从udp server这边看到，当esp32发出来的消息没有被答复，超时之后，esp32将重新创建udp连接（端口号发生改变），重新发送消息。实际上，并没有硬性规定重新发送消息时，一定要重新创建udp连接，改变端口号的。程序可以修改为，一直发送udp消息，不在乎udp server的答复。
![[Pasted image 20240918210222.png]]

实际应用时，当udp server收到消息时，能知道对方的ip地址与端口号，此时回复一下消息即可。如下图所示，这个例程能正常接收udp消息，且接收到udp消息时，也能获取消息是谁发送的（ip地址与port端口）。
![[Pasted image 20240918210239.png]]
### 三、细节补充
---
### 3.1、udp client的超时设置
在代码上可以设置超时时间。
![[Pasted image 20240918210258.png]]