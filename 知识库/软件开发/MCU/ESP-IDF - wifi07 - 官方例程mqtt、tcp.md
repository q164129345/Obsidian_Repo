## 一、前言
---
官方的MQTT例程有如下几个：
![[Pasted image 20240920153024.png]]
- **custom_outbox**：这个示例展示了如何使用自定义的出站消息缓存来实现MQTT客户端。它允许开发者自定义消息的存储和重发策略。
- **ssl**：这个示例展示了如何使用TLS（Transport Layer Security）加密进行安全的MQTT通信。它使用SSL/TLS证书来确保数据传输的安全性。
- **ssl_ds**：这个示例展示了如何使用安全元素（Secure Element）进行MQTT通信的SSL/TLS客户端认证。它利用硬件加密模块来存储和处理私钥和证书。
- **ssl_mutual_auth**：这个示例展示了如何实现双向SSL/TLS认证。客户端和服务器都需要验证对方的证书，以确保双方的身份真实性。
- **ssl_psk**：这个示例展示了如何使用预共享密钥（Pre-Shared Key，PSK）进行SSL/TLS加密的MQTT通信。PSK是一种简化的加密机制，不需要使用证书。
- **tcp**：这个示例展示了如何使用纯TCP（无加密）进行MQTT通信。它是最基础的MQTT实现，不提供任何加密或认证。
- **ws**：这个示例展示了如何通过WebSocket协议进行MQTT通信。WebSocket是一种在浏览器和服务器之间建立长连接的协议，适用于Web应用程序。
- **wss**：这个示例展示了如何通过安全的WebSocket（WebSocket Secure，WSS）协议进行MQTT通信。WSS在WebSocket之上增加了SSL/TLS加密，确保数据传输的安全性。

## 二、例程tcp
---
### 2.1、例程添加到vscode工作区
源码的文件夹位置：
![[Pasted image 20240920153048.png]]
![[Pasted image 20240920153059.png]]
代码让ChatGPT帮忙备注中文：
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

#define PORT CONFIG_EXAMPLE_PORT  // 定义端口号

static const char *TAG = "example";  // 日志标签

// UDP服务器任务函数
static void udp_server_task(void *pvParameters)
{
    char rx_buffer[128];  // 接收缓冲区
    char addr_str[128];  // 地址字符串缓冲区
    int addr_family = (int)pvParameters;  // 地址族
    int ip_protocol = 0;  // IP协议
    struct sockaddr_in6 dest_addr;  // 目标地址结构体

    while (1) {

        // 根据地址族配置目标地址
        if (addr_family == AF_INET) {
            struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;  // 定义IPv4目标地址
            dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);  // 设置目标IP地址为任何地址
            dest_addr_ip4->sin_family = AF_INET;  // 设置地址族为IPv4
            dest_addr_ip4->sin_port = htons(PORT);  // 设置端口号
            ip_protocol = IPPROTO_IP;  // 设置IP协议
        } else if (addr_family == AF_INET6) {
            bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));  // 清零目标地址
            dest_addr.sin6_family = AF_INET6;  // 设置地址族为IPv6
            dest_addr.sin6_port = htons(PORT);  // 设置端口号
            ip_protocol = IPPROTO_IPV6;  // 设置IP协议
        }

        // 创建UDP套接字
        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);  // 打印创建套接字错误信息
            break;
        }
        ESP_LOGI(TAG, "Socket created");  // 打印创建套接字成功信息

#if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
        int enable = 1;
        lwip_setsockopt(sock, IPPROTO_IP, IP_PKTINFO, &enable, sizeof(enable));  // 设置套接字选项
#endif

#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
        if (addr_family == AF_INET6) {
            // 注意，默认情况下，IPv6绑定到两个协议，必须禁用
            // 如果同时使用两个协议（在CI中使用）
            int opt = 1;
            setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));  // 设置重用地址选项
            setsockopt(sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));  // 设置IPv6选项
        }
#endif
        // 设置接收超时时间
        struct timeval timeout;
        timeout.tv_sec = 10;
        timeout.tv_usec = 0;
        setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

        // 绑定套接字到目标地址
        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);  // 打印绑定错误信息
        }
        ESP_LOGI(TAG, "Socket bound, port %d", PORT);  // 打印绑定端口信息

        struct sockaddr_storage source_addr;  // 源地址结构体（足够大，适用于IPv4或IPv6）
        socklen_t socklen = sizeof(source_addr);  // 源地址长度

#if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
        struct iovec iov;
        struct msghdr msg;
        struct cmsghdr *cmsgtmp;
        u8_t cmsg_buf[CMSG_SPACE(sizeof(struct in_pktinfo))];

        iov.iov_base = rx_buffer;
        iov.iov_len = sizeof(rx_buffer);
        msg.msg_control = cmsg_buf;
        msg.msg_controllen = sizeof(cmsg_buf);
        msg.msg_flags = 0;
        msg.msg_iov = &iov;
        msg.msg_iovlen = 1;
        msg.msg_name = (struct sockaddr *)&source_addr;
        msg.msg_namelen = socklen;
#endif

        while (1) {
            ESP_LOGI(TAG, "Waiting for data");  // 打印等待数据日志
#if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
            int len = recvmsg(sock, &msg, 0);  // 接收消息
#else
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);  // 接收数据
#endif
            // 接收过程中出错
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);  // 打印接收错误信息
                break;
            }
            // 接收到数据
            else {
                // 获取发送方的IP地址作为字符串
                if (source_addr.ss_family == PF_INET) {
                    inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);  // 转换IPv4地址为字符串
#if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
                    for ( cmsgtmp = CMSG_FIRSTHDR(&msg); cmsgtmp != NULL; cmsgtmp = CMSG_NXTHDR(&msg, cmsgtmp) ) {
                        if ( cmsgtmp->cmsg_level == IPPROTO_IP && cmsgtmp->cmsg_type == IP_PKTINFO ) {
                            struct in_pktinfo *pktinfo;
                            pktinfo = (struct in_pktinfo*)CMSG_DATA(cmsgtmp);
                            ESP_LOGI(TAG, "dest ip: %s", inet_ntoa(pktinfo->ipi_addr));  // 打印目标IP地址
                        }
                    }
#endif
                } else if (source_addr.ss_family == PF_INET6) {
                    inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);  // 转换IPv6地址为字符串
                }

                rx_buffer[len] = 0; // 将接收到的数据添加字符串结束符
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);  // 打印接收到的数据长度和来源
                ESP_LOGI(TAG, "%s", rx_buffer);  // 打印接收到的数据

                int err = sendto(sock, rx_buffer, len, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));  // 发送回显消息
                if (err < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);  // 打印发送错误信息
                    break;
                }
            }
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

#ifdef CONFIG_EXAMPLE_IPV4
    xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET, 5, NULL);  // 创建UDP服务器任务（IPv4）
#endif
#ifdef CONFIG_EXAMPLE_IPV6
    xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET6, 5, NULL);  // 创建UDP服务器任务（IPv6）
#endif

}

```

### 2.2、配置、编译、烧录代码
![[Pasted image 20240920153207.png]]
![[Pasted image 20240920153217.png]]
![[Pasted image 20240920153226.png]]
值得注意的是，填写Broker URL格式需要按照以下的来：
![[Pasted image 20240920153241.png]]
![[Pasted image 20240920153249.png]]
![[Pasted image 20240920153257.png]]
如下图所示，成功编译代码：
![[Pasted image 20240920153312.png]]

### 2.3、运行，监控程序
![[Pasted image 20240920153346.png]]
笔记本电脑：

1、需要使用mosquitto创建一个MQTT服务器(Broker）。搭建MQTT Broker的教程：[mosquitto | Windows + mosquitto搭建MQTT Broker（本地服务器）与MQTTX客户端联调](https://www.notion.so/mosquitto-Windows-mosquitto-MQTT-Broker-MQTTX-e4443f58697d4a3da40ea2917531dca0?pvs=21)
2、用MQTTX，创建一个MQTT客户端Client，发布ESP32订阅了的主题。
![[Pasted image 20240920153401.png]]
ESP32侧：
![[Pasted image 20240920153410.png]]
## 三、细节补充
---
### 3.1、增加主题发送，周期1S
在函数mqtt_app_start()增加一段代码，实现周期1S的主题发送。
![[Pasted image 20240920153431.png]]
ESP32侧：
![[Pasted image 20240920153440.png]]
笔记本电脑侧：
![[Pasted image 20240920153450.png]]