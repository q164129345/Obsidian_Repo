## 一、前言
---
[https://blog.csdn.net/qq_45803449/article/details/134255011](https://blog.csdn.net/qq_45803449/article/details/134255011)
跟着这位CSDN博主学习如何运行官方的WIFI例程。
官方文档：[https://docs.espressif.com/projects/esp-idf/zh_CN/v5.1/esp32s3/api-guides/wifi.html#esp32-s3-wi-fi-ap](https://docs.espressif.com/projects/esp-idf/zh_CN/v5.1/esp32s3/api-guides/wifi.html#esp32-s3-wi-fi-ap)
udp工作流程:
![[Pasted image 20240920152436.png]]
## 二、vscode
---
### 2.1、将例程代码添加到工作区
udp_server的文件夹地址如下：
![[Pasted image 20240920152456.png]]
接着，添加到vscode的工作区：
![[Pasted image 20240920152508.png]]
还是找Chat GPT将代码增加中文备注。

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

### 2.2、设置、编译、下载程序
![[Pasted image 20240920152543.png]]
![[Pasted image 20240920152554.png]]
设置udp server的port口。
![[Pasted image 20240920152615.png]]
![[Pasted image 20240920152628.png]]
设置WIFI热点的名字与密码。
![[Pasted image 20240920152654.png]]
![[Pasted image 20240920152709.png]]
编译代码。
![[Pasted image 20240920152725.png]]
### 2.3、监控、调试程序
![[Pasted image 20240920152742.png]]
![[Pasted image 20240920152800.png]]
在笔记本电脑使用udp协议，发送的之前填写好upd server的ip地址与对应的port口。
![[Pasted image 20240920152815.png]]