## 一、前言
---
[https://blog.csdn.net/qq_45803449/article/details/134255011](https://blog.csdn.net/qq_45803449/article/details/134255011)
跟着这位CSDN博主学习如何运行官方的WIFI例程。
官方文档：[https://docs.espressif.com/projects/esp-idf/zh_CN/v5.1/esp32s3/api-guides/wifi.html#esp32-s3-wi-fi-ap](https://docs.espressif.com/projects/esp-idf/zh_CN/v5.1/esp32s3/api-guides/wifi.html#esp32-s3-wi-fi-ap)
![[Pasted image 20240918201925.png]]
## 二、vscode
---
### 2.1、将例程添加进工作区
源码在esp/v5.2.1/esp-idf/example/wifi/getting_started/softAP，如下图所示：
![[Pasted image 20240918201958.png]]
接着，使用vscode将文件夹softAP添加进入工作区。
![[Pasted image 20240918202018.png]]

softap_example_main.c代码如下所示，我使用ChatGPT加入了中文注释。
```c
/*  WiFi softAP Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

/* 这个例子使用的WiFi配置可以通过项目配置菜单设置。

   如果你不想这么做，只需将下面的条目更改为你想要的配置字符串 - 即 #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID  // WiFi SSID配置宏
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD  // WiFi密码配置宏
#define EXAMPLE_ESP_WIFI_CHANNEL   CONFIG_ESP_WIFI_CHANNEL  // WiFi信道配置宏
#define EXAMPLE_MAX_STA_CONN       CONFIG_ESP_MAX_STA_CONN  // 最大连接数配置宏

static const char *TAG = "wifi softAP";  // 日志标签

// WiFi事件处理函数
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    // 如果事件是有站点连接到AP
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;  // 转换事件数据类型
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);  // 打印连接的站点的MAC地址和AID
    // 如果事件是有站点断开与AP的连接
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;  // 转换事件数据类型
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);  // 打印断开连接的站点的MAC地址和AID
    }
}

// 初始化WiFi softAP
void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());  // 初始化底层TCP/IP堆栈
    ESP_ERROR_CHECK(esp_event_loop_create_default());  // 创建默认事件循环
    esp_netif_create_default_wifi_ap();  // 创建默认的WiFi AP接口

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();  // 配置WiFi初始化参数
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));  // 初始化WiFi

    // 注册WiFi事件处理函数
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {  // 配置WiFi AP参数
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,  // 设置SSID
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),  // 设置SSID长度
            .channel = EXAMPLE_ESP_WIFI_CHANNEL,  // 设置信道
            .password = EXAMPLE_ESP_WIFI_PASS,  // 设置密码
            .max_connection = EXAMPLE_MAX_STA_CONN,  // 设置最大连接数
#ifdef CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT
            .authmode = WIFI_AUTH_WPA3_PSK,  // 使用WPA3 PSK认证
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,  // 同时支持H2E和传统SAE
#else /* CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT */
            .authmode = WIFI_AUTH_WPA2_PSK,  // 使用WPA2 PSK认证
#endif
            .pmf_cfg = {
                    .required = true,  // 启用保护管理帧
            },
        },
    };
    // 如果密码为空，设置认证模式为开放
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));  // 设置WiFi模式为AP
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));  // 设置WiFi配置
    ESP_ERROR_CHECK(esp_wifi_start());  // 启动WiFi

    // 打印WiFi配置信息
    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS, EXAMPLE_ESP_WIFI_CHANNEL);
}

// 主函数
void app_main(void)
{
    // 初始化NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());  // 擦除NVS
      ret = nvs_flash_init();  // 重新初始化NVS
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");  // 打印日志信息
    wifi_init_softap();  // 初始化WiFi softAP
}

```

### 2.2、编译、下载
首先，先配置好烧录与调试的端口，我的电脑上是/dev/ttyACM0。然后，选择芯片的型号，我的是esp32_s3。
![[Pasted image 20240918202047.png]]

接着，打开esp-idf终端配置WIFI的设置。
![[Pasted image 20240918202104.png]]
![[Pasted image 20240918202133.png]]
![[Pasted image 20240918202150.png]]
![[Pasted image 20240918202210.png]]
![[Pasted image 20240918202223.png]]
![[Pasted image 20240918202240.png]]

### 2.3、监控程序
点击监控，可以看到顺利开启AP热点esp32_s3_ex。
![[Pasted image 20240918202256.png]]
使用我的另外一台笔记本电脑，连接WIFI热点esp32_s3_ex。
![[Pasted image 20240918202312.png]]
![[Pasted image 20240918202321.png]]





