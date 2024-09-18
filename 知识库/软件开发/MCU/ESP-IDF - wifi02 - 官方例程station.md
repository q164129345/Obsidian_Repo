## 一、前言
---
[https://blog.csdn.net/qq_45803449/article/details/134255011](https://blog.csdn.net/qq_45803449/article/details/134255011)
跟着这位CSDN博主学习如何运行官方的WIFI例程。
官方文档：[https://docs.espressif.com/projects/esp-idf/zh_CN/v5.1/esp32s3/api-guides/wifi.html#esp32-s3-wi-fi-ap](https://docs.espressif.com/projects/esp-idf/zh_CN/v5.1/esp32s3/api-guides/wifi.html#esp32-s3-wi-fi-ap)
![[Pasted image 20240918202431.png]]

## 二、vscode
---
### 2.1、将例程添加进工作区
![[Pasted image 20240918202503.png]]
接着，使用vscode将文件夹station添加进入工作区。
![[Pasted image 20240918202519.png]]

```c
/* WiFi station Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

/* 这个例子使用的WiFi配置可以通过项目配置菜单设置

   如果你不想这么做，只需将下面的条目更改为你想要的配置字符串 - 即 #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID  // WiFi SSID配置宏
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD  // WiFi密码配置宏
#define EXAMPLE_ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY  // 最大重试次数配置宏

// 配置WPA3 SAE模式
#if CONFIG_ESP_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif

// 配置WiFi认证模式阈值
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

/* FreeRTOS事件组，用于信号我们何时连接上*/
static EventGroupHandle_t s_wifi_event_group;

/* 事件组允许每个事件有多个位，但我们只关心两个事件：
 * - 我们连接到AP并获取IP
 * - 我们在最大重试次数后未能连接 */
#define WIFI_CONNECTED_BIT BIT0  // 连接成功事件位
#define WIFI_FAIL_BIT      BIT1  // 连接失败事件位

static const char *TAG = "wifi station";  // 日志标签

static int s_retry_num = 0;  // 记录重试次数

// WiFi事件处理函数
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    // 处理WiFi启动事件
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();  // 连接WiFi
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        // 处理WiFi断开事件
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();  // 重新连接WiFi
            s_retry_num++;  // 增加重试次数
            ESP_LOGI(TAG, "retry to connect to the AP");  // 打印重试日志
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);  // 设置失败事件位
        }
        ESP_LOGI(TAG,"connect to the AP fail");  // 打印连接失败日志
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        // 处理获取IP事件
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;  // 转换事件数据类型
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));  // 打印获取的IP地址
        s_retry_num = 0;  // 重置重试次数
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);  // 设置连接成功事件位
    }
}

// 初始化WiFi station
void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();  // 创建事件组

    ESP_ERROR_CHECK(esp_netif_init());  // 初始化底层TCP/IP堆栈

    ESP_ERROR_CHECK(esp_event_loop_create_default());  // 创建默认事件循环
    esp_netif_create_default_wifi_sta();  // 创建默认的WiFi station接口

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();  // 配置WiFi初始化参数
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));  // 初始化WiFi

    esp_event_handler_instance_t instance_any_id;  // 定义任意事件实例
    esp_event_handler_instance_t instance_got_ip;  // 定义获取IP事件实例
    // 注册WiFi事件处理函数
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    // 注册IP事件处理函数
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {  // 配置WiFi station参数
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,  // 设置SSID
            .password = EXAMPLE_ESP_WIFI_PASS,  // 设置密码
            /* 如果密码符合WPA2标准（密码长度>=8），则将认证模式阈值重置为WPA2。
             * 如果你想将设备连接到不推荐使用的WEP/WPA网络，请将阈值设置为
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK，并设置符合WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK标准的密码长度和格式。
             */
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,  // 设置认证模式阈值
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,  // 设置SAE模式
            .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,  // 设置H2E标识符
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );  // 设置WiFi模式为station
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );  // 设置WiFi配置
    ESP_ERROR_CHECK(esp_wifi_start() );  // 启动WiFi

    ESP_LOGI(TAG, "wifi_init_sta finished.");  // 打印初始化完成日志

    /* 等待连接成功或连接失败事件。事件由上面的event_handler()设置。 */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() 返回调用前的事件位，因此我们可以测试哪个事件发生了。 */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);  // 打印连接成功信息
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);  // 打印连接失败信息
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");  // 打印意外事件日志
    }
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

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");  // 打印日志信息
    wifi_init_sta();  // 初始化WiFi station
}

```

### 2.2、编译、下载
配置好下载端口，芯片型号，下载的方式。
![[Pasted image 20240918202554.png]]
![[Pasted image 20240918202604.png]]![[Pasted image 20240918202617.png]]![[Pasted image 20240918202634.png]]![[Pasted image 20240918202650.png]]
### 2.3、监控程序
![[Pasted image 20240918202709.png]]
接着，查看WIFI热点的信息。
![[Pasted image 20240918202721.png]]
![[Pasted image 20240918202735.png]]
接着，尝试关掉WIFI热点。esp32会重新几次，最后提示连接WIFI热点失败。
![[Pasted image 20240918202751.png]]
