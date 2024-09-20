## 一、前言
---
esp32官方提供的wifi例程有如下：
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

通过以上的例程，可以快速掌握esp32的wifi开发。当前的scan例程的目的是扫描周边的wifi热点，并打印一些相关的信息，比如RSSI等。
![[Pasted image 20240920153544.png]]
这个例程的位置：
![[Pasted image 20240920153557.png]]
接着，将代码加入vscode工作区：
![[Pasted image 20240920153618.png]]
## 二、设置、编译、烧录代码
---
![[Pasted image 20240920153637.png]]
![[Pasted image 20240920153650.png]]
![[Pasted image 20240920153659.png]]
改变一下扫描的列表最大值。
![[Pasted image 20240920153713.png]]
![[Pasted image 20240920153722.png]]
![[Pasted image 20240920153733.png]]

## 三、调试代码
---
如下图所示，进入监控之后，可以看到扫描出来的wifi热点。
![[Pasted image 20240920153748.png]]
## 四、代码注释
---
让ChatGPT帮我注释一下代码:

```c
/* Scan Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
    本例子展示了如何扫描可用的接入点（AP）。
*/
#include <string.h>
#include "freertos/FreeRTOS.h"         // FreeRTOS头文件
#include "freertos/event_groups.h"     // FreeRTOS事件组头文件
#include "esp_wifi.h"                  // ESP WiFi头文件
#include "esp_log.h"                   // ESP 日志头文件
#include "esp_event.h"                 // ESP 事件头文件
#include "nvs_flash.h"                 // NVS 闪存头文件

#define DEFAULT_SCAN_LIST_SIZE CONFIG_EXAMPLE_SCAN_LIST_SIZE  // 定义默认的扫描列表大小

static const char *TAG = "scan";       // 定义日志标签

// 打印身份验证模式
static void print_auth_mode(int authmode)
{
    switch (authmode) {
    case WIFI_AUTH_OPEN:
        ESP_LOGI(TAG, "Authmode \\tWIFI_AUTH_OPEN");           // 开放模式
        break;
    case WIFI_AUTH_OWE:
        ESP_LOGI(TAG, "Authmode \\tWIFI_AUTH_OWE");            // OWE模式
        break;
    case WIFI_AUTH_WEP:
        ESP_LOGI(TAG, "Authmode \\tWIFI_AUTH_WEP");            // WEP模式
        break;
    case WIFI_AUTH_WPA_PSK:
        ESP_LOGI(TAG, "Authmode \\tWIFI_AUTH_WPA_PSK");        // WPA_PSK模式
        break;
    case WIFI_AUTH_WPA2_PSK:
        ESP_LOGI(TAG, "Authmode \\tWIFI_AUTH_WPA2_PSK");       // WPA2_PSK模式
        break;
    case WIFI_AUTH_WPA_WPA2_PSK:
        ESP_LOGI(TAG, "Authmode \\tWIFI_AUTH_WPA_WPA2_PSK");   // WPA_WPA2_PSK模式
        break;
    case WIFI_AUTH_ENTERPRISE:
        ESP_LOGI(TAG, "Authmode \\tWIFI_AUTH_ENTERPRISE");     // 企业模式
        break;
    case WIFI_AUTH_WPA3_PSK:
        ESP_LOGI(TAG, "Authmode \\tWIFI_AUTH_WPA3_PSK");       // WPA3_PSK模式
        break;
    case WIFI_AUTH_WPA2_WPA3_PSK:
        ESP_LOGI(TAG, "Authmode \\tWIFI_AUTH_WPA2_WPA3_PSK");  // WPA2_WPA3_PSK模式
        break;
    case WIFI_AUTH_WPA3_ENT_192:
        ESP_LOGI(TAG, "Authmode \\tWIFI_AUTH_WPA3_ENT_192");   // WPA3_ENT_192模式
        break;
    case WIFI_AUTH_WPA3_EXT_PSK:
        ESP_LOGI(TAG, "Authmode \\tWIFI_AUTH_WPA3_EXT_PSK");   // WPA3_EXT_PSK模式
        break;
    case WIFI_AUTH_WPA3_EXT_PSK_MIXED_MODE:
        ESP_LOGI(TAG, "Authmode \\tWIFI_AUTH_WPA3_EXT_PSK_MIXED_MODE"); // WPA3_EXT_PSK混合模式
        break;
    default:
        ESP_LOGI(TAG, "Authmode \\tWIFI_AUTH_UNKNOWN");        // 未知模式
        break;
    }
}

// 打印加密类型
static void print_cipher_type(int pairwise_cipher, int group_cipher)
{
    switch (pairwise_cipher) {
    case WIFI_CIPHER_TYPE_NONE:
        ESP_LOGI(TAG, "Pairwise Cipher \\tWIFI_CIPHER_TYPE_NONE");   // 无加密
        break;
    case WIFI_CIPHER_TYPE_WEP40:
        ESP_LOGI(TAG, "Pairwise Cipher \\tWIFI_CIPHER_TYPE_WEP40");  // WEP40加密
        break;
    case WIFI_CIPHER_TYPE_WEP104:
        ESP_LOGI(TAG, "Pairwise Cipher \\tWIFI_CIPHER_TYPE_WEP104"); // WEP104加密
        break;
    case WIFI_CIPHER_TYPE_TKIP:
        ESP_LOGI(TAG, "Pairwise Cipher \\tWIFI_CIPHER_TYPE_TKIP");   // TKIP加密
        break;
    case WIFI_CIPHER_TYPE_CCMP:
        ESP_LOGI(TAG, "Pairwise Cipher \\tWIFI_CIPHER_TYPE_CCMP");   // CCMP加密
        break;
    case WIFI_CIPHER_TYPE_TKIP_CCMP:
        ESP_LOGI(TAG, "Pairwise Cipher \\tWIFI_CIPHER_TYPE_TKIP_CCMP"); // TKIP和CCMP混合加密
        break;
    case WIFI_CIPHER_TYPE_AES_CMAC128:
        ESP_LOGI(TAG, "Pairwise Cipher \\tWIFI_CIPHER_TYPE_AES_CMAC128"); // AES CMAC128加密
        break;
    case WIFI_CIPHER_TYPE_SMS4:
        ESP_LOGI(TAG, "Pairwise Cipher \\tWIFI_CIPHER_TYPE_SMS4"); // SMS4加密
        break;
    case WIFI_CIPHER_TYPE_GCMP:
        ESP_LOGI(TAG, "Pairwise Cipher \\tWIFI_CIPHER_TYPE_GCMP"); // GCMP加密
        break;
    case WIFI_CIPHER_TYPE_GCMP256:
        ESP_LOGI(TAG, "Pairwise Cipher \\tWIFI_CIPHER_TYPE_GCMP256"); // GCMP256加密
        break;
    default:
        ESP_LOGI(TAG, "Pairwise Cipher \\tWIFI_CIPHER_TYPE_UNKNOWN"); // 未知加密类型
        break;
    }

    switch (group_cipher) {
    case WIFI_CIPHER_TYPE_NONE:
        ESP_LOGI(TAG, "Group Cipher \\tWIFI_CIPHER_TYPE_NONE");    // 无加密
        break;
    case WIFI_CIPHER_TYPE_WEP40:
        ESP_LOGI(TAG, "Group Cipher \\tWIFI_CIPHER_TYPE_WEP40");   // WEP40加密
        break;
    case WIFI_CIPHER_TYPE_WEP104:
        ESP_LOGI(TAG, "Group Cipher \\tWIFI_CIPHER_TYPE_WEP104");  // WEP104加密
        break;
    case WIFI_CIPHER_TYPE_TKIP:
        ESP_LOGI(TAG, "Group Cipher \\tWIFI_CIPHER_TYPE_TKIP");    // TKIP加密
        break;
    case WIFI_CIPHER_TYPE_CCMP:
        ESP_LOGI(TAG, "Group Cipher \\tWIFI_CIPHER_TYPE_CCMP");    // CCMP加密
        break;
    case WIFI_CIPHER_TYPE_TKIP_CCMP:
        ESP_LOGI(TAG, "Group Cipher \\tWIFI_CIPHER_TYPE_TKIP_CCMP"); // TKIP和CCMP混合加密
        break;
    case WIFI_CIPHER_TYPE_SMS4:
        ESP_LOGI(TAG, "Group Cipher \\tWIFI_CIPHER_TYPE_SMS4");    // SMS4加密
        break;
    case WIFI_CIPHER_TYPE_GCMP:
        ESP_LOGI(TAG, "Group Cipher \\tWIFI_CIPHER_TYPE_GCMP");    // GCMP加密
        break;
    case WIFI_CIPHER_TYPE_GCMP256:
        ESP_LOGI(TAG, "Group Cipher \\tWIFI_CIPHER_TYPE_GCMP256"); // GCMP256加密
        break;
    default:
        ESP_LOGI(TAG, "Group Cipher \\tWIFI_CIPHER_TYPE_UNKNOWN"); // 未知加密类型
        break;
    }
}

/* 初始化Wi-Fi为sta模式并设置扫描方法 */
static void wifi_scan(void)
{
    ESP_ERROR_CHECK(esp_netif_init());                         // 初始化网络接口
    ESP_ERROR_CHECK(esp_event_loop_create_default());          // 创建默认事件循环
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta(); // 创建默认的STA网络接口实例
    assert(sta_netif);                                         // 断言sta_netif初始化成功

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();       // 初始化Wi-Fi配置
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));                      // 初始化Wi-Fi

    uint16_t number = DEFAULT_SCAN_LIST_SIZE;                  // 定义扫描列表大小
    wifi_ap_record_t ap_info[DEFAULT_SCAN_LIST_SIZE];          // 定义AP信息数组
    uint16_t ap_count = 0;                                     // 定义AP计数
    memset(ap_info, 0, sizeof(ap_info));                       // 初始化AP信息数组

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));         // 设置Wi-Fi模式为STA
    ESP_ERROR_CHECK(esp_wifi_start());                         // 启动Wi-Fi
    esp_wifi_scan_start(NULL, true);                           // 开始扫描
    ESP_LOGI(TAG, "Max AP number ap_info can hold = %u", number); // 打印最大AP数量
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, ap_info)); // 获取AP记录
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));      // 获取AP数量
    ESP_LOGI(TAG, "Total APs scanned = %u, actual AP number ap_info holds = %u", ap_count, number); // 打印扫描到的总AP数量和实际存储的AP数量
    for (int i = 0; i < number; i++) {
        ESP_LOGI(TAG, "SSID \\t\\t%s", ap_info[i].ssid);         // 打印SSID
        ESP_LOGI(TAG, "RSSI \\t\\t%d", ap_info[i].rssi);         // 打印RSSI
        print_auth_mode(ap_info[i].authmode);                  // 打印身份验证模式
        if (ap_info[i].authmode != WIFI_AUTH_WEP) {
            print_cipher_type(ap_info[i].pairwise_cipher, ap_info[i].group_cipher); // 打印加密类型
        }
        ESP_LOGI(TAG, "Channel \\t\\t%d", ap_info[i].primary);   // 打印信道
    }

}

void app_main(void)
{
    // 初始化NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());                    // 擦除NVS
        ret = nvs_flash_init();                                // 重新初始化NVS
    }
    ESP_ERROR_CHECK( ret );                                    // 检查NVS初始化结果

    wifi_scan();                                               // 调用Wi-Fi扫描函数
}
```

## 五、细节补充
---
### 5.1、改代码，改为每5S扫描一次
改动wifi_scan()函数，将esp_wifi_scan_start以下的代码用while(1)包含起来，然后用vTaskDelay来延时。
![[Pasted image 20240920153815.png]]![[Pasted image 20240920153830.png]]