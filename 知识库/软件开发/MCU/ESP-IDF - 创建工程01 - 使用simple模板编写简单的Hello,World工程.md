## 导言
---
```c
#include <stdio.h>
#include <freertos/FreeRTOSConfig.h>
#include <freertos/FreeRTOS.h>
#include <esp_log.h>
#include <freertos/task.h>

void app_main(void)
{
    const char* const TAG = "main";
    while(1) {
        printf("Hello,World.\\n");
        ESP_LOG_LEVEL(ESP_LOG_INFO,TAG,"Hello,Log.\\n"); // 打印log，等级是ESP_LOG_INFO
        vTaskDelay(1000 / portTICK_PERIOD_MS); // 延时1S
    }
}
    
```
烧录进入，进入监控窗口:
![[Pasted image 20240904200934.png]]
可以看到每个1S时间，打印INFO等级的log。