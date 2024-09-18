## 导言
---
![[Pasted image 20240918200940.png]]
广播通讯时，ESP_NOW接收端可以知道消息的RSSI。接收端可以根据RSSI去筛选掉一些远距离的ESP_NOW消息，进一步实现近距离通讯，近距离匹配等控制方式。
> 接收信号强度指示器（Received Signal Strength Indicator）
![[Pasted image 20240918200959.png]]

还是之前的两块ESP32S3控制板。
接收端log打印：
![[Pasted image 20240918201030.png]]
关键的MAC地址、消息内容、RSSI都顺利得到了。
![[Pasted image 20240918201047.png]]
我试着让发送端远离接收端，RSSI会随着变得更小。

源码地址：[https://github.com/q164129345/esp32_Learning/tree/main/code_ESP_NOW_Comm](https://github.com/q164129345/esp32_Learning/tree/main/code_ESP_NOW_Comm)

## 一、发送端（broadcast_sender.c)
---
![[Pasted image 20240918201110.png]]
大部分代码跟上一章节一样，这里只讲解一下不一样的地方。如上所示，就算是广播方式发送ESP_NOW消息，也要添加MAC地址进去。
![[Pasted image 20240918201132.png]]

不加入MAC地址的话，程序会卡死。。。

## 二、发送端的main.c
---
```c
void app_main(void)
{
    //led_Init(); // 初始化LED
    broadcast_Sender_Main_Init(); // 初始化广播发送端
    //broadcast_Receiver_Main_Init(); // 初始化广播接收端
    while(1) {
        //esp_log_write(ESP_LOG_INFO,"MAIN","%s(%d): esp_now sender\\n",__FUNCTION__, __LINE__);
        //led_Toggle();
        vTaskDelay(300 / portTICK_PERIOD_MS);    
    }
}
```

## 三、接收端（broadcast_receiver.c）
---
![[Pasted image 20240918201158.png]]
![[Pasted image 20240918201211.png]]

## 四、接收端的main.c
---
```c
/**
 * @brief 主函数
 * 
 */
void app_main(void)
{
    led_Init(); // 初始化LED
    //broadcast_Sender_Main_Init(); // 初始化广播发送端
    broadcast_Receiver_Main_Init(); // 初始化广播接收端
    while(1) {
        //esp_log_write(ESP_LOG_INFO,"MAIN","%s(%d): esp_now sender\\n",__FUNCTION__, __LINE__);
        led_Toggle();
        vTaskDelay(300 / portTICK_PERIOD_MS);    
    }
}
```

## 五、关键函数

---

### 5.1、