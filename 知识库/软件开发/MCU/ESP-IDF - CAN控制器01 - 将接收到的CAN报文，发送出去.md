## 导言
---
![[Pasted image 20240904205537.png]]
![[Pasted image 20240904205555.png]]

在汽车与自动化、机器人领域，CAN总线必须掌握。非常高兴地看到ESP32终于开始具备CAN接口。
![[Pasted image 20240904205615.png]]
![[Pasted image 20240904205641.png]]
CAN收发器很便宜，我直接买了两个回来。
本次代码的目的是学会配置CAN控制器，然后发送与接收CAN报文。

效果如下：
![[Pasted image 20240904205704.png]]
## 一、sdkconfig
---
![[Pasted image 20240904205748.png]]
必须勾选Place TWAI ISR function into RAM
将中断程序放入IRAM中可以提高效率的原因主要是因为IRAM是直接连接到CPU的内存，它不受主存储器（如SPI Flash）的缓存策略影响。这意味着：
1. **缓存未命中减少**：当ISR位于IRAM中时，CPU可以直接访问这些指令，不需要通过缓存，这减少了缓存未命中的可能性，从而减少了访问延迟。
2. **缓存禁用下的运行**：特定的操作，如写SPI Flash时，需要禁用缓存以保持数据一致性。在这种情况下，如果ISR不在IRAM中，那么任何访问非IRAM存储的指令都会导致处理器停滞。而将ISR放入IRAM可以确保即使在这种情况下也能继续执行，不受影响。
因此，使用IRAM可以确保中断处理的响应更快，系统的实时性能得到提升。

## 二、相关函数
---
### 1. **`TWAI_GENERAL_CONFIG_DEFAULT()`**
生成默认的TWAI配置，包括指定的TX和RX引脚以及TWAI的工作模式。
```c
twai_general_config_t TWAI_GENERAL_CONFIG_DEFAULT(gpio_num_t tx_io, gpio_num_t rx_io, twai_mode_t mode);
```
- **参数**:
    - `tx_io`：TX引脚编号。
    - `rx_io`：RX引脚编号。
    - `mode`：TWAI模式，如正常、监听等。

如上图所示，生成的默认配置如下：

```c
#define TWAI_GENERAL_CONFIG_DEFAULT_V2(controller_num, tx_io_num, rx_io_num, op_mode) {
    .controller_id = controller_num,             // 控制器编号
    .mode = op_mode,                             // 工作模式（普通模式）
    .tx_io = tx_io_num,                          // TX引脚编号
    .rx_io = rx_io_num,                          // RX引脚编号
    .clkout_io = TWAI_IO_UNUSED,                 // 未使用的时钟输出引脚
    .bus_off_io = TWAI_IO_UNUSED,                // 未使用的总线关闭引脚
    .tx_queue_len = 5,                           // 发送队列长度
    .rx_queue_len = 5,                           // 接收队列长度
    .alerts_enabled = TWAI_ALERT_NONE,           // 不启用的警报
    .clkout_divider = 0,                         // 时钟输出分频器
    .intr_flags = ESP_INTR_FLAG_LEVEL1           // 中断标志(中断优先级比较低!!!）
}

```

### 2. **`TWAI_FILTER_CONFIG_ACCEPT_ALL()`**

生成一个过滤器配置，接受所有消息。

```c
twai_filter_config_t TWAI_FILTER_CONFIG_ACCEPT_ALL(void);
```

- **参数**:
    - 无参数。

### 3. **`TWAI_TIMING_CONFIG_500KBITS()`**

提供500 Kbps通信速率的定时配置。

```c
twai_timing_config_t TWAI_TIMING_CONFIG_500KBITS(void);
```

- **参数**:
    - 无参数。

### 4. **`twai_driver_install()`**

安装TWAI驱动程序。

```c
esp_err_t twai_driver_install(const twai_general_config_t *g_config, const twai_timing_config_t *t_config, const twai_filter_config_t *f_config);
```

- **参数**:
    - `g_config`：总线配置。
    - `t_config`：定时配置。
    - `f_config`：过滤器配置。

### 5. **`twai_start()`**

启动TWAI模块。

```c
esp_err_t twai_start(void);
```

- **参数**:
    - 无参数。

### 6. **`twai_receive()`**

从TWAI总线接收消息。

```c
esp_err_t twai_receive(twai_message_t *message, TickType_t ticks_to_wait);
```

- **参数**:
    - `message`：存储接收到的消息。
    - `ticks_to_wait`：等待消息的最长时间。

### 7. **`twai_transmit()`**

向TWAI总线发送消息。

```c
esp_err_t twai_transmit(const twai_message_t *message, TickType_t ticks_to_wait);
```

- **参数**:
    - `message`：要发送的消息。
    - `ticks_to_wait`：发送操作的超时时间。

## 三、can_com.c
---
![[Pasted image 20240904205833.png]]

![[Pasted image 20240904205846.png]]


## 四、main.c
---
![[Pasted image 20240904205900.png]]
## 五、细节补充
---
### 5.1、优化CAN控制器的配置
![[Pasted image 20240904205930.png]]
在实际项目上，增加消息队列与提高CAN中断优先级非常重要。

## 5.2、CAN滤波器设置之后，不能正常接收CAN报文，后续看看怎样解。。。。。