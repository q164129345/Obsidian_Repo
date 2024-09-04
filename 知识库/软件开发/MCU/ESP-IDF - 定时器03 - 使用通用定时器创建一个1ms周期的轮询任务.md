## 导言
---
![[Pasted image 20240904205005.png]]
通用定时器的玩法丰富很多。
效果如下：
![[Pasted image 20240904205021.png]]
源码地址：[https://github.com/q164129345/esp32_Learning/tree/main/code_SysTimer](https://github.com/q164129345/esp32_Learning/tree/main/code_SysTimer)
![[Pasted image 20240904205032.png]]
## 一、相关函数

---
### 1. **`gptimer_new_timer()`**
用于创建一个新的定时器实例。需要提供定时器的配置参数，并将返回一个定时器句柄。
```c
esp_err_t gptimer_new_timer(const gptimer_config_t *config, gptimer_handle_t *ret_timer);
```
- **参数**:
    - `config`：定时器的配置，包括时钟源、计数方向、分辨率等。
    - `ret_timer`：返回的定时器句柄。

### 2. **`gptimer_del_timer()`**
删除已创建的定时器，释放相关资源。
```c
esp_err_t gptimer_del_timer(gptimer_handle_t timer);
```
- **参数**:
    - `timer`：定时器句柄。
### 3. **`gptimer_set_raw_count()`**
设置定时器的当前计数值。
```c
esp_err_t gptimer_set_raw_count(gptimer_handle_t timer, uint64_t value);
```

- **参数**:
    - `timer`：定时器句柄。
    - `value`：要设置的计数值。

### 4. **`gptimer_get_raw_count()`**
获取当前的定时器计数值。
```c
esp_err_t gptimer_get_raw_count(gptimer_handle_t timer, uint64_t *value);
```

- **参数**:
    - `timer`：定时器句柄。
    - `value`：返回的当前计数值。

### 5. **`gptimer_set_alarm_action()`**
配置定时器达到某一计数值时的行为，例如触发报警。
```c
esp_err_t gptimer_set_alarm_action(gptimer_handle_t timer, const gptimer_alarm_config_t *config);
```

- **参数**:
    - `timer`：定时器句柄。
    - `config`：包含报警配置的结构体。

### 6. **`gptimer_start()`**
使定时器开始计数。
```c
esp_err_t gptimer_start(gptimer_handle_t timer);
```
- **参数**:
    - `timer`：定时器句柄。
### 7. **`gptimer_stop()`**
停止定时器计数。
```c
esp_err_t gptimer_stop(gptimer_handle_t timer);
```
- **参数**:
    - `timer`：定时器句柄。
### 8. `gptimer_register_event_callbacks()`
这个函数用于为 GPTimer 注册事件回调函数。事件回调允许您定义在特定定时器事件（如达到报警值时）发生时应执行的操作。
```c
esp_err_t gptimer_register_event_callbacks(gptimer_handle_t timer, const gptimer_event_callbacks_t *cbs, void *user_data);
```
- **参数**:
    - `timer`: 定时器的句柄，此句柄是通过 `gptimer_new_timer()` 创建的。
    - `cbs`: 指向 `gptimer_event_callbacks_t` 结构体的指针，该结构体包含了一个或多个回调函数。目前，此结构体主要包含对报警事件的回调，即 `on_alarm`，它是当定时器达到预设的报警计数值时调用的函数。
    - `user_data`: 用户定义的数据，这些数据将被传递给回调函数。这可以用来传递外部变量、对象或状态而无需使用全局变量。
- **返回值**:
    - `ESP_OK`: 成功注册回调。
    - `ESP_ERR_INVALID_ARG`: 传入的参数无效。
    - `ESP_ERR_INVALID_STATE`: 定时器不在正确的状态下，例如尚未初始化。
    - `ESP_FAIL`: 其他错误。

### 9. `gptimer_enable()`
此函数用于启用定时器。启用操作使定时器从初始化状态过渡到使能状态，但并不开始计数，启动计数需要调用 `gptimer_start()`。
```c
esp_err_t gptimer_enable(gptimer_handle_t timer);
```
- **参数**:
    - `timer`: 定时器的句柄。
- **返回值**:
    - `ESP_OK`: 定时器成功启用。
    - `ESP_ERR_INVALID_ARG`: 传入的定时器句柄无效。
    - `ESP_ERR_INVALID_STATE`: 定时器已经启用或尚未正确初始化。
    - `ESP_FAIL`: 其他错误。
- **功能**:
    - 这个函数主要是使定时器进入可运行状态，启用相关的硬件和软件资源，准备定时器开始计数。
    - 此函数通常在设置完定时器的配置、报警行为及回调之后调用，但在调用 `gptimer_start()` 使定时器开始计数前调用。
    - 如果定时器使用了中断，此函数可能会配置和启用相关的中断。
## 二、general_purpose_timer.c
---
![[Pasted image 20240904205122.png]]

![[Pasted image 20240904205135.png]]

## 三、main.c
---
![[Pasted image 20240904205153.png]]
