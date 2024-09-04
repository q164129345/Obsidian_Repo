## 导言
---
![[Pasted image 20240904204805.png]]
使用 esp_timer 来创建和管理软件定时器。这种类型的定时器非常适用于不需要极高精度的定时任务。然后，代码实在简单。
效果如下：
![[Pasted image 20240904204834.png]]

工程源码：[https://github.com/q164129345/esp32_Learning/tree/main/code_SysTimer](https://github.com/q164129345/esp32_Learning/tree/main/code_SysTimer)

## 一、相关函数
---
### 1. `esp_timer_create()`
这个函数用于创建一个新的 `esp_timer` 定时器实例。
```c
esp_err_t esp_timer_create(const esp_timer_create_args_t* create_args, esp_timer_handle_t* out_handle);
```
- **参数**:
    - `create_args`：指向 `esp_timer_create_args_t` 结构的指针，该结构包含定时器的配置，如回调函数、传递给回调函数的参数和定时器名称。
    - `out_handle`：输出参数，用于返回创建的定时器的句柄。
- **返回值**:
    - 返回 `ESP_OK` 表示定时器创建成功，其他错误代码表示失败。
- **功能**:
    - 创建一个定时器实例，该实例在配置的回调函数中执行用户定义的代码。

### 2. `esp_timer_start_periodic()`
这个函数用于启动一个周期性定时器。
```c
esp_err_t esp_timer_start_periodic(esp_timer_handle_t timer, uint64_t period);
```
- **参数**:
    - `timer`：定时器的句柄，由 `esp_timer_create` 返回。
    - `period`：定时器的周期，单位是微秒(us)。
- **返回值**:
    - 返回 `ESP_OK` 表示定时器启动成功，其他错误代码表示失败。
- **功能**:
    - 启动一个周期性定时器，每隔指定的周期，定时器超时并调用其回调函数。

### 3. `esp_timer_start_once()`
这个函数用于启动一个一次性定时器。
```c
esp_err_t esp_timer_start_once(esp_timer_handle_t timer, uint64_t timeout);
```
- **参数**:
    - `timer`：定时器的句柄。
    - `timeout`：定时器超时时间，单位是微秒(us)。
- **返回值**:
    - 返回 `ESP_OK` 表示定时器启动成功，其他错误代码表示失败。
- **功能**:
    - 启动一个一次性定时器，当达到指定的超时时间后，定时器触发并调用其回调函数一次。

### 4. `esp_timer_get_time()`
这个函数用于获取自系统启动以来的时间。
```c
int64_t esp_timer_get_time(void);
```
- **返回值**:
    - 返回自系统启动以来的时间，单位是微秒(us)。
- **功能**:
    - 获取自系统启动以来的时间，常用于时间测量和计时。

### 5. `esp_timer_stop()`
这个函数用于停止一个正在运行的定时器。
```c
esp_err_t esp_timer_stop(esp_timer_handle_t timer);
```

- **参数**:
    - `timer`：定时器的句柄。
- **返回值**:
    - 返回 `ESP_OK` 表示定时器成功停止，其他错误代码表示失败。
- **功能**:
    - 停止定时器，防止其回调函数被进一步调用。

### 6. `esp_timer_delete()`
这个函数用于删除一个定时器实例。
```c
esp_err_t esp_timer_delete(esp_timer_handle_t timer);
```

- **参数**:
    - `timer`：定时器的句柄。
- **返回值**:
    - 返回 `ESP_OK` 表示定时器删除成功，其他错误代码表示失败。
- **功能**:
    - 删除定时器实例，释放相关资源。在删除定时器之前应确保定时器已经停止。

## 二、simple_timer.c
---
![[Pasted image 20240904204857.png]]
## 三、main.c
---
![[Pasted image 20240904204910.png]]
调用timer_Init()初始化定时器，然后调用timer_Start()启动定时器即可。