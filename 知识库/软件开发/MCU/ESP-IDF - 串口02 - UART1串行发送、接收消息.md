## 导言
---
![[Pasted image 20240904204310.png]]
在上一个章节了解UART注意事项之后，开始写代码。
本次实验的效果是，UART1接收到一段数据后，立刻发送回去。代码简单，只是为了说清楚API函数的用法。实际项目上，建议用消息队列与任务来编写程序，最大程度提高程序的实时性。
![[Pasted image 20240904204335.png]]
源码地址：[https://github.com/q164129345/esp32_Learning/tree/main/code_Uart_Echo](https://github.com/q164129345/esp32_Learning/tree/main/code_Uart_Echo)
![[Pasted image 20240904204353.png]]
## 一、UART发送数据的相关函数
---
### 1. `uart_write_bytes()`
此函数用于将数据发送到 UART 设备。它将数据从应用程序的内存复制到驱动程序的发送（Tx）缓冲区。如果缓冲区已满，该函数会等待直到有足够的空间可用。
```c
// 示例代码：将字符串发送到指定的 UART 端口
char* test_str = "This is a test string.\\n";
uart_write_bytes(uart_num, (const char*)test_str, strlen(test_str));
```
- **参数**:
    - `uart_num`: 指定 UART 端口号。
    - `test_str`: 要发送的数据。
    - `strlen(test_str)`: 要发送的数据长度。

### 2. `uart_write_bytes_with_break()`
此函数与 `uart_write_bytes()` 类似，但它会在数据发送完毕后在 Tx 线上生成一个 "break" 信号，即保持线路一段时间为低电平，通常用于某些特定的通信协议。

```c
// 示例代码：发送数据并在末尾添加一个串行中断信号
uart_write_bytes_with_break(uart_num, "test break\\n", strlen("test break\\n"), 100);
```

- **参数**:
    - `100`: 表示在发送完毕后，Tx 线上将保持低电平的时间（以 UART 字符帧的时间单位计）。

### 3. `uart_tx_chars()`
此函数用于向 UART 的硬件 Tx FIFO 直接发送数据，如果 FIFO 没有足够的空间，它将不会阻塞等待，而是返回实际写入的字节数。

```c
// 示例代码：尝试发送数据，不等待 Tx 缓冲区空间
int num_written = uart_tx_chars(uart_num, "Quick brown fox", 15);
```

- **说明**:
    - `num_written`: 返回值表示实际写入到硬件 Tx FIFO 的字节数。

### 4. `uart_wait_tx_done()`
该函数用于阻塞当前任务，直到所有的 Tx FIFO 中的数据都被发送完毕，或者达到指定的超时时间。

```c
// 示例代码：等待直到所有数据发送完毕或超时
const uart_port_t uart_num = UART_NUM_2;
ESP_ERROR_CHECK(uart_wait_tx_done(uart_num, 100)); // 等待超时设置为 100 RTOS tick
```

- **参数**:
    - `100`: 超时时间，以 RTOS 的 tick 为单位。

### 使用场景和注意事项
- 确保在调用这些发送函数之前，UART 已经正确配置，包括波特率、数据位、停止位等。
- 选择发送函数时，考虑到数据发送的即时性和对阻塞的容忍度。
- 在使用 `uart_write_bytes_with_break()` 和 `uart_wait_tx_done()` 时，需要考虑其对通信协议的影响和实时操作系统的调度。

## 二、UART接收数据的相关函数
---
### 1. `uart_get_buffered_data_len()`
在读取 UART 数据之前，了解 Rx FIFO 缓冲区中有多少数据是非常有用的。这可以通过 `uart_get_buffered_data_len()` 函数完成。这个函数可以让您知道有多少字节的数据已经准备好被读取，这样您可以确保在读取操作中使用正确的字节长度。
```c
// 检查 UART 缓冲区中有多少数据可读
const uart_port_t uart_num = UART_NUM_2;
int length = 0;
ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t*)&length));
```
- `uart_get_buffered_data_len(uart_num, (size_t*)&length)`: 此函数将可用数据的长度存入 `length` 变量。
### 2. `uart_read_bytes()`
一旦您知道了 Rx FIFO 中有数据，可以使用 `uart_read_bytes()` 函数来读取这些数据。这个函数从 UART 的接收缓冲区中读取指定数量的字节，可以设置超时，以防数据读取不完全。
```c
// 从 UART 读取数据
uint8_t data[128]; // 数据存储缓冲区
length = uart_read_bytes(uart_num, data, length, 100);
```

- `uart_read_bytes(uart_num, data, length, 100)`: 从 UART 缓冲区读取 `length` 字节的数据到 `data` 数组中，如果指定时间内没有足够数据，则等待最多 100 RTOS ticks。
### 3. `uart_flush()`
在某些情况下，如果你确定 Rx FIFO 中的数据不再需要，可以使用 `uart_flush()` 函数来清空缓冲区。这对于错误恢复或重置通信状态是非常有用的。

```c
// 清空 UART 的 Rx FIFO 缓冲区
ESP_ERROR_CHECK(uart_flush(uart_num));
```
- `uart_flush(uart_num)`: 清空指定 UART 端口的接收缓冲区。

### 总结
- 使用 `uart_get_buffered_data_len()` 来检查接收缓冲区中的数据量，确保你读取正确数量的数据。
- 使用 `uart_read_bytes()` 来实际读取数据，根据需要设置合适的超时。
- 如果必要，使用 `uart_flush()` 来清空接收缓冲区，特别是在错误发生后或当数据不再需要时。

## 三、uart_echo.c
---
![[Pasted image 20240904204452.png]]
![[Pasted image 20240904204503.png]]

## 四、main.c
---
![[Pasted image 20240904204520.png]]