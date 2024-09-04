## 导言
---
![[Pasted image 20240904210106.png]]
如上图所示，KEY0 ~ KEY3.
![[Pasted image 20240904210120.png]]


简单概括一下，XL9555 可使用 400kHz 速率的 IIC 通信接口与微控制器进行连接，也就是 用 2 根通信线可扩展使用 16 个 IO。XL9555 器件地址会由三个硬件地址引脚决定，理论上在这 个 IIC 总线上可挂载 8 个 XL9555 器件，足以满足 IO 引脚需求。XL9555 上电进行复位，16 个 I/O 口默认为输入模式，当输入模式的 IO 口状态发生变化时，即发生从高电平变低电平或者从 低电平变高电平，中断脚会拉低。当中断有效后，必须对 XL9555 进行一次读取/写入操作，复 位中断，才可以输出下一次中断，否则中断将一直保持。
源码地址：[https://github.com/q164129345/esp32_Learning/tree/main/code_I2C_XL9555](https://github.com/q164129345/esp32_Learning/tree/main/code_I2C_XL9555)
效果如下：
![[Pasted image 20240904210136.png]]
## 一、寻址
---
![[Pasted image 20240904210158.png]]
## 二、寄存器介绍
---
![[Pasted image 20240904210214.png]]

### 2.1、地址0x00、0x01
![[Pasted image 20240904210230.png]]
### 2.2、地址0x02、0x03
![[Pasted image 20240904210242.png]]
### 2.3、地址0x04、0x05
![[Pasted image 20240904210317.png]]
### 2.4、地址0x06、0x07
![[Pasted image 20240904210336.png]]


## 三、I2C驱动
---
正点原子编写的I2C驱动不错，直接Copy..
![[Pasted image 20240904210353.png]]
学习ESP-IDF接近一周了，套路都差不多，初始化外设基本都是找一个config结构体，然后调用config函数配置一下参数，接着再调用install函数安装驱动。

## 四、XL9555驱动
---
### 1. `void xl9555_init(i2c_obj_t self);`
此函数用于初始化 XL9555 设备，配置其使用的 I2C 接口，包括相关 GPIO 的设置和基础设备配置。
```c
xl9555_init(xl9555_i2c_master); // 初始化 XL9555 设备
```
- **参数**:
    - `self`: I2C 对象，包含 I2C 配置信息和端口号。
### 2. `int xl9555_pin_read(uint16_t pin);`
该函数用于读取指定 XL9555 IO口的状态（高或低）。

```c
int pin_status = xl9555_pin_read(BEEP_IO); // 读取 BEEP_IO 状态
```
- **参数**:
    - `pin`: 指定的 IO 口编号。
### 3. `uint16_t xl9555_pin_write(uint16_t pin, int val);`
此函数用于控制指定 IO 口的电平，修改后返回所有 IO 的状态。
```c
uint16_t all_pins = xl9555_pin_write(BEEP_IO, 1); // 设置 BEEP_IO 为高电平
```
- **参数**:
    - `pin`: IO 口编号。
    - `val`: 要设置的电平值。
### 4. `esp_err_t xl9555_read_byte(uint8_t* data, size_t len);`
该函数从 XL9555 读取指定长度的数据，通常用于获取设备状态或 IO 口电平值。
```c
uint8_t data[2];
esp_err_t status = xl9555_read_byte(data, 2); // 读取两个字节的数据
```
- **参数**:
    - `data`: 存储读取数据的缓冲区。
    - `len`: 要读取的数据长度。