## 导言
---
开发板：
![[Pasted image 20240904203032.png]]
LED电路：
![[Pasted image 20240904203048.png]]

从电路看来，属于灌电流。所以：

- IO1低电平时，LED亮
- IO2高电平时，LED灭

本工程源码：
## 一、GPIO源码
---
GPIO驱动的源码，esp32官方已经帮我们准备好了，它相当于STM32的HAL库。首先，需要`include<driver/gpio.h>`
![[Pasted image 20240904203150.png]]
### 1.1、gpio.num.h
代码路径：esp/v5.2.1/esp-idf/components/soc/esp32s3/include/soc/gpio.num.h
gpio.num.h文件只有一个枚举变量gpio_num_t，描述GPIO的号码。
![[Pasted image 20240904203219.png]]
GPIO_NUM_1代表GPIO的IO1.
![[Pasted image 20240904203236.png]]
### 1.2、gpio.h

代码路径：esp/v5.2.1/esp-idf/components/driver/gpio/include/driver/gpio.h
- gpio_config_t
    - 结构体，配置GPIO口的功能
```c
typedef struct {
    uint64_t pin_bit_mask;          /*!< GPIO pin: set with bit mask, each bit maps to a GPIO */
    gpio_mode_t mode;               /*!< GPIO mode: set input/output mode                     */
    gpio_pullup_t pull_up_en;       /*!< GPIO pull-up                                         */
    gpio_pulldown_t pull_down_en;   /*!< GPIO pull-down                                       */
    gpio_int_type_t intr_type;      /*!< GPIO interrupt type                                  */
#if SOC_GPIO_SUPPORT_PIN_HYS_FILTER
    gpio_hys_ctrl_mode_t hys_ctrl_mode;       /*!< GPIO hysteresis: hysteresis filter on slope input    */
#endif
} gpio_config_t;
```

- esp_err_t gpio_config(_const_ gpio_config_t ***pGPIOConfig**)
    - 根据结构体gpio_config_t，初始化GPIO口。比如，上拉，下拉，中断等
```c
/**
 * @brief GPIO common configuration
 *
 *        Configure GPIO's Mode,pull-up,PullDown,IntrType
 *
 * @param  pGPIOConfig Pointer to GPIO configure struct
 *
 * @return
 *     - ESP_OK success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *
 */
esp_err_t gpio_config(const gpio_config_t *pGPIOConfig)
```
- esp_err_t gpio_set_level(gpio_num_t **gpio_num,** _uint32_t_ **level**)
    - 设置GPIO的当前电平
```c
/**
 * @brief  GPIO set output level
 *
 * @note This function is allowed to be executed when Cache is disabled within ISR context, by enabling `CONFIG_GPIO_CTRL_FUNC_IN_IRAM`
 *
 * @param  gpio_num GPIO number. If you want to set the output level of e.g. GPIO16, gpio_num should be GPIO_NUM_16 (16);
 * @param  level Output level. 0: low ; 1: high
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG GPIO number error
 *
 */
esp_err_t gpio_set_level(gpio_num_t gpio_num, uint32_t level);
```
- _int_ gpio_get_level(gpio_num_t **gpio_num**)
    - 获取GPIO的当前电平
```c
/**
 * @brief  GPIO get input level
 *
 * @warning If the pad is not configured for input (or input and output) the returned value is always 0.
 *
 * @param  gpio_num GPIO number. If you want to get the logic level of e.g. pin GPIO16, gpio_num should be GPIO_NUM_16 (16);
 *
 * @return
 *     - 0 the GPIO input level is 0
 *     - 1 the GPIO input level is 1
 */
int gpio_get_level(gpio_num_t gpio_num);
```

## 二、main.c
---
简单的流程图如下：
![[Pasted image 20240904203324.png]]

main.c源码如下：
```c
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>

#define LED_GPIO_PIN GPIO_NUM_1  // LED连接到IO1
void led_Init(void);
void led_Toggle(void);

/**
 * @brief 主函数
 * 
 */
void app_main(void)
{
    led_Init(); // 初始化GPIO1
    while(1) {
        led_Toggle(); // 翻转LED电平
        
    }
}

/**
 * @brief 初始化GPIO口
 * 
 */
void led_Init(void)
{
    gpio_config_t gpio_handler = {0,};
    gpio_handler.intr_type = GPIO_INTR_DISABLE;        // 关闭引脚中断
    gpio_handler.mode = GPIO_MODE_INPUT_OUTPUT;        // 输入、输出模式（一定要输入输出模式，否则无法读取pin脚的电平
    gpio_handler.pull_up_en = GPIO_PULLUP_ENABLE;      // 使能上拉
    gpio_handler.pull_down_en = GPIO_PULLDOWN_DISABLE; // 失能下拉
    gpio_handler.pin_bit_mask = 1ULL << LED_GPIO_PIN;  // 设置引脚位掩码
    gpio_config(&gpio_handler);
}

/**
 * @brief 翻转LED的电平
 * 
 */
void led_Toggle(void)
{
    static uint32_t cnt = 0;
    gpio_set_level(LED_GPIO_PIN,cnt % 2);
    cnt++;
}

```