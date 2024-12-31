# 导言
---
![[Pasted image 20241231113230.png]]
在FOC（Field-Oriented Control）框架中，Power Inverter（电源逆变器）扮演着将直流电（DC）转换为交流电（AC）的关键角色。这个转换是必要的，因为电机通常需要交流电来运行，而系统中的电源可能提供的是直流电。
在图中，逆变器接收来自SVPWM生成器的控制信号，并根据这些信号调整其输出，从而驱动PMSM电机按照预定的速度和转矩运行。通过这一过程，逆变器在FOC系统中起到了将控制算法的指令转化为实际电机驱动的桥梁作用。

项目源码:https://github.com/q164129345/MCU_Develop/tree/main/simplefoc04_stm32f103_3_phrase_pwm

# 一、电机开发板的三相逆变电路分析
---
![[Pasted image 20241227141718.png]]
如上图所示，simpleFOC的源码提供两种PWM驱动：
1. 3PWM
2. 6PWM

选择哪种驱动，取决于开发板的三相逆变电路的设计。

![[Pasted image 20241227142137.png]]
如上所示，电机驱动板的三相逆变电路只用了3路PWM，还有1路Enable控制，一共4个GPIO。所以，这个电机控制板需要移植的是BLDCDriver3PWM.cpp与.h的源码。

# 二、simpleFOC源码移植
---
## 2.1、BLDCDriver3PWM
![[Pasted image 20241227141437.png | 1200]]

# 三、CubeMX
---
## 3.1、TIM2
**TIM2的目的是产生中间对齐的PWM，并在下半桥导通后的中间位置触发定时器中断，进行电流采样。** 下面，将用示波器抓波形核对一遍。
### 3.1.1、Mode
![[Pasted image 20241227161205.png]]
### 3.1.2、Configuration
![[Pasted image 20241227164217.png]]
如上所示，完成Parameter Settings。为什么这样设置TIM去产生PWM，可以参考CSDN另外一篇详细的笔记：https://blog.csdn.net/wallace89/article/details/144520720?spm=1001.2014.3001.5501
![[Pasted image 20241227163507.png]]
![[Pasted image 20241227163545.png]]

# 四、代码
---
## 4.1、BLDCDriver3PWM.cpp
代码含以下函数：
1. BLDCDriver3PWM::BLDCDriver3PWM()，构造函数
2. BLDCDriver3PWM::enable()，使能三相逆变电路
3. BLDCDriver3PWM::disable()，失能三相逆变电路
4. BLDCDriver3PWM::init()，三相逆变电路初始化
5. BLDCDriver3PWM::setPwm()，根据控制的电压值转换为PWM占空比%
![[Pasted image 20241231152401.png]]
![[Pasted image 20241231152555.png]]
![[Pasted image 20241231152716.png]]
```C++
#include "BLDCDriver3PWM.h"

#define PWM_TIM htim2 // pwm定时器接口（本项目使用htim2）

BLDCDriver3PWM::BLDCDriver3PWM(int phA, int phB, int phC, int en1, int en2, int en3){
  // Pin initialization
  pwmA = phA;
  pwmB = phB;
  pwmC = phC;

  // enable_pin pin
  enableA_pin = en1;
  enableB_pin = en2;
  enableC_pin = en3;

  // default power-supply value
  voltage_power_supply = DEF_POWER_SUPPLY;
  voltage_limit = NOT_SET;
  pwm_frequency = NOT_SET;
}

// enable motor driver
void  BLDCDriver3PWM::enable(){
  // enable_pin the driver - if enable_pin pin available
  HAL_GPIO_WritePin(m1_enable_GPIO_Port, m1_enable_Pin, GPIO_PIN_SET); // 使能半桥驱动芯片
  // set zero to PWM
  setPwm(0,0,0);
  SEGGER_RTT_printf(0,"enable motor.\n");
}

// disable motor driver
void BLDCDriver3PWM::disable()
{
  // set zero to PWM
  setPwm(0, 0, 0);
  // disable the driver - if enable_pin pin available
  HAL_GPIO_WritePin(m1_enable_GPIO_Port, m1_enable_Pin, GPIO_PIN_RESET); // 不使能半桥驱动芯片
  SEGGER_RTT_printf(0,"disable motor.\n");
}

// init hardware pins
int BLDCDriver3PWM::init() {
  // sanity check for the voltage limit configuration
  if(!_isset(voltage_limit) || voltage_limit > voltage_power_supply) voltage_limit =  voltage_power_supply;

  // Set the pwm frequency to the pins
  pwm_frequency = __HAL_TIM_GET_AUTORELOAD(&PWM_TIM); // 获取pwm频率，即ARR寄存器
  voltage_limit = DEF_POWER_SUPPLY; // 获取电压限制
  HAL_TIM_PWM_Start(&PWM_TIM, TIM_CHANNEL_1); // 使能A相
  HAL_TIM_PWM_Start(&PWM_TIM, TIM_CHANNEL_2); // 使能B相
  HAL_TIM_PWM_Start(&PWM_TIM, TIM_CHANNEL_3); // 使能C相
  SEGGER_RTT_printf(0, "pwm_frequency:%u,voltage_limit:", pwm_frequency);
  SEGGER_Printf_Float(voltage_limit);
  SEGGER_RTT_printf(0,"\n");
  return 1;
}

// Set voltage to the pwm pin
void BLDCDriver3PWM::setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc) {
  // disable if needed
  // if( _isset(enableA_pin) &&  _isset(enableB_pin)  && _isset(enableC_pin) ){
  //   digitalWrite(enableA_pin, sa == PhaseState::PHASE_ON ? enable_active_high:!enable_active_high);
  //   digitalWrite(enableB_pin, sb == PhaseState::PHASE_ON ? enable_active_high:!enable_active_high);
  //   digitalWrite(enableC_pin, sc == PhaseState::PHASE_ON ? enable_active_high:!enable_active_high);
  // }
}

// Set voltage to the pwm pin
void BLDCDriver3PWM::setPwm(float Ua, float Ub, float Uc) {

  // limit the voltage in driver
  Ua = _constrain(Ua, 0.0f, voltage_limit);
  Ub = _constrain(Ub, 0.0f, voltage_limit);
  Uc = _constrain(Uc, 0.0f, voltage_limit);
  // calculate duty cycle
  // limited in [0,1]
  dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
  dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
  dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

  // hardware specific writing
  // hardware specific function - depending on driver and mcu
  //_writeDutyCycle3PWM(dc_a, dc_b, dc_c, params);
  _writeDutyCycle3PWM(&PWM_TIM, dc_a, dc_b, dc_c);
}

```

## 4.2、BLDCDriver3PWM.h
![[Pasted image 20241231153516.png]]
```C++
#ifndef BLDCDriver3PWM_h
#define BLDCDriver3PWM_h

#include "main.h"
#include "tim.h"
#include "BLDCDriver.h"
#include "foc_utils.h"
#include "time_utils.h"
#include "defaults.h"
//#include "hardware_api.h"
#include "bsp_pwm.h"

/**
 3 pwm bldc driver class
*/
class BLDCDriver3PWM: public BLDCDriver
{
  public:
    /**
      BLDCDriver class constructor
      @param phA A phase pwm pin
      @param phB B phase pwm pin
      @param phC C phase pwm pin
      @param en1 enable pin (optional input)
      @param en2 enable pin (optional input)
      @param en3 enable pin (optional input)
    */
    BLDCDriver3PWM(int phA,int phB,int phC, int en1 = NOT_SET, int en2 = NOT_SET, int en3 = NOT_SET);
    
    /**  Motor hardware init function */
  	int init() override;
    /** Motor disable function */
  	void disable() override;
    /** Motor enable function */
    void enable() override;

    // hardware variables
  	int pwmA; //!< phase A pwm pin number
  	int pwmB; //!< phase B pwm pin number
  	int pwmC; //!< phase C pwm pin number
    int enableA_pin; //!< enable pin number
    int enableB_pin; //!< enable pin number
    int enableC_pin; //!< enable pin number
    bool enable_active_high = true;

    /** 
     * Set phase voltages to the harware 
     * 
     * @param Ua - phase A voltage
     * @param Ub - phase B voltage
     * @param Uc - phase C voltage
    */
    void setPwm(float Ua, float Ub, float Uc) override;

    /** 
     * Set phase voltages to the harware 
     * 
     * @param sc - phase A state : active / disabled ( high impedance )
     * @param sb - phase B state : active / disabled ( high impedance )
     * @param sa - phase C state : active / disabled ( high impedance )
    */
    virtual void setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc) override;
  private:
};


#endif

```

## 4.3、bsp_pwm.c
![[Pasted image 20241231153751.png]]
```c
#include "bsp_pwm.h"

/**
 * @brief 设置三相PWM的占空比
 * 
 * @param htimx 定时器句柄
 * @param dc_a A相的占空比%
 * @param dc_b B相的占空比%
 * @param dc_c C相的占空比%
 */
void _writeDutyCycle3PWM(TIM_HandleTypeDef *htimx, float dc_a, float dc_b, float dc_c)
{
    // 计算CCR值
    uint32_t ccr_a = (uint32_t)(dc_a * __HAL_TIM_GET_AUTORELOAD(htimx));
    uint32_t ccr_b = (uint32_t)(dc_b * __HAL_TIM_GET_AUTORELOAD(htimx));
    uint32_t ccr_c = (uint32_t)(dc_c * __HAL_TIM_GET_AUTORELOAD(htimx));

    // 设置PWM占空比
    __HAL_TIM_SET_COMPARE(htimx, TIM_CHANNEL_1, ccr_a);
    __HAL_TIM_SET_COMPARE(htimx, TIM_CHANNEL_2, ccr_b);
    __HAL_TIM_SET_COMPARE(htimx, TIM_CHANNEL_3, ccr_c);
}
```
## 4.4、bsp_pwm.h
![[Pasted image 20241231154512.png]]
```C
/*
 * @brief 这个库用于STM32F103的三相PWM输出控制
 * 
 * 该库提供了一个函数，用于设置三相PWM的占空比。
 * 用户可以通过调用_writeDutyCycle3PWM函数，传入A、B、C三相的占空比，
 * 来控制PWM输出。
 * 
 * 主要功能：
 * - 设置三相PWM的占空比
 * 
 * 使用示例：
 * @code
 * _writeDutyCycle3PWM(0.5, 0.5, 0.5); // 设置A、B、C三相的占空比为50%
 * @endcode
 */
#ifndef _BSP_PWM_H_
#define _BSP_PWM_H_

#include "main.h"
#include "tim.h"

#ifdef __cplusplus
extern "C" {
#endif


// public
void _writeDutyCycle3PWM(TIM_HandleTypeDef *htimx, float dc_a, float dc_b, float dc_c);


// private


#ifdef __cplusplus
} 
#endif

#endif

```
## 4.5、user_main.cpp
![[Pasted image 20241231155758.png]]
```C
#include "user_main.h"
#include "AS5600_I2C.h"
#include "BLDCDriver3PWM.h"

// 外部变量
extern struct AS5600_I2CConfig_s AS5600_I2C_Config;
extern I2C_HandleTypeDef hi2c1;

// 全局变量
AS5600_I2C AS5600_1(AS5600_I2C_Config); // 创建AS5600_I2C对象
float g_Velocity; // 便于使用J-LINK Scope观察曲线
BLDCDriver3PWM motorDriver(GPIO_PIN_0,GPIO_PIN_1,GPIO_PIN_2); // PA0,PA1,PA2

/**
 * @brief C++环境入口函数
 * 
 */
void main_Cpp(void)
{
    AS5600_1.init(&hi2c1); // 初始化AS5600
    motorDriver.init(); // 初始化电机驱动
    motorDriver.enable(); // 使能电机驱动
    motorDriver.setPwm(6,6,6); // 设置50%占空比
    HAL_Delay(1000); // 延时1s
    while(1) {
        HAL_GPIO_TogglePin(run_led_GPIO_Port,run_led_Pin); // 心跳灯跑起来
        //SEGGER_RTT_printf(0,"Dwt_time_us:%u\n",_micros()); // 打印DWT定时器的us级时间戳，看看有没有问题
        AS5600_1.update(); // 更新位置，获取速度
        g_Velocity = AS5600_1.getVelocity(); // 获取速度
        SEGGER_RTT_printf(0,"motor_Velocity:%f\n",g_Velocity);
        SEGGER_Printf_Float(g_Velocity); // 打印电机速度
        delayMicroseconds(100000U); // 延时10ms
    }
}

```
# 五、调试
---
## 5.1、示波器确认PWM波形
![[Pasted image 20241231154911.png]]
![[Pasted image 20241231155604.png]]
如上图所示，示波器测量出来的PWM波形的频率是20KHz，占空比50.2%，没有问题。


# 六、细节补充
---
## 6.1、验证电流采样的中断回调
