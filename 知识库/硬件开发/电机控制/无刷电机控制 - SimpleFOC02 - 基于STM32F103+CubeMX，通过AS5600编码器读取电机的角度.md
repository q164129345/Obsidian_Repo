# 导言
---
[[无刷电机控制 - SimpleFOC01 - 基于STM32F103+CubeMX，移植核心的common代码]]
上一章节完成common代码的移植，下一步是完成位置编码器的角度读取。
![[Pasted image 20241226203127.png]]
如上所示，通过`getSensorAngle()`方法可以获取电机的当前角度信息。

![[Pasted image 20241225203755.png | 500]]
如上所示，通过AS5600编码器获取云台电机的位置角度信息。
![[Pasted image 20241226193821.png]]
如上所示，使用FOC开发板跟云台电机的编码器AS5600的接线。
![[AS5600_Angle.gif]]
最终的效果如上。
项目源码:https://github.com/q164129345/MCU_Develop/tree/main/simplefoc02_stm32f103

# 一、Keil
---
![[Pasted image 20241225204308.png | 1100]]
![[Pasted image 20241225204405.png | 1100]]

# 二、CubeMX
---
![[Pasted image 20241226141529.png | 1100]]



# 三、代码
---
![[Pasted image 20241225204652.png | 1100]]
因为AS5600使用的是I2C通讯接口，跟源码MagneticSensorI2C类很相似，可以直接Copy过来修改。

## 3.1、先解决编译失败问题

**simpleFOC源码是基于Arduino框架编写的，需要先注释掉Arduino代码。** 接着，根据STM32的HAL库框架来编写AS5600_I2C类。

## 3.1.1、AS5600_I2C.h
```C++
#ifndef AS5600_I2C_H
#define AS5600_I2C_H

#include "main.h"
//#include <Wire.h>
#include "Sensor.h"
#include "foc_utils.h"
#include "time_utils.h"

struct AS5600_I2CConfig_s  {
  int chip_address;
  int bit_resolution;
  int angle_register;
  int data_start_bit; 
};
// some predefined structures
// extern AS5600_I2CConfig_s AS5600_I2C;

class AS5600_I2C: public Sensor{
 public:
    /**
     * AS5600_I2C class constructor
     * @param chip_address  I2C chip address
     * @param bits number of bits of the sensor resolution 
     * @param angle_register_msb  angle read register msb
     * @param _bits_used_msb number of used bits in msb
     */
    AS5600_I2C(uint8_t _chip_address, int _bit_resolution, uint8_t _angle_register_msb, int _msb_bits_used);

    /**
     * AS5600_I2C class constructor
     * @param config  I2C config
     */
    AS5600_I2C(AS5600_I2CConfig_s config);

    // static AS5600_I2C AS5600();
        
    /** sensor initialise pins */
    //void init(TwoWire* _wire = &Wire); // 根据STM32的框架，重新写
    void init(I2C_HandleTypeDef* hi2c1);

    // implementation of abstract functions of the Sensor class
    /** get current angle (rad) */
    float getSensorAngle() override;

    /** experimental function to check and fix SDA locked LOW issues */
    //int checkBus(byte sda_pin , byte scl_pin );

    /** current error code from Wire endTransmission() call **/
    uint8_t currWireError = 0;

  private:
    float cpr; //!< Maximum range of the magnetic sensor
    uint16_t lsb_used; //!< Number of bits used in LSB register
    uint8_t lsb_mask;
    uint8_t msb_mask;
    
    // I2C variables
    uint8_t angle_register_msb; //!< I2C angle register to read
    uint8_t chip_address; //!< I2C chip select pins

    // I2C functions
    /** Read one I2C register value */
    int read(uint8_t angle_register_msb);

    /**
     * Function getting current angle register value
     * it uses angle_register variable
     */
    int getRawCount();
    
    /* the two wire instance for this sensor */
    //TwoWire* wire;
    I2C_HandleTypeDef* hi2c1;

};


#endif

```
## 3.1.2、AS5600_I2C.cpp
```C++
#include "AS5600_I2C.h"

/** Typical configuration for the 12bit AMS AS5600 magnetic sensor over I2C interface */
struct AS5600_I2CConfig_s AS5600_I2C_Config = {
  .chip_address   = 0x36, // AS5600的I2C地址
  .bit_resolution = 12,   // 分辨率
  .angle_register = 0x0C, // 角度寄存器
  .data_start_bit = 11    // 数据起始位
};

/** Typical configuration for the 12bit AMS AS5048 magnetic sensor over I2C interface */
// MagneticSensorI2CConfig_s AS5048_I2C = {
//   .chip_address = 0x40,  // highly configurable.  if A1 and A2 are held low, this is probable value
//   .bit_resolution = 14,
//   .angle_register = 0xFE,
//   .data_start_bit = 15
// };

// MagneticSensorI2C(uint8_t _chip_address, float _cpr, uint8_t _angle_register_msb)
//  @param _chip_address  I2C chip address
//  @param _bit_resolution  bit resolution of the sensor
//  @param _angle_register_msb  angle read register
//  @param _bits_used_msb number of used bits in msb
AS5600_I2C::AS5600_I2C(uint8_t _chip_address, int _bit_resolution, uint8_t _angle_register_msb, int _bits_used_msb){
    // chip I2C address
    chip_address = _chip_address;
    // angle read register of the magnetic sensor
    angle_register_msb = _angle_register_msb;
    // register maximum value (counts per revolution)
    cpr = _powtwo(_bit_resolution);

    // depending on the sensor architecture there are different combinations of
    // LSB and MSB register used bits
    // AS5600 uses 0..7 LSB and 8..11 MSB
    // AS5048 uses 0..5 LSB and 6..13 MSB
    // used bits in LSB
    lsb_used = _bit_resolution - _bits_used_msb;
    // extraction masks
    lsb_mask = (uint8_t)( (2 << lsb_used) - 1 );
    msb_mask = (uint8_t)( (2 << _bits_used_msb) - 1 );
    //wire = &Wire;
}

AS5600_I2C::AS5600_I2C(AS5600_I2CConfig_s config){
    chip_address = config.chip_address; 

    // angle read register of the magnetic sensor
    angle_register_msb = config.angle_register;
    // register maximum value (counts per revolution)
    cpr = _powtwo(config.bit_resolution);

    int bits_used_msb = config.data_start_bit - 7;
    lsb_used = config.bit_resolution - bits_used_msb;
    // extraction masks
    lsb_mask = (uint8_t)( (2 << lsb_used) - 1 );
    msb_mask = (uint8_t)( (2 << bits_used_msb) - 1 );
    //wire = &Wire;
}

// 初始化
void AS5600_I2C::init(I2C_HandleTypeDef* _hi2c1){
  hi2c1 = _hi2c1;
  this->Sensor::init(); // call base class init
}

//  Shaft angle calculation
//  angle is in radians [rad]
float AS5600_I2C::getSensorAngle(){
  // (number of full rotations)*2PI + current sensor angle 
  return  ( getRawCount() / (float)cpr) * _2PI ;
}

// function reading the raw counter of the magnetic sensor
int AS5600_I2C::getRawCount(){
    return (int)AS5600_I2C::read(angle_register_msb);
}

// I2C functions
/*
* Read a register from the sensor
* Takes the address of the register as a uint8_t
* Returns the value of the register
*/
int AS5600_I2C::read(uint8_t angle_reg_msb) {
    uint8_t buffer[2]; // 用于存储读取的数据
    HAL_StatusTypeDef status;
    // 发送寄存器地址
    status = HAL_I2C_Mem_Read(hi2c1, chip_address << 1, angle_reg_msb, I2C_MEMADD_SIZE_8BIT, buffer, 2, 50);
    if (status != HAL_OK) {
        // 处理错误
        return -1; // 返回错误代码
    }
    // 将读取的两个字节组合成一个整数
    int value = (buffer[0] << 8) | buffer[1];
    return value;
}

/*
* Checks whether other devices have locked the bus. Can clear SDA locks.
* This should be called before sensor.init() on devices that suffer i2c slaves locking sda
* e.g some stm32 boards with AS5600 chips
* Takes the sda_pin and scl_pin
* Returns 0 for OK, 1 for other master and 2 for unfixable sda locked LOW
*/
// int AS5600_I2C::checkBus(byte sda_pin, byte scl_pin) {

//   pinMode(scl_pin, INPUT_PULLUP);
//   pinMode(sda_pin, INPUT_PULLUP);
//   delay(250);

//   if (digitalRead(scl_pin) == LOW) {
//     // Someone else has claimed master!");
//     return 1;
//   }

//   if(digitalRead(sda_pin) == LOW) {
//     // slave is communicating and awaiting clocks, we are blocked
//     pinMode(scl_pin, OUTPUT);
//     for (byte i = 0; i < 16; i++) {
//       // toggle clock for 2 bytes of data
//       digitalWrite(scl_pin, LOW);
//       delayMicroseconds(20);
//       digitalWrite(scl_pin, HIGH);
//       delayMicroseconds(20);
//     }
//     pinMode(sda_pin, INPUT);
//     delayMicroseconds(20);
//     if (digitalRead(sda_pin) == LOW) {
//       // SDA still blocked
//       return 2;
//     }
//     _delay(1000);
//   }
//   // SDA is clear (HIGH)
//   pinMode(sda_pin, INPUT);
//   pinMode(scl_pin, INPUT);

//   return 0;
// }

```

## 3.1.3、user_main.cpp
![[Pasted image 20241226193149.png | 1100]]

# 四、调试程序
---
## 4.1、编译
![[Pasted image 20241226115547.png | 1100]]
如上所示，编译通过。

## 4.2、RTT_Viewer
![[AS5600_Angle.gif]]
如上图所示，转动电机时，电机的角度（弧度值）发生变化，范围0 ～ 2PI。

# 五、细节补充
---
## 5.1、I2C的高速模式
### 5.1.1、CubeMX
![[Pasted image 20241226204649.png | 1100]]
### 5.1.2、RTT_Viewer
AS5600支持I2C的高速模式。
![[I2C_HighSpeed.gif]]
