# 导言
---
开始移植SimpleFOC的电流环代码之前，先实现ADC通道采样相电流的功能。
![[Pasted image 20250120204223.png]]
本章节主要目的是完成函数_readADCVoltageInline(pinx)，它会被InlineCurrentSensor类使用，获取相电流。

## 为什么是InlineCurrentSense类，而不是LowsideCurrentSense类？
![[Pasted image 20250120204045.png | 1100]]
从SimpleFOC的源码看到，current_snese里一共有三个类：
1. GenericCurrentSense（这个不清楚，暂时不管）
2. InlineCurrentSense（采样电阻在上桥臂与下桥臂之间）
3. LowsideCurrentSense（采样电阻在下桥臂下方且与GND连接。*多数大功率且高电压的FOC驱动板使用这个方案*）

![[Pasted image 20250120204425.png | 1100]]
如上图所示，选择InlineCurrentSense的原因是电路板的采样电路设计。

**RTT**
![[ADC_RTT.gif]]
项目源码：https://github.com/q164129345/MCU_Develop/tree/main/simplefoc09_stm32f103_adc_sample

# 一、CubeMX
---
## 1.1、ADC1
参考以下博主的文章：
《[STM32CubeMX | STM32使用HAL库的ADC多通道数据采集（DMA+非DMA方式）+ 读取内部传感器温度](https://blog.51cto.com/u_15950551/6031866)》
《[STM32CUBEMX配置教程（十一）STM32的ADC轮询模式扫描多个通道](https://blog.csdn.net/weixin_44584198/article/details/119442880)》
![[Pasted image 20250120205111.png]]
### 1.1.1、Scan Conversion Mode（扫描模式）
目的：让 ADC 能依次转换多个规则通道（Rank1 → Rank2 → …）。
配置：在 CubeMX 中设置“Number of Conversion”为你要采样的通道数，比如 2 个。然后分别把你要采的通道（如 Channel3、Channel4）放到合适的顺位（Rank1、Rank2）。
效果：每次触发（软件触发或硬件触发）时，ADC 会顺序地把这 2 个通道都转换完，*实现了“一次触发，两通道采样”*。
### 1.1.2、Continuous Conversion Mode（连续模式）
目的：让 ADC 在完成一轮“扫描序列”后，自动立即再来下一轮，不停地转换。
配置：*如果只想“一次触发就采完全部通道并停下”，就关闭它（Disabled）*；如果要 ADC 不停地采样，则开启（Enabled）。

### 1.1.3、Discontinuous Conversion Mode（不连续模式）
目的：把本来一次要连着转换的全部通道分成若干小批次，每次只转换指定个数通道，然后等待下一个触发再转换后面的通道。
“Number of Discontinuous Conversions” 决定了“每次触发转换多少个通道”。
例如：假设总共要扫描 4 个通道，Number of Discontinuous Conversions=2，则每次触发会转换 2 个通道，真正完整采完 4 个通道需要触发 2 次。*这一次采样只有两个通道，触发一次即可。所以Disabled。*

### 1.1.4、总结
Scan Conversion Mode = Enabled
Continuous Mode = Disabled（若你只想单次采样）
Discontinuous Mode = Disabled
Number of Conversions = 2
Rank1 = Channel 3，Rank2 = Channel 4
然后 HAL_ADC_Start(&hadc1) 一次，就能顺序完成对 Channel 3、Channel 4 的转换。

# 1.2、ADC时钟频率
![[Pasted image 20250120210035.png | 1100]]


# 二、代码
---
## 2.1、adc.c
![[Pasted image 20250120210115.png | 1100]]
## 2.2、移植SimpleFOC源码的InlineCurrentSense类
![[Pasted image 20250121200953.png | 1100]]
如上所示，先从SimpleFOC源码的LnlineCurrentSense.cpp与.h代码Copy到Keil项目。
![[Pasted image 20250121201248.png | 1100]]
如上图所示，将代码添加到Keil项目上。
![[Pasted image 20250121201356.png | 1100]]
如上所示，添加include路径。
![[Pasted image 20250121201453.png | 1100]]
如上所示，在Keil项目目录看到了LnlineCurrentSense.cpp代码。到此，移植完毕。接下来，稍微修复代码。

## 2.3、修改InlineCurrentSense.cpp
![[Pasted image 20250121204838.png | 1100]]
![[Pasted image 20250121210043.png | 1100]]
![[Pasted image 20250122181340.png | 1100]]

修改完后的源码如下：
```cpp
#include "InlineCurrentSense.h"
//#include "communication/SimpleFOCDebug.h"

// 定义一个宏，将SIMPLEFOC_DEBUG替换为SEGGER_RTT_printf
#define SIMPLEFOC_DEBUG(fmt, ...) SEGGER_RTT_printf(0, fmt "\n", ##__VA_ARGS__)

// InlineCurrentSensor constructor
//  - shunt_resistor  - shunt resistor value
//  - gain  - current-sense op-amp gain
//  - phA   - A phase adc pin
//  - phB   - B phase adc pin
//  - phC   - C phase adc pin (optional)
InlineCurrentSense::InlineCurrentSense(float _shunt_resistor, float _gain, int _pinA, int _pinB, int _pinC){
    pinA = _pinA;
    pinB = _pinB;
    pinC = _pinC;

    shunt_resistor = _shunt_resistor;
    amp_gain  = _gain;
    volts_to_amps_ratio = 1.0f /_shunt_resistor / _gain; // volts to amps
    // gains for each phase
    gain_a = volts_to_amps_ratio;
    gain_b = -volts_to_amps_ratio; // 这里从simpleFOC源码改为-的原因是电路板的采样电阻上ADC采样端口（IN+与IN-）是反的。
    gain_c = volts_to_amps_ratio;
};


InlineCurrentSense::InlineCurrentSense(float _mVpA, int _pinA, int _pinB, int _pinC){
    pinA = _pinA;
    pinB = _pinB;
    pinC = _pinC;

    volts_to_amps_ratio = 1000.0f / _mVpA; // mV to amps
    // gains for each phase
    gain_a = volts_to_amps_ratio;
    gain_b = -volts_to_amps_ratio;
    gain_c = volts_to_amps_ratio;
};



// Inline sensor init function
int InlineCurrentSense::init(){
    // if no linked driver its fine in this case 
    // at least for init()
    //void* drv_params = driver ? driver->params : nullptr;
    // configure ADC variables
    //params = _configureADCInline(drv_params,pinA,pinB,pinC);
    // if init failed return fail
    //if (params == SIMPLEFOC_CURRENT_SENSE_INIT_FAILED) return 0; 
    // calibrate zero offsets
    calibrateOffsets();
    // set the initialized flag
    initialized = 1;
    // return success
    return 1;
}
// Function finding zero offsets of the ADC
void InlineCurrentSense::calibrateOffsets(){
    const int calibration_rounds = 1000;
    
    // find adc offset = zero current voltage
    offset_ia = 0;
    offset_ib = 0;
    offset_ic = 0;
    // read the adc voltage 1000 times ( arbitrary number )
    for (int i = 0; i < calibration_rounds; i++) {
        ADC_StartReadVoltageFromChannels(); // 每一次都要启动一次ADC采样流程
        if(_isset(pinA)) offset_ia += _readADCVoltageInline(pinA);
        if(_isset(pinB)) offset_ib += _readADCVoltageInline(pinB);
        if(_isset(pinC)) offset_ic += _readADCVoltageInline(pinC);
        _delay(1);
    }
    // calculate the mean offsets
    if(_isset(pinA)) offset_ia = offset_ia / calibration_rounds;
    if(_isset(pinB)) offset_ib = offset_ib / calibration_rounds;
    if(_isset(pinC)) offset_ic = offset_ic / calibration_rounds;
}

// read all three phase currents (if possible 2 or 3)
PhaseCurrent_s InlineCurrentSense::getPhaseCurrents(){
    PhaseCurrent_s current;
    ADC_StartReadVoltageFromChannels(); // 每一次都要启动一次ADC采样流程
    current.a = (_readADCVoltageInline(pinA) - offset_ia) * gain_a;// amps
    current.b = (_readADCVoltageInline(pinB) - offset_ib) * gain_b;// amps
    current.c = (!_isset(pinC)) ? 0 : (_readADCVoltageInline(pinC) - offset_ic)*gain_c; // amps
    return current;
}

// Function aligning the current sense with motor driver
// if all pins are connected well none of this is really necessary! - can be avoided
// returns flag
// 0 - fail
// 1 - success and nothing changed
// 2 - success but pins reconfigured
// 3 - success but gains inverted
// 4 - success but pins reconfigured and gains inverted
int InlineCurrentSense::driverAlign(float voltage){
    
    int exit_flag = 1;
    if(skip_align) return exit_flag;

    if (driver==nullptr) {
        SIMPLEFOC_DEBUG("CUR: No driver linked!");
        return 0;
    }

    if (!initialized) return 0;

    if(_isset(pinA)){
        // set phase A active and phases B and C down
        driver->setPwm(voltage, 0, 0);
        _delay(2000);
        PhaseCurrent_s c = getPhaseCurrents();
        // read the current 100 times ( arbitrary number )
        for (int i = 0; i < 100; i++) {
            PhaseCurrent_s c1 = getPhaseCurrents();
            c.a = c.a*0.6f + 0.4f*c1.a;
            c.b = c.b*0.6f + 0.4f*c1.b;
            c.c = c.c*0.6f + 0.4f*c1.c;
            _delay(3);
        }
        driver->setPwm(0, 0, 0);
        // align phase A
        float ab_ratio = c.b ? fabs(c.a / c.b) : 0;
        float ac_ratio = c.c ? fabs(c.a / c.c) : 0;
        if(_isset(pinB) && ab_ratio > 1.5f ){ // should be ~2
            gain_a *= _sign(c.a);
        }else if(_isset(pinC) && ac_ratio > 1.5f ){ // should be ~2
            gain_a *= _sign(c.a);
        }else if(_isset(pinB) && ab_ratio < 0.7f ){ // should be ~0.5
            // switch phase A and B
            int tmp_pinA = pinA;
            pinA = pinB;
            pinB = tmp_pinA;
            float tmp_offsetA = offset_ia;
            offset_ia = offset_ib;
            offset_ib = tmp_offsetA;
            gain_a *= _sign(c.b);
            exit_flag = 2; // signal that pins have been switched
        }else if(_isset(pinC) &&  ac_ratio < 0.7f ){ // should be ~0.5
            // switch phase A and C
            int tmp_pinA = pinA;
            pinA = pinC;
            pinC= tmp_pinA;
            float tmp_offsetA = offset_ia;
            offset_ia = offset_ic;
            offset_ic = tmp_offsetA;
            gain_a *= _sign(c.c);
            exit_flag = 2;// signal that pins have been switched
        }else{
            // error in current sense - phase either not measured or bad connection
            return 0;
        }
    }

    if(_isset(pinB)){
        // set phase B active and phases A and C down
        driver->setPwm(0, voltage, 0);
        _delay(200);
        PhaseCurrent_s c = getPhaseCurrents();
        // read the current 50 times
        for (int i = 0; i < 100; i++) {
            PhaseCurrent_s c1 = getPhaseCurrents();
            c.a = c.a*0.6 + 0.4f*c1.a;
            c.b = c.b*0.6 + 0.4f*c1.b;
            c.c = c.c*0.6 + 0.4f*c1.c;
            _delay(3);
        }
        driver->setPwm(0, 0, 0);
        float ba_ratio = c.a ? fabs(c.b / c.a) : 0;
        float bc_ratio = c.c ? fabs(c.b / c.c) : 0;
        if(_isset(pinA) && ba_ratio > 1.5f ){ // should be ~2
            gain_b *= _sign(c.b);
        }else if(_isset(pinC) && bc_ratio > 1.5f ){ // should be ~2
            gain_b *= _sign(c.b);
        }else if(_isset(pinA) && ba_ratio < 0.7f ){ // it should be ~0.5
            // switch phase A and B
            int tmp_pinB = pinB;
            pinB = pinA;
            pinA = tmp_pinB;
            float tmp_offsetB = offset_ib;
            offset_ib = offset_ia;
            offset_ia = tmp_offsetB;
            gain_b *= _sign(c.a);
            exit_flag = 2; // signal that pins have been switched
        }else if(_isset(pinC) && bc_ratio < 0.7f ){ // should be ~0.5
            // switch phase A and C
            int tmp_pinB = pinB;
            pinB = pinC;
            pinC = tmp_pinB;
            float tmp_offsetB = offset_ib;
            offset_ib = offset_ic;
            offset_ic = tmp_offsetB;
            gain_b *= _sign(c.c);
            exit_flag = 2; // signal that pins have been switched
        }else{
            // error in current sense - phase either not measured or bad connection
            return 0;
        }   
    }

    // if phase C measured
    if(_isset(pinC)){
        // set phase C active and phases A and B down
        driver->setPwm(0, 0, voltage);
        _delay(200);
        PhaseCurrent_s c = getPhaseCurrents();
        // read the adc voltage 500 times ( arbitrary number )
        for (int i = 0; i < 100; i++) {
            PhaseCurrent_s c1 = getPhaseCurrents();
            c.a = c.a*0.6 + 0.4f*c1.a;
            c.b = c.b*0.6 + 0.4f*c1.b;
            c.c = c.c*0.6 + 0.4f*c1.c;
            _delay(3);
        }
        driver->setPwm(0, 0, 0);
        float ca_ratio = c.a ? fabs(c.c / c.a) : 0;
        float cb_ratio = c.b ? fabs(c.c / c.b) : 0;
        if(_isset(pinA) && ca_ratio > 1.5f ){ // should be ~2
            gain_c *= _sign(c.c);
        }else if(_isset(pinB) && cb_ratio > 1.5f ){ // should be ~2
            gain_c *= _sign(c.c);
        }else if(_isset(pinA) && ca_ratio < 0.7f ){ // it should be ~0.5
            // switch phase A and C
            int tmp_pinC = pinC;
            pinC = pinA;
            pinA = tmp_pinC;
            float tmp_offsetC = offset_ic;
            offset_ic = offset_ia;
            offset_ia = tmp_offsetC;
            gain_c *= _sign(c.a);
            exit_flag = 2; // signal that pins have been switched
        }else if(_isset(pinB) && cb_ratio < 0.7f ){ // should be ~0.5
            // switch phase B and C
            int tmp_pinC = pinC;
            pinC = pinB;
            pinB = tmp_pinC;
            float tmp_offsetC = offset_ic;
            offset_ic = offset_ib;
            offset_ib = tmp_offsetC;
            gain_c *= _sign(c.b);
            exit_flag = 2; // signal that pins have been switched
        }else{
            // error in current sense - phase either not measured or bad connection
            return 0;
        }   
    }

    if(gain_a < 0 || gain_b < 0 || gain_c < 0) exit_flag +=2;
    // exit flag is either
    // 0 - fail
    // 1 - success and nothing changed
    // 2 - success but pins reconfigured
    // 3 - success but gains inverted
    // 4 - success but pins reconfigured and gains inverted

    return exit_flag;
}

```


## 2.4、修改InlineCurrentSense.h
![[Pasted image 20250121210807.png | 1100]]
修改完后的源码如下：
```cpp
#ifndef INLINE_CS_LIB_H
#define INLINE_CS_LIB_H

//#include "Arduino.h"
#include "main.h"
#include "adc.h"
#include "foc_utils.h"
#include "time_utils.h"
#include "defaults.h"
#include "CurrentSense.h"
#include "lowpass_filter.h"
//#include "hardware_api.h"


class InlineCurrentSense: public CurrentSense{
  public:
    /**
      InlineCurrentSense class constructor
      @param shunt_resistor shunt resistor value
      @param gain current-sense op-amp gain
      @param phA A phase adc pin
      @param phB B phase adc pin
      @param phC C phase adc pin (optional)
    */
    InlineCurrentSense(float shunt_resistor, float gain, int pinA, int pinB, int pinC = NOT_SET);
    /**
      InlineCurrentSense class constructor
      @param mVpA mV per Amp ratio
      @param phA A phase adc pin
      @param phB B phase adc pin
      @param phC C phase adc pin (optional)
    */
    InlineCurrentSense(float mVpA, int pinA, int pinB, int pinC = NOT_SET);

    // CurrentSense interface implementing functions 
    int init() override;
    PhaseCurrent_s getPhaseCurrents() override;
    int driverAlign(float align_voltage) override;

    // ADC measuremnet gain for each phase
    // support for different gains for different phases of more commonly - inverted phase currents
    // this should be automated later
    float gain_a; //!< phase A gain
    float gain_b; //!< phase B gain
    float gain_c; //!< phase C gain

    // // per phase low pass fileters
    // LowPassFilter lpf_a{DEF_LPF_PER_PHASE_CURRENT_SENSE_Tf}; //!<  current A low pass filter
    // LowPassFilter lpf_b{DEF_LPF_PER_PHASE_CURRENT_SENSE_Tf}; //!<  current B low pass filter
    // LowPassFilter lpf_c{DEF_LPF_PER_PHASE_CURRENT_SENSE_Tf}; //!<  current C low pass filter

    float offset_ia; //!< zero current A voltage value (center of the adc reading)
    float offset_ib; //!< zero current B voltage value (center of the adc reading)
    float offset_ic; //!< zero current C voltage value (center of the adc reading)
    
  private:
  
    // hardware variables
  	int pinA; //!< pin A analog pin for current measurement
  	int pinB; //!< pin B analog pin for current measurement
  	int pinC; //!< pin C analog pin for current measurement

    // gain variables
    float shunt_resistor; //!< Shunt resistor value
    float amp_gain; //!< amp gain value
    float volts_to_amps_ratio; //!< Volts to amps ratio
    
    /**
     *  Function finding zero offsets of the ADC
     */
    void calibrateOffsets();

};

#endif
```

## 2.5、user_main.cpp
![[Pasted image 20250122143413.png | 1100]]
![[Pasted image 20250122143738.png | 1100]]
![[Pasted image 20250122144228.png | 1100]]




# 三、细节补充
----
## 3.1、ADC采样时间与转换时间

### 3.1.1、ADC采样时间
来自bilibili：《[[STM32 HAL库][ADC]采样时间和转换时间，最佳教程，没有之一](https://www.bilibili.com/video/BV1XM4m1y7eP/?spm_id_from=333.1391.0.0&vd_source=5a3da2d3c2504fa7535978c724745a9e)》的视频6：45分。
![[Pasted image 20250120174234.png]]
![[Pasted image 20250120174418.png]]
![[Pasted image 20250120174601.png]]
![[Pasted image 20250120174734.png]]
所谓采样时间，就是开关闭合，让模拟信号跑进电容Cadc，接着再断开开关，这个流程的时间。

### 3.1.2、采样时间怎样计算?
![[Pasted image 20250120175057.png]]
其中Radc=1k是MCU的内阻，它是固定的。电容Cadc也是固定的8pF。MCU外部的电阻有锂电池的内阻 + 电机相电阻 + 导线电阻等等。
![[Pasted image 20250120175416.png]]
如上图所示，博主计算了一番：
- 当MCU外面的电阻 = 400欧姆时，如果ADC运行频率 = 14MHz时，那么采样时间至少 = 1.54cycles
- 当MCU外面的电阻 = 10k欧姆时，如果ADC运算频率 = 14MHz时，那么采样时间至少 = 11.9cycles

>一般锂电池的电芯都是m欧姆级别，电机的相电阻一般也才几个欧姆，导线的电阻也很低。所示，就算我们当400欧姆来计算，1.54cycles也有一定的冗余。大部分的人都建议至少用7.5 cycles。
>如果ADC频率是12MHz，采样时间等于0.625us。1 / 12MHz = 83.3333ns、7.5 cycles * 83.333ns = 0.625us。

### 3.1.3、转换时间
![[Pasted image 20250120180154.png | 1100]]
如上所示，STM32F103的中文参考手册的161页有说明，转换时间是固定的12.5cycle。
>注：如果ADC频率是12MHz，转换时间等于1.0417us。 1 / 12MHz = 83.333ns、12.5cycle * 83.333ns = 1.0317us。

### 3.1.4、总时间
如果ADC频率是12MHz时，总时间 = 采样时间（7cycles = 0.625us) + 转换时间（12.5cycles = 1.0317us) = 1.6567us

## 3.2、InlineCurrentSense类实例化，且init()后，其对象的属性变成怎样了？？
![[Pasted image 20250122150859.png | 1100]]
执行完实例化与init()方法，为了就是计算出各相的offset（偏置量，又叫“零电流电压基准”）与volts_to_amps_ratio（电压到电流的转换系数）。

### 3.2.1、为什么需要offset（零电流电压基准）
![[Pasted image 20250122151208.png | 1100]]
**后续在实际测量电流时，就会用这些偏置量来抵消因硬件或采样环境带来的偏移误差，以得到更准确的电流测量结果。** 从函数InlineCurrentSense::calibrateOffsets()看到，它通过多次读取（默认1000次）来求取“零电流电压基准”，把这个值保存为 offset_ia / offset_ib / offset_ic。

## 3.3、使用示波器测量STM32F103运行函数ADC_StartReadVoltageFromChannels()的所需时间
![[20250122-163536.mp4]]
**从视频看到，高电平的持续时间约为133.0us，STM32F103执行函数ADC_StartReadVoltageFromChannels()的总时间是133.0us。**

![[Pasted image 20250122163252.png | 1100]]
如上所示，关闭全局中断的目的是保证MCU在执行函数ADC_StartReadVoltageFromChannels()期间不被打断（不会在中途跑去执行中断回调函数）。
![[Pasted image 20250122163516.png | 1100]]


