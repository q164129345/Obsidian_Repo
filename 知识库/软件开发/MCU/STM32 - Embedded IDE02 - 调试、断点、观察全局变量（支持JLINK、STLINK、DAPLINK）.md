# 导言
---
[[STM32 - Embedded IDE01 - vscode搭建Embedded IDE开发环境（支持JLINK、STLINK、DAPLINK）]]
![[Pasted image 20241219154223.png]]
继续使用安富莱的开发板。
在vscode上实现程序的调试、打断点、观察全局变量确实要折腾一番。有一些开发者跳过了这个功能，使用串口打印log来调试程序。但是，我认为打断点，观察全局变量还是很有效的调试手段，不能放弃。所以，决定折腾vscode调试、打断点、观察全局变量。

# 一、Cortex-Debug插件
---
![[Pasted image 20241219170641.png | 1200]]
调试，打断点，观察全局变量依赖这个插件。

# 二、创建launch.json
---
![[Pasted image 20241219171952.png]]
![[Pasted image 20241219171909.png]]
![[Pasted image 20241219172019.png]]
![[Pasted image 20241219172154.png]]
按照上面的步骤，创建了launch.json文件。

# 三、安装芯片支持包
---
![[Pasted image 20241219172423.png]]
![[Pasted image 20241219172456.png]]
如上所示，找到STM32F4xx的芯片支持包，下载，安装它。
![[Pasted image 20241219172634.png]]
![[Pasted image 20241219172935.png]]

# 四、修改launch.json
模版如下：
```json
{
    "configurations": [
        {
            "cwd": "${workspaceRoot}",
            "executable": ".\\\\build\\\\rt_thread_vcu\\\\rt_thread_vcu.elf",  // 找到.elf文件的位置
            "name": "Debug with OpenOCD",
            "request": "launch",
            "type": "cortex-debug",     
            "servertype": "openocd",
            "configFiles": [
                "interface/stlink-v2.cfg",
                "target/stm32f4x.cfg"
            ],
            "searchDir": [],
            "runToEntryPoint": "main",
            "showDevDebugOutput": "none",
            "svdFile": ".\\\\.pack\\\\GigaDevice\\\\GD32F4xx_DFP.2.0.0\\\\SVD\\\\GD32F4xx.svd", // 找到.svd文件的位置
            // 实时观察全局变量的开关,不需要实时观察全局变量的话，就不需要这段代码
            "liveWatch": {
                "enabled": true,
                "samplesPerSecond": 4
            }
        },
    ]
}
```

## 4.1、J-LINK版本的launch.json
```json
{
    "configurations": [
        {
            "cwd": "${workspaceRoot}",
            "executable": "./build/F407IGT6-BASE/F407IGT6-BASE.elf",  // 找到.elf文件的位置
            "name": "Debug",
            "request": "launch",
            "type": "cortex-debug",
            "servertype": "jlink",
            "device": "STM32F407IG", // JLINK必须要指定芯片型号
            // "configFiles": [
            //     "interface/stlink-v2.cfg",
            //     "target/stm32f4x.cfg"
            // ],
            "searchDir": [],
            "runToEntryPoint": "main",
            "showDevDebugOutput": "none",
            "svdFile": ".pack/Keil/STM32F4xx_DFP.2.14.0/CMSIS/SVD/STM32F40x.svd", // 找到.svd文件的位置
            // 实时观察全局变量的开关,不需要实时观察全局变量的话，就不需要这段代码
            "liveWatch": {
                "enabled": true,
                "samplesPerSecond": 4
            }
        },
    ]
}
```
如上所示，使用J-LINK版本的launch.json。
### 4.1.1、excutable
![[Pasted image 20241219174332.png]]
### 4.1.2、svdFile
![[Pasted image 20241219174503.png]]
### 4.1.3、其他
![[Pasted image 20241219174836.png]]
### 4.1.4、开始调试
![[jlink-debug.gif]]

## 4.2、ST-LINK版本的launch.json
```json
{
    "configurations": [
        {
            "cwd": "${workspaceRoot}",
            "executable": "./build/F407IGT6-BASE/F407IGT6-BASE.elf",  // 找到.elf文件的位置
            "name": "Debug",
            "request": "launch",
            "type": "cortex-debug",
            "servertype": "openocd",
            //"device": "STM32F407IG", // JLINK必须要指定芯片型号
            "configFiles": [
                "interface/stlink-v2.cfg",
                "target/stm32f4x.cfg"
            ],
            "searchDir": [],
            "runToEntryPoint": "main",
            "showDevDebugOutput": "none",
            "svdFile": ".pack/Keil/STM32F4xx_DFP.2.14.0/CMSIS/SVD/STM32F40x.svd", // 找到.svd文件的位置
            // 实时观察全局变量的开关,不需要实时观察全局变量的话，就不需要这段代码
            "liveWatch": {
                "enabled": true,
                "samplesPerSecond": 4
            }
        },
    ]
}
```
![[Pasted image 20241219180202.png]]
### 4.2.1、开始调试
![[stlink-debug.gif]]

## 4.3、DAP-LINK版本的launch.json
```json
{
    "configurations": [
        {
            "cwd": "${workspaceRoot}",
            "executable": "./build/F407IGT6-BASE/F407IGT6-BASE.elf",  // 找到.elf文件的位置
            "name": "Debug",
            "request": "launch",
            "type": "cortex-debug",
            "servertype": "openocd",
            //"device": "STM32F407IG", // JLINK必须要指定芯片型号
            "configFiles": [
                "interface/cmsis-dap.cfg",
                "target/stm32f4x.cfg"
            ],
            "searchDir": [],
            "runToEntryPoint": "main",
            "showDevDebugOutput": "none",
            "svdFile": ".pack/Keil/STM32F4xx_DFP.2.14.0/CMSIS/SVD/STM32F40x.svd", // 找到.svd文件的位置
            // 实时观察全局变量的开关,不需要实时观察全局变量的话，就不需要这段代码
            "liveWatch": {
                "enabled": true,
                "samplesPerSecond": 4
            }
        },
    ]
}
```
![[Pasted image 20241219180547.png]]
### 4.3.1、开始调试
![[dap-debug.gif]]