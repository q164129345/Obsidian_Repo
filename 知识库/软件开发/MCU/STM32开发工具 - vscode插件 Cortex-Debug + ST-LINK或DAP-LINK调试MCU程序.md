# 一、前言
---
[[STM32开发工具 - vscode插件Embedded IDE 导入Keil工程，开发编译、下载程序]]
继上一章节完成程序的编译、下载程序之后，本章节将使用另外一个插件Cortex-Debug来进行MCU程序的调试，可以打断点，查看全局变量，实时观察全局变量。
![[Pasted image 20240820142617.png]]
这套开发MCU的方法，需要安装如下工具才能完成编译、下载、debug程序：
- STM32CubeMX
- STM32CubeIDE
- STM32CubePorgrammer
- Keil

# 二、Eembbded IDE
---
## 2.1、烧录配置
上一章节的烧录配置选择ST-LINK。但是，怎样都调不通程序debug。摸索了一下，改为OpenOCD之后，同样可以使用ST-LINK下载并调试程序。
![[Pasted image 20240820142807.png]]
编译正常。
![[Pasted image 20240820142826.png]]
下载程序，正常。
![[Pasted image 20240820142838.png]]
## 2.2、launch.json
这个搞得我非常头疼，总算弄出来了。首先，回到vscode的工作区，找到lunch.json文件。工程一开始好像并没有lanch.json文件，需要在Debug界面那里点击才能生成launch.json。
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
模板如上所示，svdFile、configFiles、executable需要根据自己的工程来修改。
### 2.2.1、svdFile
svdFile在刚创建工程的时候，并没有的。在芯片支持包那里安装即可，如下图所示，我安装了GD32F4xx_DFP的。接着，下面SvdPath就是svdFile的位置。不要慌，就在本工程里面。
![[Pasted image 20240820142918.png]]
![[Pasted image 20240820142939.png]]
![[Pasted image 20240820143000.png]]
### 2.2.2、configFiles
对应上即可。
![[Pasted image 20240820143018.png]]
### 2.2.3、executable
位置对应上
![[Pasted image 20240820143034.png]]
![[Pasted image 20240820143046.png]]

# 三、进入调试
---
## 3.1、ST-LINK
![[Pasted image 20240820143113.png]]
可以实时观察全局变量test的变化。
![[Pasted image 20240820143130.png]]
可以打断点。
![[Pasted image 20240820143154.png]]
## 3.2、DAP-LINK
正点原子的无线DAP-LINK也挺好用的，在调试机器人的时候，比较方便。
![[Pasted image 20240820143220.png]]
在ST-LINK的基础上，改变两个地方即可。
![[Pasted image 20240820143238.png]]

## 3.3、J-LINK
如下图所示，J-LINK需要额外增加以下字段：
- `"device"` - 根据芯片型号填写，我当前用APM32F103CBT6；
![[Pasted image 20241022134834.png]]






