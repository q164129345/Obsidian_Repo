## 导言
---
![[Pasted image 20240904202835.png]]
按照正点原子的esp32工程文件架构确实变得比较简单，components/BSP/下只需要一个CMakeLists.txt即可。

## 一、set(src_dirs …) 与set(include_dirs…)
---
![[Pasted image 20240904202851.png]]
CMakeLists.txt的set(src_dirs…)与set(include_dirs…)的内容是工程根目录/components/BSP/文件夹下的各个文件夹名称。

## 二、set(requirs…)
---
![[Pasted image 20240904202907.png]]
实际上，这里也是文件夹的名字。这里是官方的组件源码，工程需要使用#include “esp_timer.h”时，必须添加组建esp_timer进去，否则编译器会报“找不到该头文件。”
比如，使用esp_now协议时，要用到wifi模块，此时就需要添加esp_wifi的组建。