# 导言
---
[[STM32开发工具 - vscode插件 Cortex-Debug + ST-LINK与DAP-LINK与J-LINK调试MCU程序]]
在上一篇备忘录介绍使用OpenOCD的stlink-v2.cfg接口下载固件，在STM32F407的项目上很顺利。但是，在stm32f103上会出现下载失败。
在网上搜了一下，这位博主的文章解决了这个问题: https://www.iotword.com/26738.html

# 一、修改stm32f1x.cfg
---
首先，找到stm32f1x.cfg文件。
![[Pasted image 20241018173858.png]]

修改第44行代码，改为0x2ba01477，保存！！！如下图所示：
![[Pasted image 20241018174017.png]]

如下图所示，下载成功！！
![[Pasted image 20241018174153.png]]
