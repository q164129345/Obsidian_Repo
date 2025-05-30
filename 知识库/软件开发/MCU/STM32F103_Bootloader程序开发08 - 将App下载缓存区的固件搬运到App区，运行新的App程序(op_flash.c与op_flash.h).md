# 导言
---
![[Pasted image 20250530171925.png]]
上一章节使用J-Flash将App_crc.bin下载到咱项目定义的”App下载缓存区“，起始地址0x08040000、大小0x30000。然后，通过soft_crc32.c的函数Calculate_Firmware_CRC32_SW()对App_crc.bin进行CRC32校验。详细请回头看上一章节[[STM32F103_Bootloader程序开发07 - 使用J-Flash将App_crc.bin烧录到App下载缓存区，然后校验CRC32]]。

![[Pasted image 20250530172554.png]]
CRC32校验成功，证明App下载缓冲区里的固件完成性是没有问题的。接着只需要将"App下载缓冲区“里的App_crc.bin搬运到App区，运行新的App程序，完成IAP升级。


# 一、代码
---






