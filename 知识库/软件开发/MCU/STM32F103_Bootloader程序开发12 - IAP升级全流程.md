# 导言
---
《[[STM32F103_Bootloader程序开发11 - 实现 App 安全跳转至 Bootloader]]》上一章节实现App跳转bootloader，接着，跳转到bootloader后，要回复‘C'给IAP上位机，表示我准备好接收固件数据。

![[Pasted image 20250701202532.png]]

**项目地址：**
*   **Gitee (国内推荐)**: [https://gitee.com/wallace89/MCU_Develop/tree/main/bootloader11_stm32f103_app_jump_boot](https://gitee.com/wallace89/MCU_Develop/tree/main/bootloader11_stm32f103_app_jump_boot)
*   **GitHub**: [https://github.com/q164129345/MCU_Develop/tree/main/bootloader11_stm32f103_app_jump_boot](https://github.com/q164129345/MCU_Develop/tree/main/bootloader11_stm32f103_app_jump_boot)

# 一、代码
---
## 1.1、main.c
![[Pasted image 20250701202740.png | 1100]]
![[Pasted image 20250701202832.png | 1100]]
![[Pasted image 20250701202950.png | 1100]]

# 二、测试IAP升级全流程
---
## 2.1、正常IAP流程
![[iap_update1.gif]]如上图所示，在iap_py文件夹下使用指令`python3 main.py --port COM8 --baud 115200 --file .\firmware\App_crc.bin --auto-jump`启动IAP升级全流程。最后，IAP升级完毕，顺利跳转App程序运行。

## 2.2、异常IAP流程
![[iap_update2.gif]]
如上所示，在IAP升级中途，我强制退出IAP程序。接着，bootloader在通讯倒计时结束后，跳转回之前的App程序。控制板并没有变成“砖头”，等待下一次的IAP升级请求！

