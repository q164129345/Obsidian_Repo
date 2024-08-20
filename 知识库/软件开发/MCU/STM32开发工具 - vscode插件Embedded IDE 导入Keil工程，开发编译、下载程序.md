# 一、前言
---
跟别的团队开发一款新的割草机器人，经过讨论后使用STM32CubeMX + vscode来开发VCU程序。
vscode插件Embedded IDE能导入Keil工程进行开发。
这套开发MCU的方法，需要安装如下工具才能完成编译、下载、debug程序：
- STM32CubeMX
- STM32CubeIDE
- STM32CubePorgrammer
- Keil
![[Pasted image 20240820140203.png]]
# 二、开发流程
---
## 2.1、STM32CubeMX生成Keil工程代码
![[Pasted image 20240820140332.png]]

Keil工程编译通过。然后，可以关闭Keil软件，打开vscode继续Embedded IDE导入Keil项目。
![[Pasted image 20240820140408.png]]

## 2.2、vscode Embedded IDE
### 2.2.1、安装实用工具
如下图所示，安装需要的工具。
![[Pasted image 20240820142140.png]]
![[Pasted image 20240820140458.png]]
### 2.2.2、打开插件配置
![[Pasted image 20240820140517.png]]
### 2.2.3、导入项目
![[Pasted image 20240820140536.png]]
![[Pasted image 20240820140550.png]]
如下图所示，成功导入Keil工程。
![[Pasted image 20240820140611.png]]
点击Build之后，可以成功编译代码。
![[Pasted image 20240820140652.png]]
### 2.2.4、构建配置（重要）
本次实验的芯片是GD32F407VET，CPU类型属于Cortex-M4，并支持浮点。
![[Pasted image 20240820140734.png]]
![[Pasted image 20240820140745.png]]
![[Pasted image 20240820140755.png]]
![[Pasted image 20240820140806.png]]
取消勾选“不生成Hex/Bin文件后，再编译一次工程。提示生成Hex、Bin等文件。
![[Pasted image 20240820140851.png]]
### 2.2.5、烧录配置
烧录，根据手上的烧录器来选择即可，我使用ST-LINK V2。
![[Pasted image 20240820140939.png]]
### 2.2.6、下载程序
如下图所示，可以正常下载程序了！！！
![[Pasted image 20240820141005.png]]