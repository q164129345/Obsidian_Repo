# 导言
---
![[Pasted image 20250703230116.png]]
本教程使用正点原子战舰板开发。

通过Keil的相关配置，可以灵活地修改输出文件的保存路径及文件名称。在Bootloader程序开发过程中，合理配置输出文件对于后续固件升级和自动化脚本处理至关重要。完成路径和文件名配置后，还可以借助Keil自带的fromelf.exe工具，将生成的axf文件转换为bin格式文件，便于后续烧录和升级操作。

# 一、修改生成文件的路径
----
![[Pasted image 20250528164207.png]]
![[Pasted image 20250528164251.png]]
如上所示，在MDK-ARM文件夹里创建Ouputs文件夹，这个文件夹用于存放Keil自动生成的文件。
![[Pasted image 20250528164607.png]]
![[Pasted image 20250528164819.png]]
![[Pasted image 20250528164904.png]]
![[Pasted image 20250528165002.png | 1000]]
![[Pasted image 20250528165058.png | 1000]]

# 二、修改生成的文件名
---
![[Pasted image 20250528165215.png]]
![[Pasted image 20250528165429.png | 1000]]

# 三、让Keil调用fromelf.exe生成升级用的bin文件
----
## 3.1、什么是fromelf.exe？
这是 ARM Keil 自带的一个命令行工具，用于将编译生成的目标文件（axf/elf格式）转换为其他格式（比如 bin、hex）。
![[Pasted image 20250528170718.png | 1000]]

## 3.2、生成App.bin
![[Pasted image 20250528170853.png | 1000]]
![[Pasted image 20250528171042.png | 1000]]
```sh
C:\Keil_v5\ARM\ARMCC\bin\fromelf.exe --bin --output .\Output\App.bin .\Outputs\App.axf
```
意思是：
1. C:\Keil_v5\ARM\ARMCC\bin\fromelf.exe
	- 这是 ARM Keil 自带的一个命令行工具，用于将编译生成的目标文件（axf/elf格式）转换为其他格式（比如 bin、hex）。
2. --bin
	- 这是 fromelf 的参数，意思是将输入的 axf 文件（即你的可执行文件）转换成二进制（.bin）格式。
	- 这种 bin 格式是裸数据，没有任何调试信息，适合直接烧录进 MCU Flash，用于 bootloader、IAP、量产等场景。
3. --output .\Output\App.bin
	- --output 是 fromelf 工具的参数，用来指定输出文件名和路径。
	- .\Output\App.bin 是在当前工程目录下的Output文件夹下生成App.bin二进制文件。
4. .\Outputs\App.axf
	- 指定输入的 axf 文件，它在工程目录下的Outputs文件夹下，名字是App.axf。

![[Pasted image 20250528172042.png]]
如上所示，得到了我们IAP升级需要的App程序的二进制文件App.bin了。