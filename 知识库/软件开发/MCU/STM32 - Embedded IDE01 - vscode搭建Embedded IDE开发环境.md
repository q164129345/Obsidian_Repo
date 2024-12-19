# 导言
---
Embedded IDE官网:https://em-ide.com/docs/intro
我猜肯定有部分人使用SI + Keil开发STM32项目，也有vscode + Keil开发STM32程序。SI或vscode编写代码，然后切换Keil编译、下载、调试程序。有一段时间，我也是这么干的。但是，程序切换来，切换去，把我整得特别难受，最终还是忍受着远古开发工具的环境，纯Keil开发。在参与新的项目开发时，有人向我推荐一款vscode插件Embedded IDE。说很好用，务必尝试一下。
经过一段时间的使用，最终我决定后续所有的项目开发都将使用vscode + Embedded IDE插件 + Keil编译器（需要安装Keil软件）。废话不多说，开始实践一遍。

![[Pasted image 20241219154223.png]]
本次实验使用安富莱的开发板 + J-LINK。

# 一、CubeMX
---
![[Pasted image 20241219154438.png]]
如上图所示，生成Keil工程。
![[Pasted image 20241219154602.png]]
说白了，就是一个Keil工程。


# 二、Keil
---
![[Pasted image 20241219154746.png]]
![[Pasted image 20241219154810.png]]
如上图所示，简单配置一下Keil。
![[Pasted image 20241219154929.png]]
接着，简单写一个心跳灯程序，烧录到开发板试试，确保Keil工程没有问题。
![[Pasted image 20241219155033.png]]
编译成功。
![[Pasted image 20241219155111.png]]
烧录程序也成功。

# 三、vscode
---
## 3.1、安装vscode插件
![[Pasted image 20241219155827.png]]
如上图所示，安装两个重要的插件。
![[Pasted image 20241219155927.png]]
安装完插件后，进入EIDE插件，如上图所示。
## 3.2、EIDE环境配置
### 3.2.1、安装实用工具
![[Pasted image 20241219160215.png]]
如上所示，安装built-in里的软件。
![[Pasted image 20241219160315.png]]
如上图所示，external只安装两个软件。
## 3.3、打开插件配置
![[Pasted image 20241219160617.png]]
如上所示，告诉EIDE，Keil的编译器所在位置。另外，记得勾选Axf To Elf，否则没办法烧录程序。我之前漏了这里，折腾了很久很久。

## 3.4、导入Keil项目
![[Pasted image 20241219160927.png]]
![[Pasted image 20241219161023.png]]
如上所示，进行Keil项目导入。
![[Pasted image 20241219161103.png]]
选择Yes，将EIDE的项目跟Keil项目放在一起，方便后续分别打开EIDE项目或者Keil项目。
![[Pasted image 20241219161429.png]]
如上所示，刚才在Keil编写的代码还在。
![[Pasted image 20241219161601.png]]
尝试一下编译，编译成功并且生成hex、s19、bin文件。此时，还不能下载程序到开发板！！

## 3.5、构建配置
![[Pasted image 20241219161954.png]]
![[Pasted image 20241219162049.png]]
如上图所示，构建配置里的构建器选项，跟Keil软件的魔术棒里的配置一样，真的是从Keil导入进来的！！！！

# 3.6、烧录配置（下载程序）
### 3.6.1、J-LINK
![[Pasted image 20241219162404.png]]
![[Pasted image 20241219162655.png]]




### 3.6.2、ST-LINKv2
方法一：
![[Pasted image 20241219163353.png]]
如上所示，ST-LINK下载程序ok，ST-LINK不需要关心芯片的信号。

方法二（OpenOCD）：
![[Pasted image 20241219164340.png]]

### 3.6.3、DAP-LINK
![[Pasted image 20241219163939.png]]