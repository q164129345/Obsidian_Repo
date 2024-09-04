## 导言
---
windows与macos平台搭建vscode + esp-idf开发环境比较顺利。但是，在ubuntu系统下折腾了较久。其实，也是自己缺乏在ubuntu系统开发的经验所致。
按照以下步骤解决了问题：
1、掌握lsusb指令，确认USB设备能否被ubuntu系统识别出来。
2、使用指令`ls /dev/ttyUSB*`查看设备文件是否正常。
3、为当前用户开放usb设备访问权限（超级重要！！！）
![[Pasted image 20240904194333.png]]
首先，把esp32开发板连接到电脑的USB接口上。

## 一、lsusb
---
**`lsusb`** 是一个在 Linux 系统中，包括 Ubuntu 在内的，用于显示与 USB（通用串行总线）相关的设备信息的命令。这个命令会列出系统中所有 USB 总线和连接到它们上的设备。它对于诊断连接到你的计算机上的 USB 设备的问题非常有用，也可以帮助你获取设备的详细信息，例如厂商和产品ID，设备类别等。

使用 **`lsusb`** 命令时，它会提供一系列已连接的 USB 设备，以及每个设备的信息，包括但不限于：
- **Bus**：USB 设备连接的总线编号。
- **Device**：设备在该总线上的编号。
- **ID**：设备的唯一标识符，包含了供应商ID和产品ID（以供应商ID:产品ID的形式显示）。
- **Class**、**Subclass**、**Protocol**：定义了设备的类型和它使用的通信协议。
- **Manufacturer**：制造商名称。
- **Product**：产品名称。
- **Serial Number**：设备的序列号。

**`lsusb`** 的一些常用选项包括：
- **`v`** 或 **`-verbose`**：显示关于 USB 设备的详细信息。
- **`t`**：以树状图形式显示设备，这有助于理解设备之间的连接关系。
- **`s [bus]:[devnum]`**：只显示指定总线和设备编号的信息。
- **`d [vendor]:[product]`**：只显示具有指定供应商和产品ID的设备信息。
![[Pasted image 20240904194404.png]]
使用lsubs指令后，能看到CH340的串口设备。

## 二、ls /dev/ttyUSB*
---
命令 **`ls /dev/ttyUSB*`** 在 Linux 系统中，包括 Ubuntu，被用来列出所有连接到系统的 USB 串行设备。这些设备通常是通过 USB 接口连接的外部硬件，如调制解调器、USB 串行转换器或其他串行通信设备。
当你运行这个命令时，系统会显示 **`/dev`** 目录下所有以 **`ttyUSB`** 开头的设备文件。这些文件代表了系统识别到的串行设备，可以通过它们与这些设备进行通信。每个设备文件对应一个设备，如 **`/dev/ttyUSB0`**、**`/dev/ttyUSB1`** 等，其中数字代表了设备的编号。
例如，如果你连接了一个 USB 到串行适配器到 Ubuntu 系统，该设备可能会被系统识别并分配一个名为 **`/dev/ttyUSB0`** 的设备文件。你可以通过访问这个设备文件与连接的硬件进行通信。
如果命令返回了如下输出：
```bash
/dev/ttyUSB0
/dev/ttyUSB1
```
这意味着系统当前有两个 USB 串行设备被识别和配置。这对于进行串行通信、调试或发送命令到串行设备非常有用。
![[Pasted image 20240904194430.png]]
指令找到一个设备/dev/ttyUSB0。实际上，`HL-340 USB-Serial adapter`对应的就是`/dev/ttyUSB0` 。

## 三、开放ubuntu用户的USB设备访问权限
---
```bash
sudo usermod -a -G dialout wallace  // wallace是我的ubuntu用户名
```
最后，记得使用指令
```bash
sudu reboot // 重启一下系统
```
## 四、测试vscode + esp-idf的监控（Monitor）功能
---
![[Pasted image 20240904194512.png]]
![[Pasted image 20240904194527.png]]
监控跑起来了。。