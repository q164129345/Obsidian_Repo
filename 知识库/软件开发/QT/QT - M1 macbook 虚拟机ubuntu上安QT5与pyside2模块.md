# 一、前言
---
苹果电脑换成m1芯片后，芯片的架构从intel改为ARM。intel芯片的电脑使用虚拟机运行ubuntu与m1芯片(ARM架构)的电脑使用虚拟机运行ubuntu所能安装的软件是不一样的。在qt官网下载的.run软件压根就不能被安装。
![[Pasted image 20240918193338.png]]


后来想起m1芯片的macbook跟树莓派4b都属于ARM架构，按照树莓派4b的教程来安装qt5是否可行？我尝试过的结果是可行的。如下图所示，能将pyside2的项目运行起来了。
![[Pasted image 20240918193357.png]]
QT Creator的版本是4.11.0，QT的版本是5.12.8。版本不需要太高，最主要的是兼容性好。
# 二、安装QT5
---
参考这篇文章即可完成QT5的安装。
[https://shumeipai.nxez.com/2020/08/25/qt-development-environment-on-raspberry-pi.html](https://shumeipai.nxez.com/2020/08/25/qt-development-environment-on-raspberry-pi.html)
上面的文章中，最主要是以下5条终端指令安装QT5。
![[Pasted image 20240918193429.png]]
# 三、安装pyside2
---
安装完QT5后，使用qt creator创建一个python window项目。编译，运行python window项目会报错。大概的意思是缺少pyside2模块。
![[Pasted image 20240918193502.png]]

接着，在终端上按顺序执行以下命令完成pyside模块的安装。
```bash
sudo apt install python3-pip
sudo pip3 install PyQt5
sudo apt install python3-pyside2.qt3dcore python3-pyside2.qt3dinput python3-pyside2.qt3dlogic python3-pyside2.qt3drender python3-pyside2.qtcharts python3-pyside2.qt
sudo apt install shiboken2
```

# 四、终端测试
---
![[Pasted image 20240918193522.png]]
