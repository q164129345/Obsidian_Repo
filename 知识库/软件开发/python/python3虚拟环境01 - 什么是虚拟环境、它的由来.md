# 导言
---
python3真的不是安装了就完事，必须理解虚拟环境是什么才算是初步掌握python环境。学习python3虚拟环境，建议参考B站视频:https://www.bilibili.com/video/BV1V7411n7CM/?spm_id_from=333.1391.0.0&vd_source=5a3da2d3c2504fa7535978c724745a9e
这位博主用了6个视频，每一个视频仅仅几分钟。居然把python3的虚拟环境讲明白了。

![[152526673-1-208.mp4]]

虚拟环境（Virtual Environment）是Python中的一个独立的、隔离的Python运行环境。它就像是在您的计算机上创建了一个"小房间"，这个房间里有：
- 独立的Python解释器（python.exe)
- 独立的包管理系统(pip.exe)
- 独立的依赖库(site-packages)

# 一、虚拟环境的由来？
---
![[152526685-1-208.mp4]]
视频里，作者讲述了python虚拟环境的由来。


# 二、MacOS下的python虚拟环境
---
最近开始启动Qt6 Pyside6 + QML开发应用程序。通过Qt开发环境，跟着视频学习python虚拟环境。

## 2.1、虚拟环境的结构
![[Pasted image 20241107001115.png]]
如上图所示，在MacOS，Qt6的python虚拟环境的目录结构发生了变化，Scripts文件夹被替换成bin文件夹。接着，多了一个文件pyvenv.cfg与include文件夹。

![[Pasted image 20241107001508.png]]
如上图所示，使用Qt_Creator创建的一个Qt6的QML + Pyside6项目，项目名paho_mqtt。虚拟环境目录下包含：
1. bin/
2. include/
3. lib/
4. pyvenv.cfg


# 三、为什么需要虚拟环境？
----
## 3.1. 项目隔离
- 不同项目可能需要同一个包的不同版本
- 例如：项目A需要Django 2.2，项目B需要Django 3.0
- 虚拟环境让每个项目都有自己的"小房间"，互不影响

## 3.2 依赖管理
- 方便管理每个项目的依赖包
- 可以轻松导出项目依赖列表（requirements.txt）
- 在其他环境快速重建相同的开发环境

## 3.3. 避免冲突
- 防止全局环境被污染
- 避免不同项目之间的包版本冲突
- 保持系统Python环境的清洁

## 3.4. 项目部署
- 便于确定项目的真实依赖
- 方便在不同机器上部署项目
- 提高项目的可移植性

## 3.5.实际应用场景
假设您正在同时开发两个项目：
1. 一个旧项目使用Python 3.7和Django 2.2
2. 一个新项目使用Python 3.9和Django 3.2




