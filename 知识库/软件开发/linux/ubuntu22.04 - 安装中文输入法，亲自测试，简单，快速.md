# 前言
---
ubuntu系统安装中文输入法的方法五花八门，每一次重装系统后都要折腾一遍。记录一下这一次安装中文输入法的最简单过程。

# 一、安装并配置 Fcitx5 输入法框架
Fcitx5 是 Ubuntu 22.04 推荐的输入法框架，支持多种中文输入法，配置简单。
1. **安装 Fcitx5** 打开终端，输入以下命令安装 Fcitx5 及其中文输入法引擎（比如拼音）：
```bash
sudo apt update 
sudo apt install fcitx5 fcitx5-config-qt fcitx5-chinese-addons
```

2. **设置 Fcitx5 为默认输入法框架** 安装完成后，打开系统的输入法设置，可以在“语言支持”中将输入法切换为 Fcitx5。
也可以在终端输入以下命令来设置 Fcitx5 为默认输入法框架：
```bash
im-config -n fcitx5
```
3. **重启系统** 更改输入法设置后，重启系统以确保设置生效。
4. **配置 Fcitx5** 重启后，你可以在应用程序列表中找到 **Fcitx5 配置工具**，打开后添加或调整中文输入法，比如选择拼音或双拼等输入方式。

# 二、修改切换输入法的快捷键
---
![[Pasted image 20241029164555.png]]
如上所示，进入配置界面。

![[Pasted image 20241029164713.png]]
找到全局选项，我喜欢将中英切换的快捷键设置为**左shifg**，跟Windows系统的类似。

![[Pasted image 20241029165058.png]]
可以输入中文了。


# 三、使用zsh(oh my zsh)后，fcitx5中文输入法不正常了
---
从bash终端切换到zsh终端，安装oh my zsh后，突然发现中文输入法不管用了。经过几番折腾，原因是fcitx5没有被启动。解决方法：
1. 将fcitx5的环境变量加入~/.zshrc的文件里
2. 配置fcitx5开机自启。

## 3.1、将fcitx5的环境变量加入~/.zshrc的文件
在终端运行：
![[Pasted image 20241118190815.png]]
然后在最下面的地方填入：
```zsh
export XMODIFIERS="@im=fcitx" 
export GTK_IM_MODULE=fcitx 
export QT_IM_MODULE=fcitx
```
接着，保存。
![[Pasted image 20241118190741.png]]

## 3.2、配置fcitx5开机自启
**创建自启动文件：** 使用如下命令在 `~/.config/autostart` 目录下创建 Fcitx5 的自启动配置文件：
```zsh
mkdir -p ~/.config/autostart
nano ~/.config/autostart/fcitx5.desktop
```

**在文件中输入以下内容：**
```zsh
[Desktop Entry]
Type=Application
Exec=fcitx5
Hidden=false
NoDisplay=false
X-GNOME-Autostart-enabled=true
Name=Fcitx5
Comment=Start Fcitx5 input method framework

```
-**保存并退出：** 按下 `Ctrl + O` 保存文件，按 `Ctrl + X` 退出编辑器。
 **验证启动效果：** 重启系统后，`fcitx5` 应会自动启动。
 