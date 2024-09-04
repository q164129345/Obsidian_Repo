## 导言
---
export.sh的文件夹位置：esp/v5.2.1/esp-idf/export.sh
目前，乐鑫官方比较推荐vscode +ESP-IDF的组合开发环境。据说，目前官方开发的Espressif_IDE的软件bug比较多，所以不太推荐使用。
跟着正点原子的视频与文档来搭建vscode + ESP-IDF开发环境比较舒服且顺利的。但是，正点原子只介绍了Window系统的搭建。在ubuntu系统上搭建的话，坑还是有一点点的。比如，需要手动添加idf.py到系统环境变量。
首先，必须看官方文档：[https://docs.espressif.com/projects/esp-idf/zh_CN/v5.1/esp32s3/get-started/linux-macos-setup.html#get-started-set-up-tools](https://docs.espressif.com/projects/esp-idf/zh_CN/v5.1/esp32s3/get-started/linux-macos-setup.html#get-started-set-up-tools)
![[Pasted image 20240904195933.png]]
如上图所示，第四步：设置环境变量。
![[Pasted image 20240904200006.png]]


## 一、怎样确认export.sh脚本没有添加到系统环境变量？
---
![[Pasted image 20240904200044.png]]
如上图所示，[终端不认识idf.py](http://xn--idf-628dv56r32eq7z1ha.py)，表明export.sh脚本没有添加到系统环境变量。

## 二、将esp/v5.2.1/esp-idf/export.sh脚本添加到环境变量
---
如上图所示，先找到esp-idf的目录，找到export.sh的位置。我的ubuntu位置是/home/wallace/esp/v5.2.1/esp-idf。
所以最终要将`/home/wallace/esp/v5.2.1/esp-idf/export.sh`告诉终端。
<aside> 💡 我使用zsh终端，ubuntu默认是bash终端

</aside>

以下是如何将`export.sh`脚本添加到`zsh`配置的详细步骤：
1. 打开终端。
2. 使用文本编辑器打开`.zshrc`文件。这个文件位于你的用户主目录下。你可以使用`nano`或其他编辑器，如：
    ```bash
    nano ~/.zshrc
    ```
3. 在文件的末尾添加以下行：
    ```bash
    source /home/wallace/esp/v5.2.1/esp-idf/export.
    ```

这一行命令的作用是让`zsh`在每次启动时自动执行该脚本，从而加载你的环境变量设置。
4. 保存并关闭编辑器。如果你使用的是`nano`，可以按`Ctrl+O`来保存文件，然后按`Ctrl+X`退出。
5. 为了使改动立即生效，你可以通过执行以下命令重新加载`.zshrc`：
    ```bash
    source ~/.zshrc
    ```
这样设置之后，无论何时你打开一个新的终端窗口，`/home/wallace/esp/v5.2.1/esp-idf/export.sh`脚本就会自动执行，它设置的环境变量也会立即生效。这是管理和自动化环境变量配置的一个非常方便的方法。
最终，我的~/.zshrc文件如下：
![[Pasted image 20240904200204.png]]
最后，使用终端输入`idf.py`的效果如下：
![[Pasted image 20240904200221.png]]
## 三、ubuntu默认终端bash添加环境变量
---
简单介绍一下bash终端设置环境变量的流程：
1. 打开终端。
2. 使用文本编辑器打开`.bashrc`文件。这个文件位于用户的主目录下。你可以使用`nano`或`gedit`等编辑器，如：
    ```bash
    nano ~/.bashrc
    ```
3. 在文件的末尾添加以下行：
    ```
    source /home/wallace/esp/v5.2.1/esp-idf/export.sh
    ```
4. 保存并关闭编辑器。如果你使用的是`nano`，可以按`Ctrl+O`保存更改，然后按`Ctrl+X`退出。
5. 为了使改动立即生效，可以执行以下命令来重新加载`.bashrc`：
    ```bash
    source ~/.bashrc
    ```
完成以上步骤后，每次你打开一个新的终端窗口时，`export.sh`脚本将自动执行，其设置的环境变量也将可用。这是一个方便的方式来确保环境变量的设置在每个会话中都是一致的。