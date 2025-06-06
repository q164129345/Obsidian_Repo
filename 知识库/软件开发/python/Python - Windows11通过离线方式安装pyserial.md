# 导言
---
因公司网络访问的限制，没办法使用pip install pyserial去轻松地安装pyserial库。

打开网页：https://pypi.org/project/pyserial/#files

## 下载.whl
![[Pasted image 20250605204721.png | 1100]]

## cmd命令行
![[Pasted image 20250605205125.png | 1100]]
```shell
pip install .\pyserial-3.5-py2.py3-none-any.whl

python -m serial.tools.list_ports
```