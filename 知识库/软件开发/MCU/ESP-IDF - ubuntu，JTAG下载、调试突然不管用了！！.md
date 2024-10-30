## 导言
---
新建一个工程，突然JTAG就不管用了。重启开发板，重启系统也不管用。
输出下面一段log:
```c
Open On-Chip Debugger v0.12.0-esp32-20230921 (2023-09-21-13:41)
Licensed under GNU GPL v2
For bug reports, read
	<http://openocd.org/doc/doxygen/bugs.html>
debug_level: 2
Info : only one transport option; autoselecting 'jtag'
Info : esp_usb_jtag: VID set to 0x303a and PID to 0x1001
Info : esp_usb_jtag: capabilities descriptor set to 0x2000

[OpenOCD]
Info : Listening on port 6666 for tcl connections
Info : Listening on port 4444 for telnet connections

[OpenOCD]
❌ Error: libusb_get_string_descriptor_ascii() failed with -7

[OpenOCD]
❌ /home/wallace/esp/tools/openocd-esp32/v0.12.0-esp32-20230921/openocd-esp32/share/openocd/scripts/target/esp_common.cfg:9: Error: 
at file "/home/wallace/esp/tools/openocd-esp32/v0.12.0-esp32-20230921/openocd-esp32/share/openocd/scripts/target/esp_common.cfg", line 9

[Flash]
Failed to flash (via JTag), due to some unknown error in tcl, please try to relaunch open-ocd
[OpenOCD]
For assistance with OpenOCD errors, please refer to our Troubleshooting FAQ: <https://github.com/espressif/openocd-esp32/wiki/Troubleshooting-FAQ>
[OpenOCD]
OpenOCD Exit with non-zero error code 1
[OpenOCD]
[Stopped] : OpenOCD Server
```

## 一、解决方案1
---
突然想起之前删除baltty之后，突然JTAG下载也是不管用了。然后，我尝试重启brltty之后又可以了！！！
### 重启 `brltty`
对配置进行更改后，你需要重启服务以使更改生效：
```bash
sudo systemctl restart brltty
```
### 验证 `brltty` 是否正常工作
最后，确保 `brltty` 正在正常运行，并且已正确配置你的盲文显示设备。你可以再次检查服务的状态或查看日志来确认：
```bash
systemctl status brltty
```
## 二、解决方案二
---
![[Pasted image 20240904195140.png]]

# 三、解决方案三 - 重启ubuntu