# 导言
---
![[Pasted image 20250611114030.png]]
上一章节完成bin_reader.py，成功地读取二进制升级固件（在firmware/App_crc.bin），本章节开始梳理serial_manager.py的串口驱动代码。

![[Pasted image 20250611114426.png]]
上位机读取升级固件后（比如App_crc.bin），将App_crc.bin分包发送给MCU（可以使用串口、网口等，每包的大小可以是256bytes、或者512bytes等）。MCU的bootloader程序将App_crc.bin接收完成后，一般都会对整个固件进行校验，确保固件的完整性（避免在通讯的过程中，有可能通讯收到干扰，二进制被莫名其妙地修改）。固件完整性没有问题后，MCU会将App_crc.bin复制到App区完成IAP升级，执行新的程序App_crc.bin。


# 一、代码
---
## 1.1、serial_manager.py
```python
import serial
import sys
import time

class SerialManager:
    """
    Brief: 串口管理类，负责串口初始化和数据收发
    Params: port 串口号，如COM3
            baudrate 波特率
    """
    def __init__(self, port, baudrate):
        """
        Brief: 初始化串口
        Params: port - 串口号，如COM3
                baudrate - 波特率
        """
        self.ser = serial.Serial(port, baudrate, timeout=1)

    def Send_String(self, data):
        """
        Brief: 发送字符串数据
        Params: data - 要发送的字符串
        Return: 实际发送的字节数
        """
        return self.ser.write(data.encode('utf-8'))
    
    def Send_Bytes(self, data):
        """
        Brief: 发送字节数据
        Params: data - 要发送的bytes对象
        Return: 实际发送的字节数
        """
        return self.ser.write(data)

    def close(self):
        """
        Brief: 关闭串口
        """
        self.ser.close()

# 当直接运行serial_manager.py时执行以下代码
# 其他模块导入serial_mnnager.py时，这段代码将被忽略。
if __name__ == "__main__":
    # 检查命令行参数
    if len(sys.argv) < 3:
        print("用法: python3 serial_manager.py <串口> <波特率>")
        print("示例: python3 serial_manager.py COM3 115200")
        sys.exit(1)
    
    # 从命令行获取串口和波特率
    port = sys.argv[1]
    baudrate = int(sys.argv[2])
    
    try:
        # 创建SerialManager实例
        print(f"正在连接串口 {port}，波特率 {baudrate}...")
        serial_mgr = SerialManager(port, baudrate)
        
        # 发送测试字符串
        test_str = "Hello,World!"
        print(f"发送字符串: '{test_str}'")
        bytes_sent = serial_mgr.Send_String(test_str)
        print(f"成功发送 {bytes_sent} 字节")

        # 等待一秒，确保数据发送完成
        time.sleep(1)
        
        # 关闭串口
        serial_mgr.close()
        print("串口已关闭")
        
    except Exception as e:
        print(f"错误: {str(e)}")
        sys.exit(1)
```

# 二、测试代码
---
![[serial_manager.gif]]
1. 首先，在我的电脑环境里，使用com0com软件虚拟了两个串口通道COM6与COM7。
2. 执行命令`python3 serial_manager.py COM6 115200`初始化串口使用COM6发送一个字符串"Hello,World!"。
3. 从SSCOM软件上成功收到字符串“Hello,World!"，证明串口驱动调试完成。

# 三、细节补充
---
