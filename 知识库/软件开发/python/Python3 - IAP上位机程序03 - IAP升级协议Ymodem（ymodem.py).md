# 导言
---
在嵌入式系统的固件升级场景中，**Ymodem协议**以其可靠性和简洁性成为串口传输的常用选择，尤其适用于资源受限的MCU平台。
![[Pasted image 20250615092826.png]]

# 一、Ymodem协议
---
## 1.1、帧结构
![[Pasted image 20250614194647.png]]
Ymodem协议的帧总长度由包头决定：
1. 包头=SOH(0x01)时，帧长度是128bytes + 5bytes（包头+包号+包号取反+CRC16）。
2. 同理，包头=STX(0x02)时，帧长度是1024bytes + 5bytes。

Ymodem协议的设计，有点像一个物流系统，可以选用不同尺寸的箱子来寄送货物。
- SOH (0x01): 代表一个“小箱子”，能装 128 字节的货物。
- STX (0x02): 代表一个“大箱子”，能装 1024 字节的货物。

在一次完整的“寄件”（文件传输）过程中，发送方和接收方可以协商决定主要使用哪种箱子。
1. `灵活策略（混合使用大小箱子）:`
发送方可以根据每批“货物”（数据块）的多少，来动态选择用大箱子还是小箱子。比如，大部分时候用1024字节的STX大箱子，但最后一批货物只有80字节，就可以换成128字节的SOH小箱子来装，这样更节省空间（传输带宽）。这是完全符合YMODEM协议规范的。
2. `简单策略（只用一种大箱子）`- (我的代码目前采用的策略):
类比: 为了简化流程，物流公司决定，无论货物多少，一律使用1024字节的STX大箱子。如果货物装不满，就用填充物（比如泡沫或旧报纸）把箱子塞满。

## 1.2、起始帧
![[Pasted image 20250614200339.png]]
我的IAP上位机统一使用STX，统一使用1024bytes的“大箱子”。另外，起始帧规定剩余的字节使用0x00填充。
"App_crc.bin"是升级的固件名称，"5284"是App_crc.bin的固件大小，表示5284bytes。

## 1.3、数据帧
![[Pasted image 20250614200727.png]]
包号从0x01开始，0x02、0x03.....直到，最后一帧数据。
包号取反即从0xFE开始，0xFD，0xFC....直到，最后一帧数据。

值得注意的是，最后一帧肯定没有1024bytes，剩余的内容使用0x1A填充（Ymodem协议规定）。

> 起始帧与结束帧使用0x00填充，数据帧使用0x1A填充。

## 1.4、结束帧
![[Pasted image 20250614201741.png]]
结束帧跟起始帧用一样的包号与包号取反，然后，内容全部用0x00填充。上位机发送结束帧，告诉下位机IAP升级正式结束。

## 1.5、其他功能帧
- ACK - 十六进制0x06 - 正确收到并确认(下位机回应上位机)
- C     - 十六进制0x43 - 开始接收文件（下位机告诉上位机）
- NAK - 十六进制0x15 - 数据错误，请重发(下位机要求上位机)
- CAN - 十六进制0x18 - 强制取消传输（下位机要求上位机，一般连续发送两次）
- EOT - 十六进制0x04 - 文件数据发送结束（上位机告诉下位机）

## 1.6、升级流程
![[Pasted image 20250624095636.png]]
**掌握‘C’字符在YModem协议中的作用，才能更好地理解YModem协议。** 简单来说，'C' 字符在YModem协议中，是下位机（接收方）用来“催促”上位机（发送方）进行下一步操作的信号，意思是“我准备好了，请继续”。例如：“收到EOT（文件传输结束）信号后”：
- 场景: 文件的所有数据包都已成功传输完毕。上位机发送一个EOT (End of Transmission) 字符，表示这个文件传完了。
- 目的: 这也是一个两段式的交流，正是我们之前修复的重点：
	1. 发送 ACK: 确认“我收到了你发的EOT信号”。
	2. 发送 C: 这是为了告诉上位机“好的，这个文件我收完了。现在我准备好接收下一个文件了。请把下一个文件的信息包（第0包）发给我。如果你没有更多文件了，就请发送一个空的结束包。”

# 二、代码
---
## 2.1、ymodem.py
```python
# -*- coding: utf-8 -*-
# Tested with Python 3.13.3

import os
import struct
from bin_reader import BinReader

class YModem:
    """
    Brief: YModem协议实现，专注于数据包的构建与解析
    """
    # YModem协议常量
    SOH = 0x01  # 128字节数据包开始标记
    STX = 0x02  # 1024字节数据包开始标记
    EOT = 0x04  # 传输结束标记
    ACK = 0x06  # 应答，接收正确
    NAK = 0x15  # 否定应答，接收错误
    CAN = 0x18  # 取消传输
    C = 0x43    # ASCII 'C'，表示CRC校验方式

    # 响应状态码
    STATUS_OK = 0       # 成功
    STATUS_NAK = 1      # 需要重传
    STATUS_CANCEL = 2   # 传输取消
    STATUS_TIMEOUT = 3  # 超时
    STATUS_SUCCESS = 4  # 升级成功

    # 数据包大小
    PACKET_SIZE_128 = 128
    PACKET_SIZE_1024 = 1024
    
    def __init__(self):
        """
        Brief: 初始化YModem协议处理器
        """
        self.bin_reader = BinReader()
    
    def calculate_crc(self, data):
        """
        Brief: 计算CRC-16校验值
        Params:
            data: 要计算校验值的字节数据
        Return:
            int: 16位CRC校验值
        """
        # CRC-16/XMODEM算法
        crc = 0x0000
        for byte in data:
            crc ^= (byte << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ 0x1021) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc
    
    def build_packet(self, packet_num, data):
        """
        Brief: 构建YModem数据包
        Params:
            packet_num: 数据包序号(0-255)
            data: 要发送的数据
        Return:
            bytes: 构建好的数据包
        """
        # 统一使用STX作为包头，数据包大小始终为1024字节
        packet_header = self.STX
        packet_size = self.PACKET_SIZE_1024
        
        # 如果数据不足，根据包类型选择填充字符
        if len(data) < packet_size:
            # 第0包（起始帧）用0x00填充，其他包用0x1A填充
            padding_byte = b'\x00' if packet_num == 0 else b'\x1A'
            data = data + padding_byte * (packet_size - len(data))
        elif len(data) > packet_size:
            data = data[:packet_size]  # 截断到指定大小
        
        # 构建数据包：起始标记 + 包序号 + 包序号取反 + 数据 + CRC校验
        packet = struct.pack('>B', packet_header)  # 起始标记(统一使用STX)
        packet += struct.pack('>B', packet_num)    # 包序号
        packet += struct.pack('>B', 255 - packet_num)  # 包序号取反
        packet += data  # 数据部分
        
        # 计算并添加CRC校验
        crc = self.calculate_crc(data)
        packet += struct.pack('>H', crc)  # 两字节CRC校验值
        
        return packet
    
    def parse_response(self, response_byte):
        """
        Brief: 解析接收方的响应
        Params:
            response_byte: 接收到的响应字节
        Return:
            int: 状态码
        """
        if response_byte == self.ACK:
            return self.STATUS_OK
        elif response_byte == self.NAK:
            return self.STATUS_NAK
        elif response_byte == self.CAN:
            return self.STATUS_CANCEL
        elif response_byte == self.EOT:
            return self.STATUS_SUCCESS
        else:
            # 其他字节可能是特定的成功指示等
            return self.STATUS_TIMEOUT
    
    def build_end_packet(self):
        """
        Brief: 构建YModem的结束帧,表示传输结束
        Return:
            bytes: 结束帧数据包
        """
        # 发送空文件名数据包，表示传输结束
        return self.build_packet(0, b'\0')

# 当直接运行ymodem.py时执行以下代码
if __name__ == "__main__":
    # 创建YModem实例和BinReader实例
    ymodem = YModem()
    bin_file_path = "firmware/App_crc.bin"
    
    # 加载二进制文件
    if not ymodem.bin_reader.load_file(bin_file_path):
        print(f"错误：无法加载文件 {bin_file_path}")
        exit(1)
    
    # 获取文件信息
    file_size = ymodem.bin_reader.get_file_size()
    file_name = os.path.basename(bin_file_path)
    
    # 打印文件信息
    print(f"\n文件信息：{file_name}, 大小：{file_size} 字节")
    
    # 创建第0包(文件信息包)
    file_info = f"{file_name}\0{file_size}\0".encode('utf-8')
    packet0 = ymodem.build_packet(0, file_info)
    
    # 创建第1包(数据包)，从文件中读取前1024字节
    data_chunk = ymodem.bin_reader.get_data(0, ymodem.PACKET_SIZE_1024)
    packet1 = ymodem.build_packet(1, data_chunk)
    
    # 计算最后一包的位置和大小
    last_packet_offset = (file_size // ymodem.PACKET_SIZE_1024) * ymodem.PACKET_SIZE_1024
    last_packet_size = file_size - last_packet_offset
    
    # 如果文件大小正好是1024的倍数，最后一包就是最后1024字节
    if last_packet_size == 0 and file_size > 0:
        last_packet_offset = file_size - ymodem.PACKET_SIZE_1024
        last_packet_size = ymodem.PACKET_SIZE_1024
    
    # 创建最后一包
    last_packet_num = (last_packet_offset // ymodem.PACKET_SIZE_1024) + 1
    last_data_chunk = ymodem.bin_reader.get_data(last_packet_offset, last_packet_size)
    last_packet = ymodem.build_packet(last_packet_num, last_data_chunk)
    
    # 打印第0包(文件信息包)
    print("第0包(文件信息包):")
    hex_packet0 = " ".join([f"{b:02X}" for b in packet0])
    print(hex_packet0)
    print(f"第0包长度: {len(packet0)-5} 字节数据 + 5 字节头尾 = {len(packet0)} 字节")  # 减去包头(1)+序号(1)+序号取反(1)+CRC(2)=5字节
    
    # 打印第1包(数据包)
    print("\n第1包(数据包):")
    hex_packet1 = " ".join([f"{b:02X}" for b in packet1])
    print(hex_packet1)
    print(f"第1包长度: {len(packet1)-5} 字节数据 + 5 字节头尾 = {len(packet1)} 字节")
    
    # 打印最后一包(数据包)
    print(f"\n最后一包(第{last_packet_num}包):")
    print(f"偏移量: {last_packet_offset}, 实际数据大小: {last_packet_size} 字节")
    hex_last_packet = " ".join([f"{b:02X}" for b in last_packet])
    print(hex_last_packet)
    print(f"最后一包长度: {len(last_packet)-5} 字节数据 + 5 字节头尾 = {len(last_packet)} 字节")
    
    # 打印最后一包数据的填充情况
    if last_packet_size < ymodem.PACKET_SIZE_1024:
        padding_start = min(20, last_packet_size)  # 显示最多20个字节的实际数据
        padding_bytes = min(20, ymodem.PACKET_SIZE_1024 - last_packet_size)  # 显示最多20个填充字节
        
        # 显示最后一包的部分实际数据
        actual_data = " ".join([f"{b:02X}" for b in last_data_chunk[-padding_start:]])
        print(f"\n最后一包末尾{padding_start}个实际数据字节: {actual_data}")
        
        # 显示填充部分的开始
        padding_preview = " ".join([f"{b:02X}" for b in last_packet[3+last_packet_size:3+last_packet_size+padding_bytes]])
        print(f"填充部分开始{padding_bytes}个字节(应为0x1A): {padding_preview}")
    
    # 打印结束帧
    packet_end = ymodem.build_end_packet()
    print(f"\n结束帧:")
    hex_packet_end = " ".join([f"{b:02X}" for b in packet_end])
    print(hex_packet_end)
    print(f"结束帧长度: {len(packet_end)-5} 字节数据 + 5 字节头尾 = {len(packet_end)} 字节")


```

![[Pasted image 20250615095520.png]]
![[Pasted image 20250615095759.png]]
![[Pasted image 20250615115111.png]]
![[Pasted image 20250615115416.png]]

# 三、测试代码
---
![[ymodem_test.gif]]
如上所示，在项目目录下执行指令`python3 ymodem.py`，运行ymodem.py最后的`if __name__ == "__main__":`的代码块，测试发送的消息对不对。

## 3.1、分析起始帧
![[Pasted image 20250615120311.png]]
如上所示，起始帧的格式正确。

## 3.2、分析数据帧
### 3.2.1、第1包数据帧
![[Pasted image 20250615124018.png]]
如上所示，是第1包数据。

![[Pasted image 20250615174506.png]]
如上所示，Python程序使用Ymodem协议发出来的二进制文件跟vscode直接打开的二进制文件App_crc.bin的内容一致。必须一致，否则IAP升级后，MCU肯定会跑不起来，变成砖。


### 3.2.2、最后1包数据帧
![[Pasted image 20250615124307.png]]
![[Pasted image 20250615175351.png]]
如上所示，从最后1包的最后几十个字节看来，没问题！

## 3.3、分析结束帧
![[Pasted image 20250615180519.png]]
结束帧除了前三个字节外，其他所有字节都是0x00。







