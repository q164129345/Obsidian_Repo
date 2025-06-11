# 导言
---
使用Python3开发一个上位机，目的是给MCU进行IAP升级。

![[Pasted image 20250610203354.png]]
如上所示，整个IAP上位机的框架比较简单。

# 一、代码
---
## bin_reader.py
```python
import os

class BinReader:
    """
    Brief: 二进制文件读取器，负责读取固件文件并提供按字节访问的方法
    """
    def __init__(self):
        self.file_data = None
        self.file_size = 0
        self.file_path = ""
    
    def load_file(self, file_path):
        """
        Brief: 加载二进制文件到内存
        Params:
            file_path: 文件路径
        Return:
            bool: 是否加载成功
        """
        if not os.path.exists(file_path): # 读取文件之前，先确认文件存在，避免程序崩溃
            print(f"错误：文件 {file_path} 不存在")
            return False
        
        try:
            with open(file_path, 'rb') as f: # 用只读、二进制方式打开文件
                self.file_data = f.read()    # read()返回的是bytes对象，它是不可变的字节序列
            
            self.file_size = len(self.file_data) # 获取bytes对象的长度
            self.file_path = file_path           # 保存固件的路径
            print(f"成功加载文件: {file_path}, 大小: {self.file_size} 字节")
            return True
        except Exception as e: # 通用的异常处理方式
            print(f"加载文件时出错: {str(e)}")
            self.file_data = None
            self.file_size = 0
            self.file_path = ""
            return False
    
    def get_file_size(self):
        """
        Brief: 获取文件大小
        Return:
            int: 文件大小(字节)
        """
        return self.file_size
    
    def get_data(self, start_pos, length):
        """
        Brief: 获取指定位置和长度的二进制数据
        Params:
            start_pos: 起始位置(字节偏移，从0开始)
            length: 要读取的字节数
        Return:
            bytes: 读取的二进制数据，如果失败则返回None
        """
        if self.file_data is None:
            print("错误：未加载文件")
            return None
        
        if start_pos < 0 or start_pos >= self.file_size:
            print(f"错误：起始位置 {start_pos} 超出文件范围(0-{self.file_size-1})")
            return None
        
        end_pos = min(start_pos + length, self.file_size) # 防止读取超出文件范围
        actual_length = end_pos - start_pos
        
        if actual_length < length:
            print(f"警告：请求的长度超出文件范围，只返回 {actual_length} 字节")
        
        return self.file_data[start_pos:end_pos]

# Python特殊机制，
# 1、允许bin_reader.py作为独立程序运行（如python bin_reader.py)；
# 2、其他模块导入bin_reader.py时，这段代码将被忽略。
if __name__ == "__main__":
    # 示例用法
    reader = BinReader()
    
    # 加载文件
    if reader.load_file("firmware/App_crc.bin"):
        # 获取文件大小
        print(f"文件大小: {reader.get_file_size()} 字节")
        
        # 获取特定位置的数据
        start_pos = 512  # 起始地址
        length = 512     # 长度
        data = reader.get_data(start_pos, length)
        if data:
            print(f"成功读取数据，长度: {len(data)} 字节")
            # 以0x格式打印前10个字节，并包含起始地址(完整32位格式)
            hex_bytes = [f"0x{b:02X}" for b in data[:10]]
            print(f"起始地址0x{start_pos:08X}，数据前10个字节: {' '.join(hex_bytes)}")

```

# 二、测试代码
---
![[Pasted image 20250610204111.png]]
![[Pasted image 20250610204357.png]]
如上所示，bin_reader.py读取到的内容跟实际的App_crc.bin的内容一致。

# 三、细节补充
---
## 3.1、if \_\_name__ == "\_\_main\_\_":
![[Pasted image 20250610204711.png]]

不会影响其他模块调用。这段代码使用了 if __name__ == "__main__": 条件判断，这是 Python 的一个特殊机制：
1. 当直接运行 bin_reader.py 文件时（如 python3 bin_reader.py），__name__ 变量的值为 "__main__"，所以这段代码会执行。
2. 当从其他模块导入 bin_reader.py 时（如 from bin_reader import BinReader），__name__ 变量的值为 "bin_reader"，所以这段代码不会执行。

**这种设计模式在 Python 中很常见，它允许一个模块既可以作为库被导入，也可以作为独立程序运行。** 这段代码只是作为示例，展示如何使用 BinReader 类，不会对其他模块调用产生任何影响。


