# 导言
---
J-LINK RTT Viewer不需要USART，只需SWD接口就可以实现高速调试信息打印（J-LINK RTT Viewer），数据可视化（J-Scope），且移植过程非常简单。
<div class="tip">
<strong>提示</strong><br><br>参考安富莱教程《专题教程第5期：工程调试利器SEGGER的RTT组件，替代串口调试(V1.1)》，网址：https://www.armbbs.cn/forum.php?mod=viewthread&tid=86177&highlight=RTT
</div>

开发板：
![[Pasted image 20241209161818.png | 800]]
![[rtt_Viewer1.gif | 900]]
如上所示，在J-LINK RTT Viewer软件上打印调试log，分别在终端0、终端1、终端2上。
代码github:https://github.com/q164129345/MCU_Develop/tree/main/jlink_rtt_viewer
J-LINK驱动下载:https://www.segger.com/products/debug-probes/j-link/tools/rtt-viewer/
官方文档:https://wiki.segger.com/RTT
# 一、代码
---
## 1.1、main.h
![[Pasted image 20241206164450.png]]
在main.h添加`#include "SEGGER_RTT.h"`。

## 1.2、main.c
![[Pasted image 20241206164610.png]]
![[Pasted image 20241206164759.png | 1000]]

# 二、编译、烧录
---
![[Pasted image 20241206165042.png]]
如上图所示，0 Error，0 Warning.

# 三、J-LINK RTT Viewer
---
![[Pasted image 20241206165417.png]]
如上所示，打开RTT Viewer后，按照以上配置即可，点击OK即可查看Log。
![[Pasted image 20241206165525.png]]

# 四、细节补充
---
## 4.1、SEGGER_RTT_WriteString()与SEGGER_RTT_printf()的区别是什么？
**优先使用 `SEGGER_RTT_WriteString()`：简单场景下效率更高，减少系统开销。在需要动态内容时选择 `SEGGER_RTT_printf()`，但要注意控制格式化内容的复杂度，避免不必要的性能损失。**

### SEGGER_RTT_WriteString()
功能
- 用于将一个**固定字符串**直接写入 RTT 通道。
- 不支持格式化操作，只能发送纯字符串。
优点
- 效率高：因为不需要格式化处理，直接将字符串写入缓冲区。
- 代码简单：只需传递字符串指针，操作简单直观。
用法
```c
SEGGER_RTT_WriteString(0, "Hello, RTT!\n");
```
适用场景
- 输出简单的调试信息（如固定文本或日志标记）。
- 不需要动态内容或复杂的格式化处理。

### SEGGER_RTT_printf()
功能
- 类似于标准库的 `printf()`，支持**格式化字符串**。
- 允许将变量值嵌入字符串中动态输出。
优点
- 灵活性高：支持格式化占位符（如 `%d`, `%s`, `%f` 等），适合需要动态内容的情况。
- 功能强大：能够生成复杂的输出内容。
用法
```c
int value = 42;
SEGGER_RTT_printf(0, "The answer is %d\n", value);
```
适用场景
- 需要输出动态数据或变量值。
- 需要格式化显示内容，例如对齐、填充、精度控制等。

## 4.2、log字体颜色设置
**只有SEGGER_RTT_WriteString()支持修改字体颜色，SEGGER_RTT_printf()不支持修改字体颜色**
摘自:https://wiki.segger.com/RTT
![[Pasted image 20241206172104.png | 900]]
![[Pasted image 20241206172705.png]]
如上所示。在代码SEGGER_RTT.h里能找到这些字体颜色选项。

```c
// 用法
SEGGER_RTT_WriteString(0,RTT_CTRL_TEXT_RED"Hello,World\n"); // 红色字体
SEGGER_RTT_WriteString(0,RTT_CTRL_TEXT_YELLOW"Hello,World\n"); // 黄色字体
SEGGER_RTT_WriteString(0,RTT_CTRL_TEXT_BLUE"Hello,World\n");  // 蓝色字体
SEGGER_RTT_WriteString(0,RTT_CTRL_TEXT_GREEN"Hello,World\n"); // 绿色字体
```