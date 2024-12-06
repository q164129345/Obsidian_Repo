# 导言
---
J-LINK RTT Viewer不需要USART，只需SWD接口就可以实现高速调试信息打印（J-LINK RTT Viewer），数据可视化（J-Scope），且移植过程非常简单。
<div class="tip">
<strong>提示</strong><br><br>参考安富莱教程《专题教程第5期：工程调试利器SEGGER的RTT组件，替代串口调试(V1.1)》，网址：https://www.armbbs.cn/forum.php?mod=viewthread&tid=86177&highlight=RTT
</div>
![[rtt_Viewer1.gif | 900]]
如上所示，在J-LINK RTT Viewer软件上打印调试log，分别在终端0、终端1、终端2上。
代码github:https://github.com/q164129345/MCU_Develop/tree/main/jlink_rtt_viewer
J-LINK驱动下载:https://www.segger.com/products/debug-probes/j-link/tools/rtt-viewer/
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


