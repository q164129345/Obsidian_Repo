# 导言
---
[[STM32 - RTT组件06 - J-Scope数据可视化，使用RTT方式#4.3、时间戳问题]] ，提到的时间戳问题，当消息包不包含时间戳时，J-Scope的横坐标的默认间隔是100us。
![[Pasted image 20241216170819.png | 800]]
基于官方的时间戳例程，实践一下。效果如下：
![[J-Scope_RTT_dwt.gif]]

项目源码：https://github.com/q164129345/MCU_Develop/tree/main/jlink_scope_rtt_dwt_timestamp

# 一、代码
---
## 1.1、tim.c
![[Pasted image 20241216171633.png | 800]]
如上图所示，定时器14的中断周期改为1ms。

## 1.2、main.c
![[Pasted image 20241216190232.png | 1200]]
![[Pasted image 20241216190418.png | 1200]]

# 二、观察时间戳与数据
---
![[Pasted image 20241216190518.png]]
![[Pasted image 20241216190540.png]]
从上面两图可以看到，正弦波的周期是100ms。因为定时器的中断周期是1ms，数据长度是100。波形很准。
![[Pasted image 20241216190704.png]]
![[Pasted image 20241216190727.png]]
从上两图可以看到，变量每隔1ms翻转一次，也很准确。

# 三、细节补充
---
## 3.1、消息包的类型，注意一下是4个字节
![[Pasted image 20241216190758.png]]
如上图所示，刚开始，msg1与msg2的变量类型用uint8_t，目的是减少消息包的字节数，毕竟msg1与msg2的值的范围在0 ～ 255。但是，J-Scope的波形显示不正常。后来，我改用uint32_t后，J-Scope的波形显示正常了。
