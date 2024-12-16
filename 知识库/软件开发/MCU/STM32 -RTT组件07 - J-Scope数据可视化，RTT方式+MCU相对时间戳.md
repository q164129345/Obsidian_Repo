# 导言
---
[[STM32 - RTT组件06 - J-Scope数据可视化，使用RTT方式#4.3、时间戳问题]] ，提到的时间戳问题，当消息包不包含时间戳时，J-Scope的横坐标的默认间隔是100us。
![[Pasted image 20241216170819.png | 800]]
基于官方的时间戳例程，实践一下。效果如下：
![[J-Scope_RTT_timestamp.gif]]

项目源码：https://github.com/q164129345/MCU_Develop/tree/main/jlink_scope_rtt_timestamp

# 一、代码
---
## 1.1、tim.c
![[Pasted image 20241216171633.png | 800]]
如上图所示，定时器14的中断周期改为1ms。

## 1.2、main.c
![[Pasted image 20241216172749.png | 1200]]
![[Pasted image 20241216173056.png | 1200]]
![[Pasted image 20241216173415.png | 1200]]
# 二、细节补充
---
## 2.1、消息包的类型，注意一下是4个字节
![[Pasted image 20241216173622.png | 1200]]
如上图所示，刚开始，msg1与msg2的变量类型用uint8_t，目的是减少消息包的字节数，毕竟msg1与msg2的值的范围在0 ～ 255。但是，J-Scope的波形显示不正常。后来，我改用uint32_t后，J-Scope的波形显示正常了。
