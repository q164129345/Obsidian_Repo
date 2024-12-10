# 导言
---
RTT Viewer既然代替串口打印与输入字符串，果然也可以接入rt_thread console。在rt_thread的串口console前提下，实现起来非常简单，只需要修改函数rt_hw_console_output()与函数rt_hw_console_getchar()即可。实际效果如下：
![[rt_thread console.gif | 1000]]
开发板 + JLink：
![[Pasted image 20241209161818.png | 800]]

项目源码:https://github.com/q164129345/MCU_Develop/tree/main/jlink_rtt_viewer_rtconsole
rt_thread官方文档:https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/application-note/debug/seggerRTT/segger_rtt

# 一、CubeMX
---
参考rt_thread官方文档完成rt_thread nano版本的移植，教程真的很详细。网址:https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-nano/nano-port-cube/an0041-nano-port-cube

# 二、代码
---
SEGGER_RTT的移植参考[[STM32 - RTT组件01 - 移植]]
## 2.1、rtconfig.h
![[Pasted image 20241210172310.png]]
解开备注，否则编译会报错。

## 2.2、main.h
![[Pasted image 20241210172511.png]]
## 2.3、main.c
![[Pasted image 20241210173537.png]]
## 2.4、board.c
![[Pasted image 20241210174604.png]]
![[Pasted image 20241210174650.png]]
两个函数的源码如下：
```c
void rt_hw_console_output(const char *str)
{
    rt_size_t i = 0, size = 0;
    char a = '\r';

//  __HAL_UNLOCK(&UartHandle);

    size = rt_strlen(str);

//    for (i = 0; i < size; i++)
//    {
//        if (*(str + i) == '\n')
//        {
//            HAL_UART_Transmit(&UartHandle, (uint8_t *)&a, 1, 1);
//        }
//        HAL_UART_Transmit(&UartHandle, (uint8_t *)(str + i), 1, 1);
//    }
    SEGGER_RTT_printf(0, "%s", str);
}

char rt_hw_console_getchar(void)
{
    /* Note: the initial value of ch must < 0 */
    int ch = -1;

//    if (__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_RXNE) != RESET)
//    {
//        ch = UartHandle.Instance->DR & 0xff;
//    }
//    else
//    {
//        rt_thread_mdelay(10);
//    }
    char tempbuffer[2];
    uint8_t NumBytes = 0;
    
    NumBytes = SEGGER_RTT_Read(0, &tempbuffer[0], 1);
    if (NumBytes == 1) {
        ch = tempbuffer[0];
    }

    return ch;
}
```
完成！

# 三、细节补充
---
![[Pasted image 20241210200946.png]]
![[Pasted image 20241210201030.png]]