# 导言
---
![[Pasted image 20250414195316.png | 600]]
如上所示，本实验的目的是使能CAN接收FIFO1的挂号中断，使用CAN过滤器0与CAN接收FIFO1的组合，接收CAN总线上所有的CAN报文。

![[Pasted image 20250409194119.png]]
如上所示，STM32F103有两个3级深度的接收FIFO。外设CAN想要正常接收CAN报文，必须配置接收FIFO与接收滤波器。两者缺一不可，否则导致无法接收CAN报文。

![[Pasted image 20250409194247.png]]
如上所示，当接收FIFO的3级深度（缓存）都被占满时，将会导致溢出。此时，必须让MCU快读取接收FIFO的邮箱，避免出现溢出。

最终效果如下：
![[LL17_CAN_RX_FIFO1_plus.gif | 1100]]
如上所示，使用CAN分析仪发送5000个CAN报文，发送间隔1ms。从Keil的debug模式观察到，全局变量g_RxCount从0变成5000。没有丢包！

项目地址：
- HAL库：https://github.com/q164129345/MCU_Develop/tree/main/stm32f103_hal_library17_Can_Rec_Interrupt
- 寄存器方式：https://github.com/q164129345/MCU_Develop/tree/main/stm32f103_ll_library17_Can_Rec_Interrupt

# 一、CubeMX
---
CubeMX不主持生成外设CAN的LL库代码，仅支持HAL库。所以，先看看HAL库怎样实现接收CAN报文。

## 1.1、开启接收FIFO1中断
![[Pasted image 20250409194909.png | 800]]
如上所示，只需要在NVIC_Settings里勾选CAN_RX1_interrupt即可开启接收FIFO1中断。另外，也可以选择接收FIFO0中断。二者只能选择其一，不能两个都选择。
![[Pasted image 20250410150527.png | 800]]
如上所示，生成代码。

# 二、代码（HAL库）
---
## 2.1、myCanDrive.c
![[Pasted image 20250410171816.png | 1100]]
![[Pasted image 20250410170415.png | 1100]]
如上所示，函数`CAN_FilterConfig_AllMessages()`配置一个滤波器，并将该滤波器关联到RX_FIFO1。之前说过，要能正常接收CAN报文，需要配置滤波器+FIFO。关于接收滤波器的设置，后面会专门拿一章节来讲。设置好滤波器，可以过滤掉大部分的CAN报文，让MCU减少没必要的中断。现在，为了方便起，让滤波器接收CAN总线上的所有CAN报文。

![[Pasted image 20250410171150.png | 1100]]
如上所示：
1. 函数CAN_Config()里增加滤波器的设置，还有使能接收FIFO1接收中断。
2. 函数HAL_CAN_RxFifo1MsgPendingCallback()是HAL库定义的一个虚函数，所以我们可以重新定义它的内容。

# 三、调试代码
---
![[HAL17_CAN_RX_FIFO1.gif | 1000]]
如上所示，使用CAN分析仪发送50条CAN报文到CAN总线上，发送间隔100ms。然后，在Keil的debug模式下观察到全局变量g_RxCount最后等于50，说明一共接收到50条CAN报文。实验成功！

# 四、寄存器梳理
---
## 4.1、滤波器相关的寄存器
```c
    /* 9. 配置滤波器0，FIFO1 */
    CAN1->FMR |= (1UL << 0);   // 进入滤波器初始化模式
    CAN1->sFilterRegister[0].FR1 = 0x00000000;
    CAN1->sFilterRegister[0].FR2 = 0x00000000;
    CAN1->FFA1R |= (1UL << 0);  // 分配到 FIFO1
    CAN1->FM1R  &= ~(1UL << 0); // 滤波器0使用屏蔽位模式
    CAN1->FA1R  |= (1UL << 0);  // 激活滤波器 0
    CAN1->FMR   &= ~(1UL << 0); // 退出滤波器初始化模式
```
上面代码的功能是配置滤波器0，可以接收所有CAN报文，并关联到接收FIFO1。接下来，一个一个梳理。

### 4.1.1、寄存器CAN_FMR
![[Pasted image 20250410191013.png]]
如上所示，CAN_FMR是控制所有滤波器的模式，配置滤波器之前，要先进入初始化模式。等待配置完成后，需要从初始化模式改为正常模式。
```c
CAN1->FMR |= (1UL << 0);   // 进入滤波器初始化模式
CAN1->FMR   &= ~(1UL << 0); // 退出滤波器初始化模式
```

### 4.1.2、寄存器CAN_FM1R
![[Pasted image 20250410191335.png]]
如上所示，寄存器CAN_FM1R主要控制滤波器的屏蔽模式，分为列表模式与屏蔽模式。
```c
CAN1->FM1R  &= ~(1UL << 0); // 滤波器0使用屏蔽位模式
```

>**屏蔽位模式**
 在屏蔽位模式下，标识符寄存器和屏蔽寄存器一起，指定报文标识符的任何一位，应该按照  “必须匹配”或“不用关心”处理。  
**标识符列表模式**
在标识符列表模式下，屏蔽寄存器也被当作标识符寄存器用。因此，不是采用一个标识符加一个屏蔽位的方式，而是使用2个标识符寄存器。接收报文标识符的每一位都必须跟过滤器标识符  
相同。  

### 4.1.3、寄存器CAN_FiRx
![[Pasted image 20250410192103.png]]
如上所示，寄存器CAN_FiRx与寄存器CAN_FM1R组合起来，实现过滤CAN总线上的CAN报文的作用。后面，将单独用一个章节来举例子，达到只接收CAN总线上的部分CAN报文。
```c
CAN1->sFilterRegister[0].FR1 = 0x00000000;
CAN1->sFilterRegister[0].FR2 = 0x00000000;
```

### 4.1.4、寄存器CAN_FFA1R
![[Pasted image 20250410192523.png]]
如上所示，寄存器CAN_FFA1R的作用是将滤波器x分配到FIFO0或者FIFO1。
```c
CAN1->FFA1R |= (1UL << 0);  // 滤波器0分配到FIFO1
```

### 4.1.5、寄存器CAN_FA1R
![[Pasted image 20250410192720.png]]
如上所示，寄存器CAN_FA1R的作用是激活滤波器x。
```c
CAN1->FA1R  |= (1UL << 0);  // 激活滤波器0
```

## 4.2、开启接收FIFO1消息挂号中断（接收中断）
![[Pasted image 20250410192955.png]]
![[Pasted image 20250410193029.png]]
如上所示，寄存器CAN_IER的FMPIE1置1时，使能FIFO1消息挂号中断。
```c
NVIC_SetPriority(CAN1_RX1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0)); // 设置CAN1_RX1全局中断的中断优先级
NVIC_EnableIRQ(CAN1_RX1_IRQn); // 开启CAN1_RX1的全局中断
CAN1->IER |= CAN_IER_FMPIE1; // 使能接收中断（消息挂号中断）
```

## 4.3、在全局中断CAN1_RX1_IRQHandler()里获取CAN报文的内容
![[Pasted image 20250410202009.png | 1000]]
如上所示，从启动文件看到：
1. 接收FIFO0对应的全局中断函数名是`USB_LP_CAN1_RX0_IRQHandler()`。
2. 接收FIFO1对应的全局中断函数名是`CAN1_RX1_IRQHandler()`。本章节实验使用FIFO1。

![[Pasted image 20250410201715.png]]
如上所示，在全局中断函数CAN1_RX1_IRQHandler()里通过4个寄存器就可以获取CAN报文的详细信息。

### 4.3.1、寄存器CAN_RIxR
![[Pasted image 20250410202406.png]]
如上所示，通过寄存器CAN_RIxR可以得到CAN报文以下内容：
1. 标准帧 还是 拓展帧 ？对应的帧ID是什么？
2. 数据帧 还是 远程帧 ？
```c
uint32_t rx_rir   = CAN1->sFIFOMailBox[1].RIR;   // 接收标识符寄存器

// 3. 判断帧类型并提取ID和远程传输请求（RTR）标志
// IDE位（一般在RIR的第2位）：0表示标准帧，1表示扩展帧
if ((rx_rir & (1UL << 2)) == 0) {  // 标准帧
	rxHeader->IDE   = 0;
	// 标准ID位于RIR的[31:21]，共11位
	rxHeader->StdId = (rx_rir >> 21) & 0x7FF;
	rxHeader->ExtId = 0;
} else {  // 扩展帧
	rxHeader->IDE   = 1;
	// 扩展ID位于RIR的[31:3]，共29位
	rxHeader->ExtId = (rx_rir >> 3) & 0x1FFFFFFF;
	rxHeader->StdId = 0;
}
// RTR位（RIR的第1位）：1表示远程帧，0表示数据帧
rxHeader->RTR = (rx_rir & (1UL << 1)) ? 1 : 0;
```

### 4.3.2、寄存器CAN_RDTxR
![[Pasted image 20250410202806.png]]
如上所示，通过寄存器CAN_RDTxR可以得到CAN报文的以下内容：
1. 时间戳
2. 数据长度
```c
uint32_t rx_rdtr  = CAN1->sFIFOMailBox[1].RDTR;  // 接收数据长度和时间戳寄存器
// 2. 解析报文头
// RDTR寄存器：
//     - [3:0]  表示数据字节数（DLC）
//     - [7:4]  保留（根据实际情况不同，这里我们只关心DLC和时间戳）
//     - [15:8] 表示滤波器匹配索引(FilterMatchIndex)
//     - [31:16] 表示接收时间戳
rxHeader->DLC               = rx_rdtr & 0x0F;
rxHeader->FilterMatchIndex  = (rx_rdtr >> 8) & 0xFF;
rxHeader->Timestamp         = (rx_rdtr >> 16) & 0xFFFF;
```

### 4.3.3、寄存器CAN_RDLxR与CAN_RDHxR
![[Pasted image 20250410203048.png]]
如上所示，通过寄存器CAN_RDLxR可以得到CAN报文的低4个字节的内容。
![[Pasted image 20250410203205.png]]
如上所示，通过寄存器CAN_RDLxR可以得到CAN报文的高4个字节的内容。
```c
uint32_t rx_rdlr  = CAN1->sFIFOMailBox[1].RDLR;  // 接收数据低32位寄存器
uint32_t rx_rdhr  = CAN1->sFIFOMailBox[1].RDHR;  // 接收数据高32位寄存器

// 4. 读取数据部分，根据DLC从RDLR和RDHR中提取数据字节（最多8字节）
uint8_t dlc = rxHeader->DLC;
if (dlc > 8)
	dlc = 8;
for (uint8_t i = 0; i < dlc; i++) {
	if (i < 4) {
		rxData[i] = (uint8_t)((rx_rdlr >> (8 * i)) & 0xFF);
	} else {
		rxData[i] = (uint8_t)((rx_rdhr >> (8 * (i - 4))) & 0xFF);
	}
}
```

### 4.3.4、寄存器CAN_RF1R
![[Pasted image 20250410203633.png]]
如上所示，在中断发生后，需要用软件清0。
```c
CAN1->RF1R |= CAN_RF1R_RFOM1; // 释放FIFO1中的报文：写1到CAN1->RF1R中的RFOM1位，释放该报文，使FIFO1指针前移
```

# 五、代码（寄存器方式）
---
## 5.1、myCanDrive_reg.c
![[Pasted image 20250410204017.png | 1100]]
如上所示，函数CAN_Config()里增加滤波器0的设置、并开启CAN接收FIFO1全局中断。
![[Pasted image 20250410204216.png | 1100]]
![[Pasted image 20250410205203.png | 1100]]
如上所示，两个全局中断函数的定义。全局中断函数`USB_HP_CAN1_TX_IRQHandler()`之前在stm32f1xx_it.c里，我觉得放在myCanDrive_reg更好，所以移过来了。

# 六、调试代码
---
![[LL17_CAN_RX_FIFO1.gif | 1100]]
如上所示，CAN分析仪发送50条CAN报文到CAN总线，发送间隔100ms。从Keil的debug模式观察全局变量g_RxCount等于50。没有丢包，效果跟HAL库一样。但是，寄存器效率肯定比HAL库高得多。

# 七、进一步优化全局中断函数CAN1_RX1_IRQHandler()的代码
----
## 7.1、再谈CAN接收FIFO1寄存器CAN_RF1R
![[Pasted image 20250411100541.png]]
FMP1（FIFO1报文数目），指的是FIFO1邮箱里一共接收到几个CAN报文（存储深度是3，所以最多3个CAN报文）。利用它，就能在全局中断函数CAN1_RX1_IRQHandler()里一次性把所有的CAN报文读取完。

![[Pasted image 20250411101823.png]]
如果你在中断函数中只读取了一个报文就退出，而此时FIFO1中仍残留有其他报文（例如还有一个报文未处理），那么由于CAN外设的中断条件依然成立（FIFO1报文挂号数FMP1不为0），会再一次进入中断。
所以，优化的思路是：
1. 如果在`CAN1_RX1_IRQHandler()`里采用了根据FMP1来循环处理FIFO1中所有报文的方法（即在中断中使用while循环，直到FMP1为0），那么当两个报文都被读取处理后，FIFO1就被清空，中断触发条件就不再满足，此时就不会立即再次触发`CAN1_RX1_IRQHandler()`。只有当之后有新的报文进入FIFO1，且FMP1变为非零时，才会再次触发中断回调。
2. 使用while循环依据FMP1处理所有报文，可以一次性清空FIFO内所有待处理报文，从而避免不必要的重复中断，减轻中断服务函数的负担。但是，值得注意的是，正因为有while循环，所以一定要避免长时间卡在全局中断函数里。所以，在`CAN1_RX1_IRQHandler()`里不能当场解析、处理CAN报文，而是使用例如消息队列(ringbuffer)等数据结构。在全局中断`CAN1_RX1_IRQHandler()`里用while循环将所有CAN报文放入ringbuffer，然后在`main()`主循环while(1)里再从ringbuffer拿出CAN报文，接着解析、处理CAN报文。

## 7.2、优化myCanDrive.c
![[Pasted image 20250411102559.png | 1100]]
如上所示，增加代码`while (CAN1->RF1R & CAN_RF1R_FMP1) { ... }`实现一次把FIFO1的所有CAN报文读取出来。

# 八、细节补充
---
## 8.1、再再谈CAN接收FIFO1寄存器（CAN_RF1R）
![[Pasted image 20250411103722.png]]
如上所示，还有另外两个中断可以使用。其中，FOVR1（FIFO1溢出）非常有用。通过它，就能知道咱们现在的系统是不是接不过来CAN总线的报文。此时，应该调整滤波器，过滤掉系统不关心的CAN报文，避免它们进入FIFO邮箱。下一章节，将用代码实践一遍这个功能。




