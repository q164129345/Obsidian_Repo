# 导言
---
CAN总线因其高速稳定的数据传输与卓越抗干扰性能，在汽车、机器人及工业自动化中被广泛应用。它采用分布式网络结构，实现多节点间实时通信，确保各控制模块精准协同。在汽车领域，CAN总线连接发动机、制动、车身系统，保障车辆安全；在机器人和工业控制中，传感器与执行器间信息传递迅速，使其成为智能制造与自动化控制不可或缺的重要技术。

遗憾的是CubeMX不支持生成CAN总线的LL库代码。所以，梳理完HAL库的实现方式后，继续梳理寄存器方式的实现。
以下是本章节的效果，开发板每隔100ms往CAN总线发送一个报文。CANID是0x0123，数据帧，标准帧，长度0x08，内容是0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08。
![[LL12_CAN_Send.gif]]

项目地址：
- （HAL库）：https://github.com/q164129345/MCU_Develop/tree/main/stm32f103_hal_library13_Can_Send
- （寄存器方式）：https://github.com/q164129345/MCU_Develop/tree/main/stm32f103_ll_library13_Can_Send
# 一、CubeMX
---
## 1.1、Clock Configuration
![[Pasted image 20250317200635.png | 1100]]

## 1.2、Parameter Settings
![[Pasted image 20250317214148.png | 1100]]
采样点的设置是CAN稳定通讯的前提！
采样点的设置是CAN稳定通讯的前提！
采样点的设置是CAN稳定通讯的前提！
如上所示，配置的重点是4分频、Tq1 = 14Times、Tq2 = 3Times。此时采样点的计算如下：
采样点位置，采样点在同步段后的 Tq1 处，即 1 + 14 = 15 Tq，采样点比例 = 15 / 18 ≈ 83.3%。所以，**Prescaler = 4, Tq1 = 14, Tq2 = 3 这种配置可以精确得到 500 kbps，并且采样点位于约 83.3% 的位置，这是一个较为合理的采样位置，既能保证波特率精确，又能提供足够的采样稳定性**。参考我的另外一篇博文[《CAN总线技术 | 物理层03 - 采样点》](https://blog.csdn.net/wallace89/article/details/119811949)

![[Pasted image 20250317201002.png | 1100]]


# 二、代码（HAL库）
---
## 2.1、main.c
![[Pasted image 20250317194721.png | 1100]]
![[Pasted image 20250317194914.png | 1100]]
如上所示，通过HAL库来实现CAN发送的代码实在简单。

## 2.2、编译、下载代码
![[Pasted image 20250317194940.png | 1100]]
如上所示，编译通过。下载代码到开发板后，效果如下：
![[LL12_CAN_Send 1.gif]]

# 三、梳理CAN发送
---
## 3.1、发送邮箱
CAN发送比CAN接收要简单许多，所以先从简单的CAN发送开始梳理。
![[Pasted image 20250317203852.png]]
![[Pasted image 20250317203951.png | 800]]
如上所示，CAN一共有三个发送邮箱，所以为什么函数`Test_CAN_Send_Msg()`里需要用`HAL_CAN_GetTxMailboxesFreeLevel()`去判断有没有空的发送邮箱，才能将需要发送的CAN报文丢进去发送邮箱。CAN的发送邮箱相当于串口的发送数据寄存器USART_TDR。只是CAN的发送邮箱一共有3个，而串口的发送数据寄存器USART_TDR只有一个。显然，CAN的发送缓存比串口的发送缓存要大得多。
请记住，软件只能将需要发送的CAN消息放到发送邮箱，并不是将CAN消息发送到总线。

## 3.2、发送优先级
![[Pasted image 20250317205358.png]]
如上所示，从《STM32F1参考手册》的章节22.7.1-发送处理看到，CAN消息的发送优先级有两种设置，通过寄存器CAN_MCR的位TXFP置1，开启FIFO模式。值得注意的是，寄存器CAN_MCR的位TXFP默认是置0，即标识符决定发送优先级。另外，标识符越小，优先级越高。
![[Pasted image 20250317205941.png | 800]]

## 3.3、标识符是什么？
如下图所示，标识符就是CANID。
![[Pasted image 20250317205843.png]]

## 3.4、终止发送
![[Pasted image 20250317210640.png]]
总的来说，如果你只是想快速清除发送邮箱，不关注TXOK的状态，那么只需要将ABRQ置1即可。

## 3.5、禁止自动重传模式
![[Pasted image 20250317210809.png]]
**默认是禁止自动重传模式的**，禁止CAN自动重传功能的影响主要体现在以下几点：
1. **消息丢失风险增大**
当一帧CAN报文发送失败（例如由于总线干扰或仲裁错误）时，如果自动重传被禁用，硬件不会重新尝试发送这帧报文，导致该报文直接丢失。此时，如果没有额外的软件层重传机制，就可能影响通信的可靠性。
2. **降低传输延时**
在一些实时性要求高的应用中，自动重传可能会引入额外延时。禁用自动重传可以使得一旦检测到发送失败就立即放弃，从而避免因连续重传而延迟后续消息的发送。
3. **简化错误处理逻辑**
禁用自动重传后，发送操作完成后，邮箱状态总会变为空置（无论是成功发送或发送失败），这对某些系统来说可以简化对邮箱状态的监控和管理。不过，这也意味着应用层需要增加判断和处理发送失败的逻辑，以决定是否进行重发或者采取其他补救措施。
4. **总线负载和资源利用**
自动重传在错误发生时可能导致总线负载增加，尤其在总线故障或者干扰情况下频繁重传会占用大量资源。禁用自动重传可以防止这种情况，但同时需要确保上层有机制去检测和响应通信错误。**总之，禁用自动重传功能是一种在追求低延时和精细控制CAN通信时的取舍，但需要权衡消息可靠性和实时性，通常需要在应用层增加适当的错误检测和补救机制。**

![[Pasted image 20250317211940.png | 800]]

## 3.6、500K波特率，发送8个字节的CAN标准帧消息，需要多长时间？
理论上，我们可以计算标准CAN数据帧的位数，再乘以每位的传输时间。对于标准CAN数据帧（CAN 2.0A），各部分位数通常为：
- 起始位 (SOF)：1位
- 仲裁段：11位标识符 + 1位远程传输请求（RTR） = 12位
- 控制段：6位（包括IDE、保留位以及4位数据长度代码 DLC）
- 数据段：8字节 × 8 = 64位
- CRC段：15位CRC + 1位CRC分隔符 = 16位
- 确认段 (ACK)：2位
- 结束位 (EOF)：7位
总计：1 + 12 + 6 + 64 + 16 + 2 + 7 = 108位

在500k波特率下，每位传输时间为：1 / 500,000 = 2微秒。因此，理论上传输108位的CAN报文所需时间为：108 × 2μs = 216微秒。注意：这只是理论传输时间，实际情况还可能受到位填充、总线仲裁和间隔等因素影响，但理论上就是**216微秒**。

# 四、寄存器梳理
---
## 4.1、配置时钟
### 4.1.1、开启CAN时钟
![[Pasted image 20250318210117.png | 800]]
![[Pasted image 20250318210149.png | 800]]
```c
RCC->APB1ENR |= (1UL << 25UL); // 开启CAN时钟
```

### 4.1.2、开启GPIOA时钟
![[Pasted image 20250318210248.png | 800]]
![[Pasted image 20250318210319.png | 800]]
```c
RCC->APB2ENR |= (1UL << 2UL); // 开启GPIOA时钟，因为CAN使用PA11与PA12端口
```

## 4.2、配置GPIO
### 4.2.1、CAN_REMAP决定使用哪个GPIO口
![[Pasted image 20250318210607.png | 800]]
![[Pasted image 20250318210958.png]]
如上所示，CAN_REMAP默认状态就是00，所以CAN1使用PA11与PA12。如需复用其他GPIO口，按照表格去修改GPIO_REMAP即可。

### 4.2.2、GPIO配置
![[Pasted image 20250318211309.png]]
如上图所示，CAN_TX与CAN_RX有对应的GPIO口模式
```c
    /* 2. 配置PA11(CAN_RX)为上拉输入、PA12(CAN_TX)为复用推挽输出 */
    // PA11: CRH[15:12], MODE=00, CNF=10(上拉输入)
    GPIOA->CRH &= ~(0xF << 12);
    GPIOA->CRH |=  (0x8 << 12);
    GPIOA->ODR |=  (1UL << 11); // 上拉
    // PA12: CRH[19:16], MODE=11(50MHz), CNF=10(复用推挽)
    GPIOA->CRH &= ~(0xF << 16);
    GPIOA->CRH |=  (0xB << 16);
```

## 4.3、配置CAN
### 4.3.1、退出睡眠模式
![[Pasted image 20250319165423.png]]
如上所示，进入初始化之前必须先退出睡眠模式，避免进入初始化模式失败。
```c
if (CAN1->MSR & (1UL << 1)) { // 检查 MSR.SLAK 是否为 1
	CAN1->MCR &= ~(1UL << 1);  // 清除 SLEEP 位
	while (CAN1->MSR & (1UL << 1));  // 等待 MSR.SLAK 变 0
}
```



### 4.3.2、进入初始化模式
![[Pasted image 20250318211833.png]]
![[Pasted image 20250318211915.png]]
![[Pasted image 20250318211939.png]]
```c
/* 3. 进入初始化模式 */
CAN1->MCR |= (1UL << 0);            // 请求进入INIT (MCR.INRQ=1)
while (!(CAN1->MSR & (1UL << 0)));  // 等待INAK=1 (MSR.INAK=1)
```

### 4.3.3、关闭时间触发模式
![[Pasted image 20250318212558.png]]
```c
CAN1->MCR &= ~(1UL << 7);  // 清除TTCM位(时间触发模式）
```

### 4.3.4、自动离线管理模式
![[Pasted image 20250318212744.png]]
```c
CAN1->MCR &= ~(1UL << 6);  // 清除ABOM位（自动离线管理模式）
```

### 4.3.5、自动唤醒
![[Pasted image 20250318212856.png]]
![[Pasted image 20250318212936.png]]
```c
CAN1->MCR &= ~(1UL << 5);  // 清除AWUM位（软件自动唤醒）
```

### 4.3.6、接收FIFO锁定模式
![[Pasted image 20250318213039.png]]
![[Pasted image 20250318213227.png]]
```c
CAN1->MCR &= ~(1UL << 3);  // 清除RFLM位（接收FIFO设置新报文覆盖旧报文）
```

### 4.3.7、发送FIFO优先级
![[Pasted image 20250318213329.png]]
![[Pasted image 20250318213406.png]]
```c
CAN1->MCR &= ~(1UL << 2);  // 清除TXFP位（发送FIFO优先级由标识符来决定）
```

### 4.3.8、禁止报文自动重传
![[Pasted image 20250318213517.png]]
![[Pasted image 20250318213535.png]]
```c
CAN1->MCR |= (1UL << 4); // CAN报文只发送一次
```

### 4.3.9、位时序（重点！！！！）
位时序设置规则参考[《CAN总线技术 | 物理层03 - 采样点》](https://blog.csdn.net/wallace89/article/details/119811949)
![[Pasted image 20250318213924.png]]
如上所示，相当于CubeMX里的Bit Timings Parameters。
![[Pasted image 20250318214215.png]]
![[Pasted image 20250318214255.png]]
```c
    4. 设置BTR=0x002D0003
       - SJW=0  => 1Tq
       - TS2=0x02 => 2 => 3Tq
       - TS1=0x0D => 13 => 14Tq
       - BRP=0x03 => 3 => 分频=4
    */
    CAN1->BTR = (0x00 << 24) |  // SILM(31) | LBKM(30) = 0
            (0x00 << 22) |  // SJW(23:22) = 0 (SJW = 1Tq)
            (0x02 << 20) |  // TS2(22:20) = 2 (TS2 = 3Tq)
            (0x0D << 16) |  // TS1(19:16) = 13 (TS1 = 14Tq)
            (0x0003);       // BRP(9:0) = 3 (Prescaler = 4)
```

### 4.3.10、退出初始化模式、进入正常模式
![[Pasted image 20250319170049.png]]
![[Pasted image 20250319170115.png]]
如上所示，软件对寄存器CAN_MCR的位INRQ清0时，会退出初始化模式，进入工作模式。
```c
CAN1->MCR &= ~(1UL << 0);  // 清除 INRQ (进入正常模式)
while (CAN1->MSR & (1UL << 0)); // 等待 MSR.INAK 变 0
```

### 4.3.11、设置过滤器0
![[Pasted image 20250319171413.png]]
如上所示，STM32F103ZET6一共有14个过滤器，但CAN要能正常收发，必须至少要设置一个过滤器。

#### 所有过滤器进入初始化模式
![[Pasted image 20250319172227.png]]
如上所示，通过寄存器CAN_FMR的位0-FINIT置1，让所有过滤器组进入初始化模式。
```c
CAN1->FMR |= (1UL << 0);   // 进入过滤器初始化模式
```

#### 设置过滤器组0通过所有标识符（CANID），即不过滤
```c
CAN1->sFilterRegister[0].FR1 = 0x00000000;
CAN1->sFilterRegister[0].FR2 = 0x00000000;
```
如上所示，数组0代表过滤器组0。当FR1与FR2都设置0x00000000时，代表不过滤任何CANID，即所有CANID都会被接收。此时，如果CAN总线上有很多高频的CAN消息的话，CAN中断会非常频繁地进入，极大地浪费MCU的资源。
后续，会弄一篇笔记，讲讲怎样设置过滤器组，让开发板只接收感兴趣的CANID，而不是所有的CANID。

#### 过滤器组0匹配FIFO0
![[Pasted image 20250319175428.png]]
```c
CAN1->FFA1R &= ~(1UL << 0);  // 过滤器组0 分配到 FIFO0
```

#### 激活过滤器组0
![[Pasted image 20250319175955.png]]
```c
CAN1->FA1R  |=  (1UL << 0);  // 激活过滤器 0
```

#### 所有过滤器组退出初始化模式
![[Pasted image 20250319180148.png]]
```c
CAN1->FMR   &= ~(1UL << 0);  // 退出过滤器初始化模式
```

## 4.4、发送CAN报文
### 4.4.1、确认邮箱是不是空闲
![[Pasted image 20250319181113.png]]
如上所示，通过判断寄存器CAN_TIxR的bit0-TXRQ是不是等于0，来确认发送邮箱是不是空闲的。
```c
/* 寻找空闲邮箱 */
for(mailbox = 0; mailbox < 3; mailbox++) {
  if((CAN1->sTxMailBox[mailbox].TIR & (1UL << 0)) == 0)
	 break;
}
if(mailbox >= 3)
	return 1; // 无空闲邮箱
```

### 4.4.2、清空某个发送邮箱
![[Pasted image 20250319181813.png]]
将发送邮箱的4个寄存器都清0即可，包括TIR、TDTR、TDLR、TDHR。
```c
/* 清空该邮箱 */
CAN1->sTxMailBox[mailbox].TIR  = 0;
CAN1->sTxMailBox[mailbox].TDTR = 0;
CAN1->sTxMailBox[mailbox].TDLR = 0;
CAN1->sTxMailBox[mailbox].TDHR = 0;
```

### 4.4.3、设置将要发送的CANID、CAN帧类型
![[Pasted image 20250319182602.png]]
```c
CAN1->sTxMailBox[mailbox].TIR |= (stdId << 21);  // 标准ID写入TIR的[31:21]、IDE=0相当于标准帧、RTR=0相当于数据帧
```

### 4.4.4、设置CAN报文长度
![[Pasted image 20250319182816.png]]
```c
CAN1->sTxMailBox[mailbox].TDTR = (DLC & 0x0F); // 设置CAN报文的长度。使用&运算的目的是保证只有变量DLC的低四位写入TDTR寄存器，不会干涉到其他位。
```

### 4.4.5、将要发送的数据放入发送邮箱
![[Pasted image 20250319183827.png]]
```c
/* 填充数据 */
if(DLC <= 4) {
	for(uint8_t i = 0; i < DLC; i++) {
	  CAN1->sTxMailBox[mailbox].TDLR |= ((uint32_t)data[i]) << (8 * i);
	}
} else {
	for(uint8_t i = 0; i < 4; i++) {
	  CAN1->sTxMailBox[mailbox].TDLR |= ((uint32_t)data[i]) << (8 * i);
	}
	for(uint8_t i = 4; i < DLC; i++) {
	  CAN1->sTxMailBox[mailbox].TDHR |= ((uint32_t)data[i]) << (8 * (i-4));
	}
}
```

### 4.4.6、请求发送数据
![[Pasted image 20250319183938.png]]
```c
CAN1->sTxMailBox[mailbox].TIR |= CAN_TI0R_TXRQ;
```

### 4.4.7、等待邮箱的CAN消息被成功发送（可选）
![[Pasted image 20250319185105.png]]
```c
/* 轮询等待TXRQ清零或超时 */
while((CAN1->sTxMailBox[mailbox].TIR & CAN_TI0R_TXRQ) && --timeout);
if(timeout == 0) {
	// 发送失败(无ACK或位错误), 返回2
	return 2;
}
```

#### 为什么要等待TXRQ清零？
TXRQ 位的作用是在发送请求时置 1，并在以下情况下自动清零：
1. 报文成功发送（总线空闲时成功仲裁，并收到 ACK）。
2. 发送失败（无 ACK 或仲裁失败）：
	- 若 NART=0（自动重传开启），CAN 硬件会自动重试，直到发送成功。
	- 若 NART=1（自动重传关闭），发送失败时 TXRQ 也会清零，并可能产生错误标志（TERR、ALST、REC/TEC 递增等）。
3. 软件手动中止发送（通过设置 ABRQ 置 1 来取消发送）。

如果不等待 TXRQ 清零，可能发生：
- 报文未成功发送（因无ACK、错误等原因），但代码并不知道，导致误以为报文已经发送成功；
- CAN 总线忙碌，报文未立即发送，但代码已经继续执行其他任务，可能影响数据完整性。

#### 什么时候可以不等 TXRQ 清零？
1. 如果程序不关心发送是否成功（只管发，不管 ACK），可以不等 TXRQ 清零。
2. 如果使用中断模式（而非轮询），可以不在此等待 TXRQ，而是注册 CAN 发送完成中断（最常用！！！！！！）。
3. 如果应用层通过 TSR（Transmit Status Register）等方式定期检查发送状态，而不依赖 TXRQ 位轮询。

#### 什么时候必须等 TXRQ 清零？
1. 需要确认报文已发送完毕（尤其是 NART=1 时，若无 ACK 会导致发送失败）。
2. 要确保 FIFO 发送顺序正确（如果多个报文依次发送，等待 TXRQ 清零可确保当前报文已经结束）。
3. 应用层需要可靠的反馈（如果 TXRQ 持续置位，说明发送失败，应触发错误处理机制）。

# 五、代码（寄存器方式）
## 5.1、main.c
![[Pasted image 20250319191709.png | 800]]
![[Pasted image 20250319191911.png | 800]]
如上所示，函数`CAN_Config()`将CAN设置好，并进入正常工作模式。波特率500K，其他小功能全部关闭，且不过滤任何CANID。
![[Pasted image 20250319192409.png | 800]]
![[Pasted image 20250319192448.png | 800]]
![[Pasted image 20250319192539.png | 800]]

## 5.2、编译、调试
![[Pasted image 20250319192614.png | 800]]
编译OK，下载程序到开发板，效果如下所示：
![[LL_12_CANSend.gif]]