# 导言
---
通过采用 CAN 发送完成中断，你可以让 CPU 从忙等待中解放出来，专注于其他任务，从而实现更高的效率和更好的系统响应。中断能让系统在处理多个任务时保持较好的调度和响应，适合复杂实时系统的设计。**总之，使用CAN发送完成中断，可以解放CPU，避免死等的情况。**
STM32CubeMX的CAN代码不支持LL库，所以只能用HAL库，或者自己撸寄存器（最高效、最底层的方式）。

程序效果：
![[LL14_CAN_Interrupt 1.gif]]
如上所示，CAN分析仪持续收到间隔100ms的CAN消息。

项目地址：
- 寄存器方式：https://github.com/q164129345/MCU_Develop/tree/main/stm32f103_ll_library14_Can_Send_Interrupt
- HAL库方式：https://github.com/q164129345/MCU_Develop/tree/main/stm32f103_hal_library14_Can_Send_Interrupt

## CAN发送中断
![[Pasted image 20250327191507.png]]
如上图所示，触发CAN发送中断的条件是发送邮箱0～2中任一个的消息发送出去，发送邮箱变成空。

# 一、CubeMX
---
![[Pasted image 20250321085420.png]]
如上所示，勾选CAN TX Interrupts全局中断。另外，其他设置跟上一章节一样[[STM32F103_LL库+寄存器学习笔记13 - 梳理外设CAN与如何发送CAN报文（串行发送）]]。

# 二、代码（HAL库）
---
## 2.1、can.c
![[Pasted image 20250321085902.png | 800]]
如上所示，勾选CAN TX Interrupts全局中断后，can.c的函数`HAL_CAN_MspInit()`里增加了优先级的设置与全局中断的开启。
## 2.2、stm32f1xx_it.c
![[Pasted image 20250328173023.png | 800]]
1. 如上所示，勾选CAN TX Interrupts全局中断后，stm32f1xx_it.c增加函数`USB_HP_CAN1_TX_IRQHandler()`，它就是CAN发送的中断回调的主函数。
2. 通过寄存器的方式更新空闲的发送邮箱数量。为什么要用寄存器方式？因为当CAN发送很频繁时，进入发送完成中断的频率很高。此时，应尽可能减少CPU的处理时间。不介意中断处理的效率的话，可以用`txmail_free = HAL_CAN_GetTxMailboxesFreeLevel(&hcan)`替代。
3. 使用`__disable_irq()`与`__enable_irq()`增加临界区保护的优点是：
	1. 100%避免所有潜在竞态条件
	2. 代码行为可预测性强
   然而，缺点是**增加约5个时钟周期的中断延迟（CPSID/CPSIE指令)**，72MHz的主频的话，大概增加0.14us的延迟。

## 2.3、main.c
![[Pasted image 20250329153447.png]]
![[Pasted image 20250329154012.png]]
1. 函数`CAN_Send_Msg_No_Serial()`的好处是肯定不会卡主循环，在裸机开发时，其他主循环任务不会出现被延时。坏处是可能会因为发送邮箱被挤满了，导致当前要发送的CAN消息被丢掉。所以，一定要认真检查全局变量canSendError。如果canSendError频繁地递增，证明要发送的CAN消息被丢掉了（因为正要发送CAN消息时，CAN发送邮箱满了）。此时，应该降低发送CAN消息的频率，或者提高CAN通讯波特率。**后续，可以通过消息队列来优化这个问题。**
2. 另外，如果使用CAN总线来传递OTA升级包（bootloader升级），一定要使用类似函数CAN_Send_Msg_Serial()这种串行的发送方式。因为当发送邮箱挤满时，它会卡死在那里，等待发送邮箱空闲。

## 2.4、编译、下载
![[Pasted image 20250329155654.png]]
如上所示，编译代码成功。

## 2.5、测试代码
![[Pasted image 20250329155828.png]]
如上所示，正常进入CAN发送完成中断。
![[LL14_CAN_Interrupt.gif]]
如上所示，CAN分析仪持续接收到CAN消息，间隔100ms。然后，全局变量canSendError一直等于0，表示没有CAN消息因为CAN发送邮箱挤满而丢弃。此时，可以进一步增加CAN的发送频率。

# 三、寄存器梳理
---
## 3.1、CAN中断使能寄存器 (CAN_IER)
![[Pasted image 20250329160841.png]]
![[Pasted image 20250329160915.png]]
如上所示，寄存器CAN_IER的位0(TMEIE)控制发送邮箱空中断使能。要开启中断，将它置1。
```c
// 使能发送中断（方法1）
CAN1->IER |= CAN_IER_TMEIE; // 发送邮箱空中断使能
// 使能发送中断（方法2）
CAN1->IER |= 0x01UL << 0;  // 发送邮箱空中断使能  
```
## 3.2、CAN发送状态寄存器 (CAN_TSR)
![[Pasted image 20250329161328.png]]
如上所示，读取寄存器CAN_TSR的位26～位28(TME0~TME2)可以知道发送邮箱是不是空。
```c
// 检查发送邮箱空中断标志(方法1)
if(CAN1->TSR & CAN_TSR_TME0) // 邮箱0空
{
	// 你的处理代码...
	CAN1->TSR |= CAN_TSR_TME0; // 清除标志（通过写1清除!!!必须清除！！）
}
if(CAN1->TSR & CAN_TSR_TME1) // 邮箱1空
{
	// 你的处理代码...
	CAN1->TSR |= CAN_TSR_TME1;
}
if(CAN1->TSR & CAN_TSR_TME2) // 邮箱2空
{
	// 你的处理代码...
	CAN1->TSR |= CAN_TSR_TME2;
}

// 检查发送邮箱空中断标志(方法2)
if(CAN1->TSR & (0x1UL << 26UL)) // 邮箱0空
{
	// 你的处理代码...
	CAN1->TSR |= (0x1UL << 26UL); // 清除标志（通过写1清除）
}
if(CAN1->TSR & (0x1UL << 27UL)) // 邮箱1空
{
	// 你的处理代码...
	CAN1->TSR |= (0x1UL << 27UL);
}
if(CAN1->TSR & (0x1UL << 28UL)) // 邮箱2空
{
	// 你的处理代码...
	CAN1->TSR |= (0x1UL << 28UL);
}
```

# 四、代码（寄存器）
---
## 4.1、stm32f1xx_it.c
![[Pasted image 20250329165056.png]]
如上所示，在启动文件startup_stm32f103xe.s的中断向量表里找到CAN1发送完成的函数名字定义。`USB_HP_CAN1_TX_IRQHandler()`就是CAN1发送完成中断。
![[Pasted image 20250329165401.png]]

## 4.2、main.c
![[Pasted image 20250329165652.png]]
![[Pasted image 20250330113920.png]]

![[Pasted image 20250330113510.png]]

## 4.3、编译、下载
![[Pasted image 20250330114017.png]]
如上所示，编译没有错误。

## 4.4、测试效果
![[LL14_CAN_IT.gif]]
如上图所示，CAN分析持续收到CAN消息，间隔100ms。另外，在debug模式下，观察全局变量`canSendError`一直都是0，表示没有CAN消息被丢掉。

# 五、细节补充
---
## 5.1、通过软件增加二级发送缓存的必要性
![[Pasted image 20250330121233.png]]
- **解决硬件发送邮箱溢出问题**。当应用层短时间内需要发送大量CAN消息（如传感器数据突发、诊断信息批量上传），而CAN控制器的3个发送邮箱已全部占用时，新消息将无法立即提交，导致数据丢失或发送延迟。
- **二级缓存作用（优点）：**
	1. 缓冲突发流量：例如64消息的环形缓存可暂时存储待发送消息，缓解硬件资源瓶颈。
	2. 平滑数据流：通过队列机制按序消费消息，避免因瞬时高负载导致的发送拥塞。
	3. 异步发送机制：应用层只需将消息投递到队列，由后台任务/中断自动处理发送（充分发挥CAN发送完成中断的价值）。
	4. 资源解耦：发送过程对应用透明，提升代码模块化程度。
	5. 消息持久化存储：在收到发送成功中断确认前，保留消息副本。
	6. 自动重发机制：检测到发送失败时，自动从队列中重新提交消息。
- **二级缓存引起的问题（缺点）：**
	1. 内存占用：64消息的缓存可能消耗较多RAM（约64×13=832字节）。
	2. 实时性降低：消息需先入队再发送，引入微小延迟。
	3. 多线程/中断同时访问队列需同步。

总的来说，**通过二级缓存的引入，系统在保持CAN硬件高效性的同时，获得了更强的鲁棒性和灵活性，是构建高可靠性CAN通信系统的关键设计模式。** 下一章笔记开始梳理怎样编写这个二级缓存。

## 5.2、继续讨论一下函数CAN_SendMessage_NonBlocking()
![[Pasted image 20250330123315.png]]
如上所示，函数CAN_SendMessage_NonBlocking()可以这样去写：
```c
/**
  * @brief  直接操作寄存器实现CAN消息发送(标准数据帧)，代码不会阻塞
  * @param  stdId: 标准ID(11位)
  * @param  data: 数据指针
  * @param  DLC: 数据长度(0~8)
  * @retval 0=发送成功; 1=无空闲邮箱,没有发送数据
  */
uint8_t CAN_SendMessage_NonBlocking(uint32_t stdId, uint8_t *data, uint8_t DLC)
{
    uint8_t mailbox;
    // 访问寄存器，计算空闲的发送邮箱数量
    uint8_t txmails = ((CAN1->TSR & CAN_TSR_TME0) ? 1 : 0) +
                 ((CAN1->TSR & CAN_TSR_TME1) ? 1 : 0) +
                 ((CAN1->TSR & CAN_TSR_TME2) ? 1 : 0);
    if (txmails > 0) {
        /* 清空该邮箱并配置ID、DLC和数据 */
        CAN1->sTxMailBox[mailbox].TIR  = 0;
        CAN1->sTxMailBox[mailbox].TDTR = (DLC & 0x0F);
        CAN1->sTxMailBox[mailbox].TDLR = 0;
        CAN1->sTxMailBox[mailbox].TDHR = 0;
        CAN1->sTxMailBox[mailbox].TIR |= (stdId << 21);

        /* 填充数据 */
        for(uint8_t i = 0; i < DLC && i < 8; i++) {
            if(i < 4)
                CAN1->sTxMailBox[mailbox].TDLR |= ((uint32_t)data[i]) << (8 * i);
            else
                CAN1->sTxMailBox[mailbox].TDHR |= ((uint32_t)data[i]) << (8 * (i-4));
        }

        /* 发起发送请求并直接返回 */
        CAN1->sTxMailBox[mailbox].TIR |= CAN_TI0R_TXRQ;
        txmail_free--;
        return 0; // 已发起发送请求，不等待完成
    } else {
        return 1; // 发送失败，因为发送邮箱满了
    }
}
```
在函数`CAN_SendMessage_NonBlocking()`里访问寄存器TSR并计算出空闲的发送邮箱的数量。这种实现方式比中断方式简单，且更加模块化。为什么要在中断里更新全局变量`txmail_free`呢？

1. **轮询全局变量txmail_free的效率更高。** 轮询“访问寄存器CAN1->TSR，计算空闲发送邮箱的数量”比轮询内存里的"全局变量txmail_free"的时间要长得多。例如在低频100Hz的条件下，两个轮询方案所占用的CPU时间差不多。此时，我认为中断方式意义不大。但是，在高频10KHz的情况下，轮询全局变量txmail_free所占用的CPU时间就短得多。但是的但是，轮询频繁不可能这么高，100Hz已经相当快了。
2. **发送完成中断的重要意义是配合二级缓存使用。** 如果想CAN发送的代码简单一些，不考虑实现二级缓存的话，我认为没有必要开启发送完成中断，简单地使用串行发送函数`CAN_SendMessage_Blocking()`就足够了。

> 在 500kbps 波特率下，发送标准CAN帧（8字节数据） 的时间约为 260μs。1M波特率下，发送标准CAN帧（8字节数据） 的时间约为 130μs。