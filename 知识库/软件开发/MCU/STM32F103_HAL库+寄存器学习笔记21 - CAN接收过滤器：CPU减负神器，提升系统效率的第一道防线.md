# 导言
---
在STM32F103的CAN总线应用中，硬件过滤器（Filter）承担着关键角色。  本章将从寄存器层面深入剖析CAN接收过滤器的工作机制与配置方法，帮助理解如何高效筛选关键信息，减轻CPU负担。  通过合理使用过滤器，不仅能提升系统整体运行效率，还能显著降低接收溢出风险。  文章将结合HAL库操作与直接寄存器配置，全面构建底层视角下的CAN接收优化思路。
<br>
## CAN接收过滤器的作用
**当总线上来的每一帧CAN报文到达CAN接收器时，先用硬件进行"筛选"，只留下感兴趣的报文，才存入FIFO（FIFO0或FIFO1）和SRAM，其他无关的报文直接丢弃，不打扰CPU。**
- 需要的报文 → 硬件自动搬入FIFO队列，触发中断或等待软件取走
- 不需要的报文 → 直接被硬件丢弃，**不占用CPU，不占用SRAM**
![[Pasted image 20250427143432.png]]
<br>
## 为什么一定要学会使用CAN接收过滤器？
1. 减少CPU负担
>CAN总线是广播式通讯（所有节点都能接收到所有消息）;
>如果**不用过滤器**，所有的报文（不管有没有用）都会进入FIFO，CPU必须**逐条检查**ID、丢弃无关数据，极大浪费性能。
>尤其是**总线上有大量节点、流量很大**时，如果不用过滤器，CPU完全忙不过来，很快FIFO溢出。
2. 防止FIFO溢出()
>STM32F1的CAN模块，FIFO只有**3个邮箱**容量，如果无关报文都塞进来，真正有用的报文可能还没到，FIFO就满了，造成丢帧。
>硬件过滤器提前丢掉不关心的报文，让FIFO只存**自己关心的ID**的数据，提高可靠性。
3. 快速响应重要数据()
>重要的报文，比如控制指令、状态回报，需要**快速响应**。
>通过过滤器配置，只接收这些关键信息，提升系统响应速度和确定性。

## 本次实验
![[Pasted image 20250427181848.png]]
如上所示，本次实验旨在使用CAN滤波器1与CAN滤波器2对CAN总线报文进行过滤，使开发板仅接收特定的CAN ID。开发板只有接收到以上CANID时，才会触发系统中断。

项目地址：  
github:
- HAL库: https://github.com/q164129345/MCU_Develop/tree/main/stm32f103_hal_library21_Can_Rx_Filter
- 寄存器方式: https://github.com/q164129345/MCU_Develop/tree/main/stm32f103_reg_library21_Can_Rx_Filter

gitee(国内):
- HAL库: https://gitee.com/wallace89/MCU_Develop/tree/main/stm32f103_hal_library21_Can_Rx_Filter
- 寄存器方式: https://gitee.com/wallace89/MCU_Develop/tree/main/stm32f103_reg_library21_Can_Rx_Filter

# 一、过滤器方案的4种组合
---

| 组合            | FM1R 位值    | FS1R 位值     | 每个 Bank 可过滤的内容                                             | 典型用途                                   |
| ------------- | ---------- | ----------- | ---------------------------------------------------------- | -------------------------------------- |
| ① Mask 32-bit | 0（Mask 模式） | 1（32 bit）   | 1 条 “ID + 掩码”  <br>（完整 11 bit 置于 _[31:21]_，可用掩码一次性选中一段 ID） | 只接收0x200 ~ 0x20F这类连续 ID 归到同一通道         |
| ② List 32-bit | 1（List 模式） | 1（32 bit）   | 2 个 离散 ID                                                  | 只关心极少数关键帧（如 0x123、0x456）               |
| ③ Mask 16-bit | 0（Mask 模式） | 0（16 bit×4） | 2 组 “ID + 掩码”  <br>（每组占 16 bit）                            | 只接收两段不同 ID 区间(0x200~0x20F或0x300~0x30F) |
| ④ List 16-bit | 1（List 模式） | 0（16 bit×4） | 4 个离散 ID                                                   | 批量白名单(只接收)：0x100/101/102/103           |
**总之，不考虑拓展帧，只考虑11bit标准帧时，组合3～组合4是最优解决方案。**

# 二、用组合③Mask 16-bit实现只接收0x200~0x20F与0x300~0x30F 
---
## 2.1、寄存器CAN_FM1R与寄存器CAN_FS1R
![[Pasted image 20250427153613.png]]
![[Pasted image 20250427153716.png]]
如上所示，将寄存器CAN_FS1R的FSCx设置0，寄存器CAN_FM1R的FBMx设置0时，CAN过滤器x进入组合③Mask 16-bit模式。

## 2.2、代码(HAL库)
### 2.2.1、myCanDrive.c
![[Pasted image 20250427160258.png]]
![[Pasted image 20250427160426.png]]
![[Pasted image 20250427160522.png]]
## 2.3、测试代码
![[LL_21_CAN_Filter.gif]]
如上所示，CAN分析仪发送CANID0x200~0x20F到开发板。然后，全局变量g_RxCount = 16证明开发板收到16个CAN报文。
![[LL_21_CAN_Filter_HAL_0X100.gif]]
如上所示，CAN分析仪发送CANID0x100~0x10F到开发板。全局变量g_RxCount仍然为16，未发生变化，证明未接收到CAN报文。CANID0x100~0x10F被过滤了，这是我们的目的。
![[LL_21_CAN_Filter_HAL_0X300.gif]]如上所示，CAN分析仪发送CANID0x300~0x30F到开发板。全局变量g_RxCount变成32，证明开发板收到16个CAN报文。
**实验证明，CAN过滤波器2只能接收两个区间的CANID，分别是0x200~0x20F与0x300~0x30F。**

## 2.4、代码（寄存器方式）
### 2.4.1、myCanDrive_reg.c
![[Pasted image 20250427162610.png]]
![[Pasted image 20250427162712.png]]
![[Pasted image 20250427162743.png]]
## 2.5、测试代码
![[LL_21_CAN_Filter_LL.gif]]
如上所示，程序的效果跟HAL库一样，开发板只能接收CANID:0x200~0x20F与0x300~0x30F。其他CANID都没办法进入FIFO1触发CAN接收挂号中断。

# 三、用组合④List 16-bit实现只接收4个白名单CANID
---
## 3.1、寄存器CAN_FM1R与寄存器CAN_FS1R
![[Pasted image 20250427174107.png]]
![[Pasted image 20250427174149.png]]
## 3.2、代码（HAL库）
### 3.2.1、myCanDrive.c
![[Pasted image 20250427174344.png]]
![[Pasted image 20250427174431.png]]
![[Pasted image 20250427174515.png]]
## 3.3、测试代码
![[LL_21_CAN_Filter_HAL_0X058.gif]]
如上所示，使用CAN分析仪发送CANID0x058~0x067，一共16帧CAN报文给开发板。从全局变量g_RxCount看来，开发板只收到一个CANID(0x058)的报文。
![[LL_21_CAN_Filter_HAL_0X158.gif]]
如上所示，使用CAN分析仪发送CANID0x158~0x167，一共16帧CAN报文给开发板。从全局变量g_RxCount看来，开发板只收到一个CANID(0x158)的报文。
**同理，0x258和0x358的实验结果相同。**

## 3.4、代码（寄存器方式）
### 3.4.1、myCanDrive_reg.c
![[Pasted image 20250427180402.png]]
![[Pasted image 20250427180434.png]]
![[Pasted image 20250427180507.png]]

## 3.5、测试代码
![[LL_21_CAN_Filter_LL 1.gif]]
如上所示，使用CAN分析仪一共发送54条不同CANID的CAN报文给开发板。从全局变量g_RxCount = 4看到，开发板只收到4个CAN报文（CANID分别是0x058、0x158、0x258、0x358）。
![[LL_21_CAN_Filter_LL_Range.gif]]
如上所示，原来滤波器2的滤波器功能还是正常的。说明滤波器1与滤波器2一起工作，筛选合适的CANID到FIFO1。实验成功！！



