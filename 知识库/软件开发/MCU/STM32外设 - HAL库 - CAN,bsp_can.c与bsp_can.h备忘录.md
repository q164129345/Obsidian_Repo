## 一、前言
----
记录一下CubeMX的CAN1与CAN2设置，然后通过bsp_can.c与bsp_can.h实现CAN发送与CAN中断接收.

## 二、CubeMX
----
### 2.1、CAN1
![[Pasted image 20240817202437.png]]
###  2.2、CAN2
![[Pasted image 20240817202529.png]]

## 三、代码
----
### 3.1、bsp_can.c
```c
#include "bsp_can.h"
#include "can.h"
#include "app_can.h"

static CAN_RxHeaderTypeDef CAN1_RecHandler;
static CAN_RxHeaderTypeDef CAN2_RecHandler;
static CanFrame_Type CAN1_Buffer;
static CanFrame_Type CAN2_Buffer;

/*****************************************************************************************************
* describe: 配置CAN1的滤波器、关联RXFIFO、开启RXFIFO中断
* input parameter: 
* output parameter: 无
* retval:无
* note:
*****************************************************************************************************/
void CAN1_Filter_Config(void)
{
    CAN_FilterTypeDef  sFilterConfig;

    sFilterConfig.FilterBank = 0;                       // 设置滤波器组0
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;   // 屏蔽位模式
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;  // 32位宽
    sFilterConfig.FilterIdHigh = 0x0000;                // 相当于不过滤CAN ID
    sFilterConfig.FilterIdLow = 0x0000;                 // 相当于不过滤CAN ID
    sFilterConfig.FilterMaskIdHigh = 0x0000;            // 相当于不过滤CAN ID
    sFilterConfig.FilterMaskIdLow = 0x0000;             // 相当于不过滤CAN ID
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;  // 使用RX_FIFO0
    sFilterConfig.FilterActivation = ENABLE;            // 激活滤波器
    sFilterConfig.SlaveStartFilterBank = 14;            // 当只使用CAN1时，该位无效。当开启双路CAN时，这个值等于CAN2的滤波器组（本代码CAN2的滤波器组是14）

	/* 配置滤波器 */
    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
   {
        while(1);
    }
	 
   /* 开启CAN1控制器 */
   if (HAL_CAN_Start(&hcan1) != HAL_OK)
   {
        while(1);
   }
   /* 使能CAN1中断的方式与RX_FIFO通道 */
   if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
   {
        while(1);
   }
}

/*****************************************************************************************************
* describe: 配置CAN2的滤波器、关联RXFIFO、开启RXFIFO中断
* input parameter: 
* output parameter: 无
* retval:无
* note:
*****************************************************************************************************/
void CAN2_Filter_Config(void)
{
    CAN_FilterTypeDef  sFilterConfig;

    sFilterConfig.FilterBank = 14;                      // 设置过滤器组14
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;   // 屏蔽位模式
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;  // 32位宽
    sFilterConfig.FilterIdHigh = 0x0000;				// 相当于不过滤CAN ID
    sFilterConfig.FilterIdLow = 0x0000;					// 相当于不过滤CAN ID
    sFilterConfig.FilterMaskIdHigh = 0x0000;			// 相当于不过滤CAN ID
    sFilterConfig.FilterMaskIdLow = 0x0000;				// 相当于不过滤CAN ID
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;  // 使用RX_FIFO1
    sFilterConfig.FilterActivation = ENABLE;            // 激活滤波器
    sFilterConfig.SlaveStartFilterBank = 14;            // 此位无效

		/* 配置滤波器 */
    if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK)
   {
        while(1);
   }

	 /* 开启CAN2控制器 */ 	
   if (HAL_CAN_Start(&hcan2) != HAL_OK)
   {
        while(1);
   }

	 /* 使能CAN2中断的方式与RX_FIFO通道 */
   if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
   {
        while(1);
   }
}

uint8_t CAN1_SendData(uint32_t std_id, uint32_t ext_id, uint8_t *msg,uint8_t len)
{
    rt_mutex_take(&CAN1_Send_Mutex, RT_WAITING_FOREVER); // 获取互斥量
    uint32_t TxMailbox;
    CAN_TxHeaderTypeDef canTxHeader;
    canTxHeader.StdId = std_id;
    canTxHeader.ExtId = ext_id;

    if(canTxHeader.ExtId > 0) {
        canTxHeader.IDE = CAN_ID_EXT;
    } else {
        canTxHeader.IDE = CAN_ID_STD;
    }
    canTxHeader.RTR = CAN_RTR_DATA;
    canTxHeader.DLC = len;

    if(HAL_CAN_AddTxMessage(&hcan1, &canTxHeader, msg, &TxMailbox) != HAL_OK) {
        return 1;
    }

    /* 这样写法不太高效，后续需要改为信号量来处理 */
    /* 确保Mailboxes完全是空的时候，才离开函数 */
    // wait can mailbox is free
    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3) {
        rt_thread_mdelay(1); // 延时1ms
    }

    rt_mutex_release(&CAN1_Send_Mutex); // 释放互斥量

    return 0;
}

uint8_t CAN2_SendData(uint32_t std_id, uint32_t ext_id, uint8_t *msg,uint8_t len)
{
    rt_mutex_take(&CAN2_Send_Mutex, RT_WAITING_FOREVER); // 获取互斥量
    uint32_t TxMailbox;
    CAN_TxHeaderTypeDef canTxHeader;
    canTxHeader.StdId = std_id;
    canTxHeader.ExtId = ext_id;

    if(canTxHeader.ExtId > 0) {
        canTxHeader.IDE = CAN_ID_EXT;
    } else {
        canTxHeader.IDE = CAN_ID_STD;
    }

    canTxHeader.RTR = CAN_RTR_DATA;			
    canTxHeader.DLC = len;

    if(HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, msg, &TxMailbox) != HAL_OK) {
        return 1;
    }
    /* 这样写法不太高效，后续需要改为信号量来处理 */
    /* 确保Mailboxes完全是空的时候，才离开函数 */
    // wait can mailbox is free
    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) != 3) {
        rt_thread_mdelay(1);  /* 延时1ms */
    }

    rt_mutex_release(&CAN2_Send_Mutex); // 释放互斥量

    return 0;
}

/*****************************************************************************************************
* describe: can1 发送数据接口
* input parameter: 
* @std_id-> 标准ID
* @ext_id-> 拓展ID
* @msg-> 要发送的数据指针
* @len-> 发送的数据长度
* output parameter: 无
* retval: 0 or 1
* note:
*****************************************************************************************************/
uint8_t CAN1_SendMsg(uint32_t std_id, uint32_t ext_id, uint8_t* msg,uint32_t len)
{
    uint8_t sendLen;
    uint8_t *t_msg = msg;

    if (msg == NULL || len == 0) {
        return 1;
    }

    do {
        sendLen = len > 8 ? 8:len;

        if (CAN1_SendData(std_id, ext_id, t_msg, sendLen) == 1){
            return 1;
        }

        len -= sendLen;
        t_msg += sendLen;
    }while (len);

    return 0;
}

/*****************************************************************************************************
* describe: can2 发送数据接口
* input parameter: 
* @std_id-> 标准ID
* @ext_id-> 拓展ID
* @msg-> 要发送的数据指针
* @len-> 发送的数据长度
* output parameter: 无
* retval: 0 or 1
* note:
*****************************************************************************************************/
uint8_t CAN2_SendMsg(uint32_t std_id, uint32_t ext_id, uint8_t* msg,uint32_t len)
{
    uint8_t sendLen;
    uint8_t *t_msg = msg;

    if (msg == NULL || len == 0) {
        return 1;
    }

    do {
        sendLen = len > 8 ? 8:len;

        if (CAN2_SendData(std_id, ext_id, t_msg, sendLen) == 1){
            return 1;
        }

        len -= sendLen;
        t_msg += sendLen;
    }while (len);

    return 0;
}

/*****************************************************************************************************
* describe: can1中断执行函数
* input parameter: 
* @hcan-> can操作句柄
* output parameter: 无
* retval: 无
* note:
*****************************************************************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if(hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&CAN1_RecHandler,CAN1_Buffer.DataBuff);  //从FIFO0获取CAN报文
        CAN1_Buffer.Len = CAN1_RecHandler.DLC; // 获取报文长度
        if (CAN_ID_STD == CAN1_RecHandler.IDE) {
            CAN1_Buffer.ID = CAN1_RecHandler.StdId;
            CAN1_Buffer.ExFlag = 0;
        } else if (CAN_ID_EXT == CAN1_RecHandler.IDE) {
            CAN1_Buffer.ID = CAN1_RecHandler.ExtId;
            CAN1_Buffer.ExFlag = 1;
        }
        rt_mq_send(&CAN1_Rec_MQ,&CAN1_Buffer,sizeof(CAN1_Buffer)); // 将消息放入消息队列
    }
    /* 使能CAN1中断的方式与RX_FIFO通道 */
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/*****************************************************************************************************
* describe: can2中断执行函数
* input parameter: 
* @hcan-> can操作句柄
* output parameter: 无
* retval: 无
* note:
*****************************************************************************************************/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint8_t rxBuf[8] = {0,};

    if(hcan->Instance == CAN2)
    {
        HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO1,&CAN2_RecHandler,CAN2_Buffer.DataBuff);  //从FIFO1获取CAN报文
        CAN2_Buffer.Len = CAN2_RecHandler.DLC; // 获取报文长度
        if (CAN_ID_STD == CAN2_RecHandler.IDE) {
            CAN2_Buffer.ID = CAN2_RecHandler.StdId;
            CAN2_Buffer.ExFlag = 0;
        } else if (CAN_ID_EXT == CAN2_RecHandler.IDE) {
            CAN2_Buffer.ID = CAN2_RecHandler.ExtId;
            CAN2_Buffer.ExFlag = 1;
        }
        rt_mq_send(&CAN2_Rec_MQ,&CAN2_Buffer,sizeof(CAN2_Buffer)); // 将消息放入消息队列
    }
    /* 使能CAN2中断的方式与RX_FIFO通道 */
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
}


```

### 3.2、bsp_can.h
```c
#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

// 数据结构
typedef struct CanFrame
{
    uint32_t ID;
    uint8_t DataBuff[8];
    uint8_t Len;
    uint8_t ExFlag;
} CanFrame_Type;

void CAN1_Filter_Config(void);
void CAN2_Filter_Config(void);
uint8_t CAN1_SendMsg(uint32_t std_id, uint32_t ext_id, uint8_t* msg,uint32_t len);
uint8_t CAN2_SendMsg(uint32_t std_id, uint32_t ext_id, uint8_t* msg,uint32_t len);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);


#ifdef __cplusplus
} 
#endif

#endif


```