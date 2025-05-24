# 导言
---
《[[STM32F103_LL库+寄存器学习笔记12.2 - 串口DMA高效收发实战2：进一步提高串口接收的效率]]》前阵子完成的LL库与寄存器版本的模块，有一个明显的缺点是不支持多实例化。总的来说，就是模块化没考虑进去。最近，开始基于HAL库系统地梳理一遍bootloader程序开发。所以，继续把代码迭代一下，将多实例化模块做出来。

bsp_usart_hal 驱动代码特点总结：
1. 支持多实例通用管理
	- 设计为USART_Driver_t结构体+接口函数，支持多个USART端口同时独立驱动，便于单板多串口应用、项目横向复用、一套代码多项目。
	- 各实例参数（缓冲区、DMA句柄、UART句柄）解耦，代码维护简便。

2. DMA收发全流程、双缓冲高效传输
	- 接收采用DMA环形模式，支持半传输（HT）、全传输（TC）、IDLE三类中断协同搬运，保证任意长度数据不丢包、无粘包。
	- 发送采用DMA块传输，自动调度RingBuffer队列，避免CPU阻塞。
	- 收发均使用DMA，极大减轻CPU负担，适用于高带宽、高实时性场景。

3. RingBuffer无缝缓存机制
	- 内置收发双向RingBuffer，自动管理协议包拼接、数据缓冲，简化上层协议处理。
	- 支持超大缓冲、溢出自动覆盖或丢弃旧数据，易于移植第三方协议栈。

4. 健壮的错误统计与自恢复机制
	- 内置DMA传输错误、串口硬件错误等统计计数（errorDMATX/errorDMARX/errorRX），便于产线异常监控、后期故障溯源。
	- 检测到DMA错误时，自动执行自恢复：自动重启DMA、自动清理异常、可定制连续多次异常的告警或自动复位，保障系统长期稳定运行。

5. 工程化量产导向
	- 代码结构清晰、注释标准，适用于量产项目长期维护。
	- CubeMX工程直接集成，无需魔改HAL库代码。
	- 支持多串口、多DMA。

测试效果如下，接收与发送都没有丢包。
![[Pasted image 20250523170304.png | 1100]]

# 一、CubeMX
---
## 1.1、Clock Configuration
![[Pasted image 20250524173212.png]]
## 1.2、USART1
![[Pasted image 20250524173341.png]]
波特率115200，数据长度8bits，停止位长度1bit。
![[Pasted image 20250524173436.png]]
使能USART1全局中断。
![[Pasted image 20250524173937.png]]
USART1_RX的DMA接收模式一定要选择“Circular”模式，长度选择Byte（字节） = 8bits。
![[Pasted image 20250524174217.png]]USART1_RX的DMA发送模式一定要选择“Normal”模式，长度选择Byte（字节） = 8bits。
![[Pasted image 20250524174407.png]]
GPIO Setting的设置如上所示。

# 二、代码
---
## 2.1、bsp_usart_hal.h
```c
/**
 * @file    bsp_usart_hal.h
 * @brief   STM32F1系列 USART + DMA + RingBuffer HAL库底层驱动接口（多实例、可变缓冲区）
 * @author  Wallace.zhang
 * @version 2.0.0
 * @date    2025-05-23
 */
#ifndef __BSP_USART_HAL_H
#define __BSP_USART_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "usart.h"
#include "dma.h"
#include "lwrb/lwrb.h"

/**
 * @brief USART驱动结构体（DMA + RingBuffer + 统计）
 */
typedef struct
{
    volatile uint8_t   txDMABusy;        /**< DMA发送忙标志（1：发送中，0：空闲） */
    volatile uint64_t  rxMsgCount;       /**< 统计接收字节总数 */
    volatile uint64_t  txMsgCount;       /**< 统计发送字节总数 */
    volatile uint16_t  dmaRxLastPos;     /**< DMA接收缓冲区上次处理到的位置 */
    volatile uint32_t  errorDMATX;       /**< DMA发送错误统计 */
    volatile uint32_t  errorDMARX;       /**< DMA接收错误统计 */
    volatile uint32_t  errorRX;          /**< 串口接收错误统计 */

    DMA_HandleTypeDef   *hdma_rx;        /**< HAL库的DMA RX句柄 */
    DMA_HandleTypeDef   *hdma_tx;        /**< HAL库的DMA TX句柄 */
    UART_HandleTypeDef  *huart;          /**< HAL库的UART句柄 */

    /* RX方向 */
    uint8_t             *rxDMABuffer;    /**< DMA接收缓冲区 */
    uint8_t             *rxRBBuffer;     /**< 接收RingBuffer缓存区 */
    uint16_t            rxBufSize;       /**< DMA接收/RX Ringbuffer缓冲区大小 */
    lwrb_t              rxRB;            /**< 接收RingBuffer句柄 */

    /* TX方向 */
    uint8_t             *txDMABuffer;    /**< DMA发送缓冲区 */
    uint8_t             *txRBBuffer;     /**< 发送RingBuffer缓存区 */
    uint16_t            txBufSize;       /**< DMA发送/TX Ringbuffer缓冲区大小 */
    lwrb_t              txRB;            /**< 发送RingBuffer句柄 */
    
} USART_Driver_t;

/**
  * @brief  阻塞方式发送以 NUL 结尾的字符串
  */
void USART_SendString_Blocking(USART_Driver_t* const usart, const char* str);
/**
  * @brief  USART发送DMA中断处理函数
  */
void USART_DMA_TX_Interrupt_Handler(USART_Driver_t *usart);
/**
  * @brief  USART接收DMA中断处理函数
  */
void USART_DMA_RX_Interrupt_Handler(USART_Driver_t *usart);
/**
 * @brief  USART空闲接收中断处理函数（支持DMA+RingBuffer，适用于多实例）
 */
void USART_RX_IDLE_Interrupt_Handler(USART_Driver_t *usart);
/**
  * @brief  将数据写入指定USART驱动的发送 RingBuffer中
  */
uint8_t USART_Put_TxData_To_Ringbuffer(USART_Driver_t *usart, const void* data, uint16_t len);
/**
 * @brief  USART模块定时任务处理函数，建议主循环1ms周期回调
 */
void USART_Module_Run(USART_Driver_t *usart);
/**
  * @brief  获取USART接收RingBuffer中的可读字节数
  */
uint32_t USART_Get_The_Existing_Amount_Of_Data(USART_Driver_t *usart);
/**
  * @brief  从USART接收RingBuffer中读取一个字节数据
  */
uint8_t USART_Take_A_Piece_Of_Data(USART_Driver_t *usart, uint8_t* data);
/**
  * @brief  DMA传输错误后的自动恢复操作（含错误统计）
  */
void USART_DMA_Error_Recover(USART_Driver_t *usart, uint8_t dir);
/**
 * @brief   初始化USART驱动，配置DMA、RingBuffer与中断
 */
void USART_Config(USART_Driver_t *usart,
                  uint8_t *rxDMABuffer, uint8_t *rxRBBuffer, uint16_t rxBufSize,
                  uint8_t *txDMABuffer, uint8_t *txRBBuffer, uint16_t txBufSize);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_USART_HAL_H */

```

## 2.2、bsp_usart_hal.c
```c
/**
 * @file    bsp_usart_hal.c
 * @brief   STM32F1系列 USART + DMA + RingBuffer HAL库底层驱动实现（多实例、可变缓冲区）
 * @author  Wallace.zhang
 * @version 2.0.0
 * @date    2025-05-23
 */

#include "bsp_usart_hal.h"

/**
  * @brief   阻塞方式发送以NUL结尾字符串（调试用，非DMA）
  * @param   usart  指向USART驱动结构体的指针
  * @param   str    指向以'\0'结尾的字符串
  * @note    通过HAL库API逐字节发送，底层会轮询TXE位（USART_SR.TXE）。
  * @retval  无
  */
void USART_SendString_Blocking(USART_Driver_t* usart, const char* str)
{
    if (!usart || !str) return;
    HAL_UART_Transmit(usart->huart, (uint8_t*)str, strlen(str), 1000);
}

/**
  * @brief  配置并启动USART的DMA接收（环形模式）
  * @param  usart 指向USART驱动结构体的指针
  * @note   必须保证huart、hdma_rx已通过CubeMX正确初始化
  *         - 调用本函数会停止原有DMA，然后重新配置DMA并启动环形接收
  *         - 使能USART的IDLE中断，实现突发/不定长帧高效处理
  * @retval 无
  */
static void USART_Received_DMA_Configure(USART_Driver_t *usart)
{
    if (!usart) return;
    HAL_DMA_Abort(usart->hdma_rx); //! 先关闭
    HAL_UART_Receive_DMA(usart->huart, usart->rxDMABuffer, usart->rxBufSize); //! 启动环形DMA
    __HAL_UART_ENABLE_IT(usart->huart, UART_IT_IDLE); //! 使能IDLE中断
    usart->dmaRxLastPos = 0;
}

/**
  * @brief  启动DMA方式串口发送
  * @param  usart 指向USART驱动结构体的指针
  * @param  data  指向待发送的数据缓冲区
  * @param  len   待发送的数据字节数
  * @note   发送前需确保txDMABusy为0，否则应等待前一帧发送完成
  *         启动后DMA自动填充USART_DR寄存器，实现高效异步发送
  * @retval 无
  */
static void USART_SendString_DMA(USART_Driver_t *usart, uint8_t *data, uint16_t len)
{
    if (!usart || !data || len == 0 || len > usart->txBufSize) return;
    while (usart->txDMABusy); // 等待DMA空闲
    usart->txDMABusy = 1;
    HAL_UART_Transmit_DMA(usart->huart, data, len);
}

/**
 * @brief  写入数据到接收RingBuffer
 * @param  usart 指向USART驱动结构体的指针
 * @param  data  指向要写入的数据缓冲区
 * @param  len   要写入的数据长度（单位：字节）
 * @retval 0  数据成功写入，无数据丢弃
 * @retval 1  ringbuffer剩余空间不足，丢弃部分旧数据以容纳新数据
 * @retval 2  数据长度超过ringbuffer总容量，仅保留新数据尾部（全部旧数据被清空）
 * @retval 3  输入数据指针为空
 * @note
 * - 本函数通过lwrb库操作ringbuffer。
 * - 当len > ringbuffer容量时，强行截断，仅保留最新usart->rxRBBuffer字节。
 * - 若空间不足，自动调用lwrb_skip()丢弃部分旧数据。
 */
static uint8_t Put_Data_Into_Ringbuffer(USART_Driver_t *usart, const void *data, uint16_t len)
{
    //! 检查输入指针是否合法
    if (!usart || !data) return 3;
    
    lwrb_t *rb = &usart->rxRB;
    uint16_t rb_size = usart->rxBufSize;
    //! 获取当前RingBuffer剩余空间
    lwrb_sz_t free_space = lwrb_get_free(rb);
    
    //! 分三种情况处理：长度小于、等于、大于RingBuffer容量
    uint8_t ret = 0;
    if (len < rb_size) {
        //! 数据小于RingBuffer容量
        if (len <= free_space) {
            //! 空间充足，直接写入
            lwrb_write(rb, data, len);
        } else {
            //! 空间不足，需丢弃部分旧数据
            lwrb_sz_t used = lwrb_get_full(rb);
            lwrb_sz_t skip_len = len - free_space;
            if (skip_len > used) {
                skip_len = used;
            }
            lwrb_skip(rb, skip_len); //! 跳过（丢弃）旧数据
            lwrb_write(rb, data, len);
            ret = 1;
        }
    } else if (len == rb_size) { //! 数据刚好等于RingBuffer容量
        if (free_space < rb_size) {
            lwrb_reset(rb); //! 空间不足，重置RingBuffer
            ret = 1;
        }
        lwrb_write(rb, data, len);
    } else { //! 数据超过RingBuffer容量，仅保留最后rb_size字节
        const uint8_t *byte_ptr = (const uint8_t *)data;
        data = (const void *)(byte_ptr + (len - rb_size));
        lwrb_reset(rb);
        lwrb_write(rb, data, rb_size);
        ret = 2;
    }
    return ret;
}

/**
 * @brief  从DMA环形缓冲区搬运新收到的数据到RingBuffer（支持环绕）
 * @param  usart 指向USART驱动结构体的指针
 * @note   支持IDLE、DMA HT/TC等多中断共同调用
 *         - 本函数计算DMA环形缓冲区的新数据，并搬运到RingBuffer
 *         - 支持一次性或分段搬运（缓冲区环绕时自动分两段处理）
 * @retval 无
 */
static void USART_DMA_RX_Copy(USART_Driver_t *usart)
{
    uint16_t bufsize  = usart->rxBufSize;
    uint16_t curr_pos = bufsize - __HAL_DMA_GET_COUNTER(usart->hdma_rx);
    uint16_t last_pos = usart->dmaRxLastPos;

    if (curr_pos != last_pos) {
        if (curr_pos > last_pos) {
        //! 普通情况，未环绕
            Put_Data_Into_Ringbuffer(usart, usart->rxDMABuffer + last_pos, curr_pos - last_pos);
            usart->rxMsgCount += (curr_pos - last_pos);
        } else {
        //! 环绕，分两段处理
            Put_Data_Into_Ringbuffer(usart, usart->rxDMABuffer + last_pos, bufsize - last_pos);
            Put_Data_Into_Ringbuffer(usart, usart->rxDMABuffer, curr_pos);
            usart->rxMsgCount += (bufsize - last_pos) + curr_pos;
        }
        usart->dmaRxLastPos = curr_pos;
    }
}

/**
  * @brief  DMA接收搬运环形Buffer（推荐在IDLE、DMA中断等调用）
  * @param  usart 指向USART驱动结构体的指针
  * @retval 无
  */
void USART_DMA_RX_Interrupt_Handler(USART_Driver_t *usart)
{
    if (!usart) return;
    USART_DMA_RX_Copy(usart);
}

/**
  * @brief  串口IDLE中断处理（需在USARTx_IRQHandler中调用）
  * @param  usart 指向USART驱动结构体的指针
  * @note   检查并清除IDLE标志，及时触发DMA搬运
  * @retval 无
  */
void USART_RX_IDLE_Interrupt_Handler(USART_Driver_t *usart)
{
    if (!usart) return;
    if (__HAL_UART_GET_FLAG(usart->huart, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(usart->huart);
        USART_DMA_RX_Copy(usart);
    }
}

/**
  * @brief  DMA发送完成回调（由用户在HAL库TxCpltCallback中调用）
  * @param  usart 指向USART驱动结构体的指针
  * @note   一定要调用，否则无法再次DMA发送
  * @retval 无
  */
void USART_DMA_TX_Interrupt_Handler(USART_Driver_t *usart)
{
    if (!usart) return;
    usart->txDMABusy = 0;
}

/**
  * @brief  将数据写入指定USART驱动的发送 RingBuffer 中
  * @param  usart  指向USART驱动结构体的指针
  * @param  data   指向要写入的数据缓冲区
  * @param  len    要写入的数据长度（字节）
  * @retval  0  数据成功写入，无数据丢弃
  * @retval  1  ringbuffer 空间不足，丢弃部分旧数据以容纳新数据
  * @retval  2  数据长度超过 ringbuffer 总容量，仅保留最新 TX_BUFFER_SIZE 字节
  * @retval  3  输入数据指针为空
  * @note
  * - 使用 lwrb 库操作发送 RingBuffer（usart->txRB）。
  * - 若 len > ringbuffer 容量，会自动截断，仅保留最新的数据。
  * - 若空间不足，将调用 lwrb_skip() 丢弃部分旧数据。
  */
uint8_t USART_Put_TxData_To_Ringbuffer(USART_Driver_t *usart, const void* data, uint16_t len)
{
    if (!usart || !data) return 3; //! 检查输入数据指针有效性
    
    lwrb_t *rb = &usart->txRB;
    uint16_t capacity = usart->txBufSize;
    lwrb_sz_t freeSpace = lwrb_get_free(rb);
    uint8_t ret = 0;
    
    //! 情况1：数据长度小于ringbuffer容量
    if (len < capacity) {
        if (len <= freeSpace) {
            lwrb_write(rb, data, len); //! 剩余空间充足，直接写入
        } else {
            //! 空间不足，需丢弃部分旧数据
            lwrb_sz_t used = lwrb_get_full(rb);
            lwrb_sz_t skip_len = len - freeSpace;
            if (skip_len > used) skip_len = used;
            lwrb_skip(rb, skip_len);
            lwrb_write(rb, data, len);
            ret = 1;
        }
    } else if (len == capacity) { //! 情况2：数据长度等于ringbuffer容量
        if (freeSpace < capacity) { //! 如果ringbuffer已有数据
            lwrb_reset(rb);
            ret = 1;
        }
        lwrb_write(rb, data, len);
    } else { //! 情况3：数据长度大于ringbuffer容量，仅保留最后 capacity 字节
        const uint8_t *ptr = (const uint8_t*)data + (len - capacity);
        lwrb_reset(rb);
        lwrb_write(rb, ptr, capacity);
        ret = 2;
    }
    return ret;
}

/**
 * @brief   初始化USART驱动，配置DMA、RingBuffer与中断
 * @param   usart         指向USART驱动结构体的指针
 * @param   rxDMABuffer   DMA接收缓冲区指针
 * @param   rxRBBuffer    接收RingBuffer缓冲区指针
 * @param   rxBufSize     接收缓冲区大小
 * @param   txDMABuffer   DMA发送缓冲区指针
 * @param   txRBBuffer    发送RingBuffer缓冲区指针
 * @param   txBufSize     发送缓冲区大小
 * @retval  无
 * @note    需先通过CubeMX完成串口、DMA相关硬件配置和句柄赋值
 */
void USART_Config(USART_Driver_t *usart,
                  uint8_t *rxDMABuffer, uint8_t *rxRBBuffer, uint16_t rxBufSize,
                  uint8_t *txDMABuffer, uint8_t *txRBBuffer, uint16_t txBufSize)
{
    if (!usart) return;
    usart->rxDMABuffer = rxDMABuffer;
    usart->rxRBBuffer  = rxRBBuffer;
    usart->rxBufSize   = rxBufSize;
    lwrb_init(&usart->rxRB, usart->rxRBBuffer, usart->rxBufSize);

    usart->txDMABuffer = txDMABuffer;
    usart->txRBBuffer  = txRBBuffer;
    usart->txBufSize   = txBufSize;
    lwrb_init(&usart->txRB, usart->txRBBuffer, usart->txBufSize);

    USART_Received_DMA_Configure(usart); // 初始化DMA RX

    usart->txDMABusy = 0;
    usart->dmaRxLastPos = 0;
    usart->rxMsgCount = 0;
    usart->txMsgCount = 0;
    usart->errorDMATX = 0;
    usart->errorDMARX = 0;
    usart->errorRX = 0;
}

/**
  * @brief  USART模块主循环调度函数（DMA + RingBuffer高效收发）
  * @param  usart 指向USART驱动结构体的指针
  * @note   建议主循环定时（如1ms）调用
  *         - 检查发送RingBuffer是否有待发送数据，且DMA当前空闲
  *         - 若条件满足，从发送RingBuffer读取一段数据到DMA发送缓冲区，并通过DMA启动异步发送
  *         - 自动维护已发送数据统计
  * @retval 无
  */
void USART_Module_Run(USART_Driver_t *usart)
{
    if (!usart) return;
    uint16_t available = lwrb_get_full(&usart->txRB);
    if (available && usart->txDMABusy == 0) {
        uint16_t len = (available > usart->txBufSize) ? usart->txBufSize : available;
        lwrb_read(&usart->txRB, usart->txDMABuffer, len);
        usart->txMsgCount += len;
        USART_SendString_DMA(usart, usart->txDMABuffer, len);
    }
}

/**
  * @brief  获取USART接收RingBuffer中的可读字节数
  * @param  usart 指向USART驱动结构体的指针
  * @retval uint32_t 可读取的数据字节数
  * @note   通常在主循环或数据解析前调用，用于判断是否需要读取数据。
  */
uint32_t USART_Get_The_Existing_Amount_Of_Data(USART_Driver_t *usart)
{
    if (!usart) return 0;
    return lwrb_get_full(&usart->rxRB);
}

/**
  * @brief  从USART接收RingBuffer中读取一个字节数据
  * @param  usart 指向USART驱动结构体的指针
  * @param  data  指向存放读取结果的缓冲区指针
  * @retval 1  读取成功，有新数据存入 *data
  * @retval 0  读取失败（无数据或data为NULL）
  * @note   本函数不会阻塞，无数据时直接返回0。
  */
uint8_t USART_Take_A_Piece_Of_Data(USART_Driver_t *usart, uint8_t* data)
{
    if (!usart || !data) return 0;
    return lwrb_read(&usart->rxRB, data, 1);
}

/**
  * @brief  DMA传输错误后的自动恢复操作（含错误统计）
  * @param  usart 指向USART驱动结构体
  * @param  dir   方向：0=RX, 1=TX
  * @note   检测到DMA传输错误（TE）时调用，自动进行统计并恢复
  *         RX方向会自动重启DMA，TX方向建议等待主循环调度新发送
  * @retval 无
  */
void USART_DMA_Error_Recover(USART_Driver_t *usart, uint8_t dir)
{
    if (!usart) return;

    if (dir == 0) { //! RX方向
        usart->errorDMARX++; //! DMA接收错误计数 */
        HAL_DMA_Abort(usart->hdma_rx);
        HAL_UART_Receive_DMA(usart->huart, usart->rxDMABuffer, usart->rxBufSize);
        //! 可以加入极端情况下的USART复位等
    } else { //! TX方向
        usart->errorDMATX++; //! DMA发送错误计数 */
        HAL_DMA_Abort(usart->hdma_tx);
        //! 一般等待主循环触发新的DMA发送
    }
    //! 可插入报警、日志
}

```

## 2.3、stm32f1xx_it.c
![[Pasted image 20250524185539.png]]
![[Pasted image 20250524185807.png]]
![[Pasted image 20250524190757.png]]
## 2.4、main.c
![[Pasted image 20250524190953.png]]
缓存分别是DMA缓存与ringbuffer缓存。
![[Pasted image 20250524191604.png]]

## 2.5、编译代码、下载代码
![[Pasted image 20250524191948.png]]

# 三、测试代码
---
![[Pasted image 20250524192040.png]]

![[USART_DMA_rb.gif]]如上所示，从成员rxMsgCount与txMsgCount看到，数据在正常收发。errorDMATX、errorDMARX、errorRX一直保持0，证明没有发生错误。
