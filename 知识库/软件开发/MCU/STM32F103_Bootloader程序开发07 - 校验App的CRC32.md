# 导言
---
![[Pasted image 20250529183254.png]]
本章节做一个实验“校验一遍App缓存区的固件”，看看能不能通过CRC32校验。在OTA升级流程上，当bootloader程序接收完上位机下发的App固件到App下载缓存区后，bootloader程序要对App下载缓存区的固件进行CRC32校验。
- 校验通过的话，将App下载区的固件Copy到App区，Copy完成后，跳转至App运行即可完成OTA升级。
- 校验失败的话，bootloader程序将反馈“OTA升级失败”给上位机。然后，跳转至App运行。等待下一次OTA升级的请求。

项目地址：
github: https://github.com/q164129345/MCU_Develop/tree/main/bootloader07_stm32f103_check_crc32
gitee(国内): https://gitee.com/wallace89/MCU_Develop/tree/main/bootloader07_stm32f103_check_crc32


# 一、将App_crc.bin下载到App下载缓存区
---
项目当前还没有开发跟上位机的通讯，为了验证“对App下载缓存区的固件进行CRC32的校验“。可以先使用下载器（J-LINK或者ST-LINK）将固件下载到App缓存区。

## 1.1、J-Flash将App_crc.bin下载到App下载缓存区
![[Pasted image 20250529185547.png]]
![[Pasted image 20250529190916.png]]
![[Pasted image 20250529191000.png]]
![[Pasted image 20250529191045.png]]
![[Pasted image 20250529191139.png]]
![[Pasted image 20250529191250.png]]
![[Pasted image 20250529191354.png]]
![[Pasted image 20250529191428.png]]

## 1.2、Keil的debug模式，Viewer->Memory2核对固件内容

![[Pasted image 20250529191910.png]]

# 二、bootloader程序代码
---
## 2.1、soft_crc32.c
```c
#include "soft_crc32.h"
#include <string.h>
#include <stdint.h>

/**
  * @brief  CRC32 查找表（多项式：0xEDB88320，反转算法）
  * @note   
  * - 表中共256项，对应每一个可能的字节值；
  * - 用于快速查表计算，提高CRC32效率；
  * - 本表使用标准以太网CRC32多项式（即：IEEE 802.3）生成；
  * - 生成公式：多项式为 0x04C11DB7，反转为 0xEDB88320。
  */
static const uint32_t crc32_tab[256] = {
    0x00000000u, 0x77073096u, 0xEE0E612Cu, 0x990951BAu, 0x076DC419u, 0x706AF48Fu,
    0xE963A535u, 0x9E6495A3u, 0x0EDB8832u, 0x79DCB8A4u, 0xE0D5E91Eu, 0x97D2D988u,
    0x09B64C2Bu, 0x7EB17CBDu, 0xE7B82D07u, 0x90BF1D91u, 0x1DB71064u, 0x6AB020F2u,
    0xF3B97148u, 0x84BE41DEu, 0x1ADAD47Du, 0x6DDDE4EBu, 0xF4D4B551u, 0x83D385C7u,
    0x136C9856u, 0x646BA8C0u, 0xFD62F97Au, 0x8A65C9ECu, 0x14015C4Fu, 0x63066CD9u,
    0xFA0F3D63u, 0x8D080DF5u, 0x3B6E20C8u, 0x4C69105Eu, 0xD56041E4u, 0xA2677172u,
    0x3C03E4D1u, 0x4B04D447u, 0xD20D85FDu, 0xA50AB56Bu, 0x35B5A8FAu, 0x42B2986Cu,
    0xDBBBC9D6u, 0xACBCF940u, 0x32D86CE3u, 0x45DF5C75u, 0xDCD60DCFu, 0xABD13D59u,
    0x26D930ACu, 0x51DE003Au, 0xC8D75180u, 0xBFD06116u, 0x21B4F4B5u, 0x56B3C423u,
    0xCFBA9599u, 0xB8BDA50Fu, 0x2802B89Eu, 0x5F058808u, 0xC60CD9B2u, 0xB10BE924u,
    0x2F6F7C87u, 0x58684C11u, 0xC1611DABu, 0xB6662D3Du, 0x76DC4190u, 0x01DB7106u,
    0x98D220BCu, 0xEFD5102Au, 0x71B18589u, 0x06B6B51Fu, 0x9FBFE4A5u, 0xE8B8D433u,
    0x7807C9A2u, 0x0F00F934u, 0x9609A88Eu, 0xE10E9818u, 0x7F6A0DBBu, 0x086D3D2Du,
    0x91646C97u, 0xE6635C01u, 0x6B6B51F4u, 0x1C6C6162u, 0x856530D8u, 0xF262004Eu,
    0x6C0695EDu, 0x1B01A57Bu, 0x8208F4C1u, 0xF50FC457u, 0x65B0D9C6u, 0x12B7E950u,
    0x8BBEB8EAu, 0xFCB9887Cu, 0x62DD1DDFu, 0x15DA2D49u, 0x8CD37CF3u, 0xFBD44C65u,
    0x4DB26158u, 0x3AB551CEu, 0xA3BC0074u, 0xD4BB30E2u, 0x4ADFA541u, 0x3DD895D7u,
    0xA4D1C46Du, 0xD3D6F4FBu, 0x4369E96Au, 0x346ED9FCu, 0xAD678846u, 0xDA60B8D0u,
    0x44042D73u, 0x33031DE5u, 0xAA0A4C5Fu, 0xDD0D7CC9u, 0x5005713Cu, 0x270241AAu,
    0xBE0B1010u, 0xC90C2086u, 0x5768B525u, 0x206F85B3u, 0xB966D409u, 0xCE61E49Fu,
    0x5EDEF90Eu, 0x29D9C998u, 0xB0D09822u, 0xC7D7A8B4u, 0x59B33D17u, 0x2EB40D81u,
    0xB7BD5C3Bu, 0xC0BA6CADu, 0xEDB88320u, 0x9ABFB3B6u, 0x03B6E20Cu, 0x74B1D29Au,
    0xEAD54739u, 0x9DD277AFu, 0x04DB2615u, 0x73DC1683u, 0xE3630B12u, 0x94643B84u,
    0x0D6D6A3Eu, 0x7A6A5AA8u, 0xE40ECF0Bu, 0x9309FF9Du, 0x0A00AE27u, 0x7D079EB1u,
    0xF00F9344u, 0x8708A3D2u, 0x1E01F268u, 0x6906C2FEu, 0xF762575Du, 0x806567CBu,
    0x196C3671u, 0x6E6B06E7u, 0xFED41B76u, 0x89D32BE0u, 0x10DA7A5Au, 0x67DD4ACCu,
    0xF9B9DF6Fu, 0x8EBEEFF9u, 0x17B7BE43u, 0x60B08ED5u, 0xD6D6A3E8u, 0xA1D1937Eu,
    0x38D8C2C4u, 0x4FDFF252u, 0xD1BB67F1u, 0xA6BC5767u, 0x3FB506DDu, 0x48B2364Bu,
    0xD80D2BDAu, 0xAF0A1B4Cu, 0x36034AF6u, 0x41047A60u, 0xDF60EFC3u, 0xA867DF55u,
    0x316E8EEFu, 0x4669BE79u, 0xCB61B38Cu, 0xBC66831Au, 0x256FD2A0u, 0x5268E236u,
    0xCC0C7795u, 0xBB0B4703u, 0x220216B9u, 0x5505262Fu, 0xC5BA3BBEu, 0xB2BD0B28u,
    0x2BB45A92u, 0x5CB36A04u, 0xC2D7FFA7u, 0xB5D0CF31u, 0x2CD99E8Bu, 0x5BDEAE1Du,
    0x9B64C2B0u, 0xEC63F226u, 0x756AA39Cu, 0x026D930Au, 0x9C0906A9u, 0xEB0E363Fu,
    0x72076785u, 0x05005713u, 0x95BF4A82u, 0xE2B87A14u, 0x7BB12BAEu, 0x0CB61B38u,
    0x92D28E9Bu, 0xE5D5BE0Du, 0x7CDCEFB7u, 0x0BDBDF21u, 0x86D3D2D4u, 0xF1D4E242u,
    0x68DDB3F8u, 0x1FDA836Eu, 0x81BE16CDu, 0xF6B9265Bu, 0x6FB077E1u, 0x18B74777u,
    0x88085AE6u, 0xFF0F6A70u, 0x66063BCAu, 0x11010B5Cu, 0x8F659EFFu, 0xF862AE69u,
    0x616BFFD3u, 0x166CCF45u, 0xA00AE278u, 0xD70DD2EEu, 0x4E048354u, 0x3903B3C2u,
    0xA7672661u, 0xD06016F7u, 0x4969474Du, 0x3E6E77DBu, 0xAED16A4Au, 0xD9D65ADCu,
    0x40DF0B66u, 0x37D83BF0u, 0xA9BCAE53u, 0xDEBB9EC5u, 0x47B2CF7Fu, 0x30B5FFE9u,
    0xBDBDF21Cu, 0xCABAC28Au, 0x53B39330u, 0x24B4A3A6u, 0xBAD03605u, 0xCDD70693u,
    0x54DE5729u, 0x23D967BFu, 0xB3667A2Eu, 0xC4614AB8u, 0x5D681B02u, 0x2A6F2B94u,
    0xB40BBE37u, 0xC30C8EA1u, 0x5A05DF1Bu, 0x2D02EF8Du
};

/**
  * @brief   CRC32 数据块更新函数（软件实现）
  * @param   crc  当前CRC值（可为上一次中间结果）
  * @param   buf  指向待处理数据缓冲区
  * @param   len  数据长度（单位：字节）
  * @retval  更新后的CRC32值
  * @note    
  * - 基于查表法实现，效率较高；
  * - 可用于分块计算、流式处理；
  * - 该函数不进行补码处理（XOR输出需外部完成）。
  */
uint32_t CRC32_Update(uint32_t crc, const void *buf, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)buf;
    while (len--)
        crc = crc32_tab[(crc ^ *p++) & 0xFFu] ^ (crc >> 8);
    return crc;
}

/**
  * @brief   计算Flash中固件数据的CRC32值（软件方式）
  * @param   flash_addr Flash起始地址（通常为固件起始地址）
  * @param   total_len  固件总长度（注意！！不包含最后的CRC32校验码）
  * @retval  计算出的CRC32值（已完成反码）
  * @note    
  * - 本函数用于Bootloader自检或烧录验证；
  * - 默认按块处理（每块256字节），支持未来DMA/多段扩展；
  * - 初始值为0xFFFFFFFF，最终异或输出结果。
  */
uint32_t Calculate_Firmware_CRC32_SW(uint32_t flash_addr, uint32_t total_len)
{
    uint32_t crc = 0xFFFFFFFFu;
    const uint8_t *p = (const uint8_t *)flash_addr;

    /* 按 256 B 块处理，可边走边校验；此处简单循环 */
    while (total_len)
    {
        uint32_t chunk = (total_len > 256u) ? 256u : total_len;
        crc = CRC32_Update(crc, p, chunk);
        p   += chunk;
        total_len -= chunk;
    }
    return crc ^ 0xFFFFFFFFu;
}

```

## 2.2、soft_crc32.h
```c
/**
 * @file    soft_crc32.h
 * @brief   纯软件 CRC-32/ISO-HDLC 计算（RefIn/RefOut = 1）
 * @author  Wallace
 * @date    2025-05-29
 * @version 1.0.0
 */

#ifndef __SOFT_CRC32_H
#define __SOFT_CRC32_H

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   初始化 CRC 上下文
 * @return  初始值 0xFFFF?FFFF
 */
static inline uint32_t CRC32_Init(void) { return 0xFFFFFFFFu; }

/**
 * @brief   输入若干字节并更新 CRC
 */
uint32_t CRC32_Update(uint32_t crc, const void *buf, uint32_t len);

/**
 * @brief   计算完成后取反得到最终 CRC
 */
static inline uint32_t CRC32_Final(uint32_t crc) { return crc ^ 0xFFFFFFFFu; }

/**
 * @brief   便捷接口：一次性计算整个缓冲区
 */
static inline uint32_t CRC32_Calc(const void *buf, uint32_t len)
{
    return CRC32_Final(CRC32_Update(CRC32_Init(), buf, len));
}

/**
  * @brief   计算Flash中固件数据的CRC32值（软件方式）
  */
uint32_t Calculate_Firmware_CRC32_SW(uint32_t flash_addr, uint32_t total_len);

#ifdef __cplusplus
}
#endif
#endif /* __SOFT_CRC32_H */

```

## 2.3、main.c
![[Pasted image 20250529192508.png | 1100]]
![[Pasted image 20250530091816.png | 1100]]
![[Pasted image 20250529192559.png | 1100]]

# 三、测试代码
---
![[Pasted image 20250529192850.png]]
CRC32校验通过，App下载区的固件完整性没有问题。






