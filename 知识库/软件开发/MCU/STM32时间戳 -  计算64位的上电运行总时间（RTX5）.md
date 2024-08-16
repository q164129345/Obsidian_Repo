## 导言
---
![[Pasted image 20240816141537.png]]
如上图所示，RTX5提供函数**`osKernelGetTickCount()`** 返回 kernel tick count给我们使用。例如，我们设置kernel的频率1000Hz的话，它将返回1m的时间戳。

**但是，这一点非常值得注意，返回值是uint32_t。例如，kernel频率是1000Hz时，返回值将在2^32ms后溢出，重新从0开始递增。大概会在第49天时溢出。**
## 一、解决溢出

---
![[Pasted image 20240816141731.png]]
其实，RTX5官方也给出相应的解决方案。

```c
#include "cmsis_os2.h"  // 引入CMSIS-RTOS2 API用于RTX5

uint64_t GetTick(void) {
    static uint32_t tick_h = 0U; // 高32位的tick值
    static uint32_t tick_l = 0U; // 上一次的低32位tick值
    uint32_t tick; // 当前的低32位tick值
    
    tick = osKernelGetTickCount(); // 获取当前的tick计数
    
    if (tick < tick_l) {
        // 如果当前tick小于上一次的tick，意味着32位的tick计数器已经回绕
        tick_h++; // 高32位计数器加1
    }
    
    tick_l = tick; // 更新上一次的tick值
    
    // 合并高低32位，得到64位的tick值
    return (((uint64_t)tick_h << 32) | tick_l);
}
```

代码很简单。但是，这段代码还不能直接使用在项目上。**需要确保GetTick函数对多线程访问是安全的，因为在RTOS环境中，不同的线程或中断服务程序可能会同时调用它。**

## 二、解决多线程访问安全
----
```c
/** 需要确保GetTick函数对多线程访问是安全的。
    因为在RTOS环境中，不同的线程或中断服务程序可能会同时调用它。
*/
#include "cmsis_os2.h"

osMutexId_t mutex_id; // 定义一个互斥量的ID

// 在系统初始化阶段调用此函数来创建互斥量
void InitializeMutex(void) {
    mutex_id = osMutexNew(NULL);
}

uint64_t GetTick(void) {
    static uint32_t tick_h = 0U;
    static uint32_t tick_l = 0U;
    uint32_t tick;
    
    // 获取互斥量以确保访问的原子性
    osMutexAcquire(mutex_id, osWaitForever);
    
    tick = osKernelGetTickCount(); // 获取os的32位相对时间戳
    if (tick < tick_l) {
        tick_h++;
    }
    tick_l = tick;
    
    // 释放互斥量
    osMutexRelease(mutex_id);
    
    return (((uint64_t)tick_h << 32) | tick_l);
}
```