---
annotation-target: "[[Cortex-M3技术参考手册(TRM)..pdf]]"
---


>%%
>```annotation-json
>{"text":"处理模式主要用于：\n1. 中断服务程序（ISR）。\n2. 异常处理（HardFault、MemManage等）。\n处理模式始终是特权模式，可以访问所有系统资源。\n3. 且处理模式强制使用MSP堆栈。","target":[{"source":"vault:/%E7%94%B5%E5%AD%90%E4%B9%A6/Cortex-M3%E6%8A%80%E6%9C%AF%E5%8F%82%E8%80%83%E6%89%8B%E5%86%8C(TRM)..pdf","selector":[{"type":"TextPositionSelector","start":45084,"end":45088},{"type":"TextQuoteSelector","exact":"处理模式","prefix":"代码能够在线程模式下运行。 z 出现异常时处理器进入处理模式，在","suffix":"中，所有代码都是特权访问的。 2.1.2 工作状态 Cortex"}]}],"created":"2025-02-17T03:50:36.173Z","updated":"2025-02-17T03:50:36.173Z","document":{"title":"Microsoft Word - Cortex-M3技术参考手册.doc","link":[{"href":"urn:x-pdf:2a8543ad58433a814357c5c8db4dce7d"},{"href":"vault:/%E7%94%B5%E5%AD%90%E4%B9%A6/Cortex-M3%E6%8A%80%E6%9C%AF%E5%8F%82%E8%80%83%E6%89%8B%E5%86%8C(TRM)..pdf"}],"documentFingerprint":"2a8543ad58433a814357c5c8db4dce7d"},"uri":"vault:/%E7%94%B5%E5%AD%90%E4%B9%A6/Cortex-M3%E6%8A%80%E6%9C%AF%E5%8F%82%E8%80%83%E6%89%8B%E5%86%8C(TRM)..pdf"}
>```
>%%
>*%%PREFIX%%代码能够在线程模式下运行。 z 出现异常时处理器进入处理模式，在%%HIGHLIGHT%% ==处理模式== %%POSTFIX%%中，所有代码都是特权访问的。 2.1.2 工作状态 Cortex*
>%%LINK%%[[#^f8p08eatbgt|show annotation]]
>%%COMMENT%%
>处理模式主要用于：
>1. 中断服务程序（ISR）。
>2. 异常处理（HardFault、MemManage等）。
>处理模式始终是特权模式，可以访问所有系统资源。
>3. 且处理模式强制使用MSP堆栈。
>%%TAGS%%
>
^f8p08eatbgt



>%%
>```annotation-json
>{"created":"2025-02-17T07:41:34.970Z","text":"1. 裸机开发中，线程模式通常不使用非特权模式，因为整个系统由开发者完全控制，没有多任务或多用户的环境。\n2. 在我看来，特权模式像Linux的root权限。\n3. 非特权模式下，很多底层寄存器都不允许直接访问了。我尝试在例程中让程序进入非特权模式并开启PSP，当程序访问SysTick时，会进入硬件异常中断。因为非特权模式不允许访问SysTick寄存器。","updated":"2025-02-17T07:41:34.970Z","document":{"title":"Microsoft Word - Cortex-M3技术参考手册.doc","link":[{"href":"urn:x-pdf:2a8543ad58433a814357c5c8db4dce7d"},{"href":"vault:/%E7%94%B5%E5%AD%90%E4%B9%A6/Cortex-M3%E6%8A%80%E6%9C%AF%E5%8F%82%E8%80%83%E6%89%8B%E5%86%8C(TRM)..pdf"}],"documentFingerprint":"2a8543ad58433a814357c5c8db4dce7d"},"uri":"vault:/%E7%94%B5%E5%AD%90%E4%B9%A6/Cortex-M3%E6%8A%80%E6%9C%AF%E5%8F%82%E8%80%83%E6%89%8B%E5%86%8C(TRM)..pdf","target":[{"source":"vault:/%E7%94%B5%E5%AD%90%E4%B9%A6/Cortex-M3%E6%8A%80%E6%9C%AF%E5%8F%82%E8%80%83%E6%89%8B%E5%86%8C(TRM)..pdf","selector":[{"type":"TextPositionSelector","start":45024,"end":45028},{"type":"TextQuoteSelector","exact":"线程模式","prefix":"器支持两种工作模式，线程模式和处理模式： z 在复位时处理器进入","suffix":"，异常返回时也会进入该模式。特权和用户（非特权）代码能够在线程模"}]}]}
>```
>%%
>*%%PREFIX%%器支持两种工作模式，线程模式和处理模式： z 在复位时处理器进入%%HIGHLIGHT%% ==线程模式== %%POSTFIX%%，异常返回时也会进入该模式。特权和用户（非特权）代码能够在线程模*
>%%LINK%%[[#^7c967xljyns|show annotation]]
>%%COMMENT%%
>1. 裸机开发中，线程模式通常不使用非特权模式，因为整个系统由开发者完全控制，没有多任务或多用户的环境。
>2. 在我看来，特权模式像Linux的root权限。
>3. 非特权模式下，很多底层寄存器都不允许直接访问了。我尝试在例程中让程序进入非特权模式并开启PSP，当程序访问SysTick时，会进入硬件异常中断。因为非特权模式不允许访问SysTick寄存器。
>%%TAGS%%
>
^7c967xljyns


>%%
>```annotation-json
>{"created":"2025-02-17T08:33:42.685Z","text":"主堆栈（Main Stack）指的是MSP（Main Stack Pointer）\n进程堆栈（Process Stack）指的是PSP（Process Stack Pointer）\n\nRTOS任务运行时（线程模式）：\n堆栈指针：使用PSP（进程堆栈指针）。\n原因：\n每个任务有独立的堆栈空间，RTOS通过切换PSP来实现任务切换。\n使用PSP可以隔离不同任务的堆栈，避免任务之间的栈空间冲突，增强系统稳定性和安全性。\n操作：\n在任务切换时，RTOS会保存当前任务的PSP值，并加载下一个任务的PSP值。\n例如，FreeRTOS或uC/OS-II在上下文切换时，会通过汇编代码直接操作PSP。","updated":"2025-02-17T08:33:42.685Z","document":{"title":"Microsoft Word - Cortex-M3技术参考手册.doc","link":[{"href":"urn:x-pdf:2a8543ad58433a814357c5c8db4dce7d"},{"href":"vault:/%E7%94%B5%E5%AD%90%E4%B9%A6/Cortex-M3%E6%8A%80%E6%9C%AF%E5%8F%82%E8%80%83%E6%89%8B%E5%86%8C(TRM)..pdf"}],"documentFingerprint":"2a8543ad58433a814357c5c8db4dce7d"},"uri":"vault:/%E7%94%B5%E5%AD%90%E4%B9%A6/Cortex-M3%E6%8A%80%E6%9C%AF%E5%8F%82%E8%80%83%E6%89%8B%E5%86%8C(TRM)..pdf","target":[{"source":"vault:/%E7%94%B5%E5%AD%90%E4%B9%A6/Cortex-M3%E6%8A%80%E6%9C%AF%E5%8F%82%E8%80%83%E6%89%8B%E5%86%8C(TRM)..pdf","selector":[{"type":"TextPositionSelector","start":45784,"end":45794},{"type":"TextQuoteSelector","exact":"主堆栈切换到进程堆栈","prefix":"中，使用 MSR 指令对 CONTROL[1]执行写操作也可以从","suffix":"。 2.3 寄存器 Cortex-M3 处理器有以下 32 位寄"}]}]}
>```
>%%
>*%%PREFIX%%中，使用 MSR 指令对 CONTROL[1]执行写操作也可以从%%HIGHLIGHT%% ==主堆栈切换到进程堆栈== %%POSTFIX%%。 2.3 寄存器 Cortex-M3 处理器有以下 32 位寄*
>%%LINK%%[[#^y219kldcptk|show annotation]]
>%%COMMENT%%
>主堆栈（Main Stack）指的是MSP（Main Stack Pointer）
>进程堆栈（Process Stack）指的是PSP（Process Stack Pointer）
>
>RTOS任务运行时（线程模式）：
>堆栈指针：使用PSP（进程堆栈指针）。
>原因：
>每个任务有独立的堆栈空间，RTOS通过切换PSP来实现任务切换。
>使用PSP可以隔离不同任务的堆栈，避免任务之间的栈空间冲突，增强系统稳定性和安全性。
>操作：
>在任务切换时，RTOS会保存当前任务的PSP值，并加载下一个任务的PSP值。
>例如，FreeRTOS或uC/OS-II在上下文切换时，会通过汇编代码直接操作PSP。
>%%TAGS%%
>
^y219kldcptk


>%%
>```annotation-json
>{"created":"2025-02-17T08:52:36.774Z","updated":"2025-02-17T08:52:36.774Z","document":{"title":"Microsoft Word - Cortex-M3技术参考手册.doc","link":[{"href":"urn:x-pdf:2a8543ad58433a814357c5c8db4dce7d"},{"href":"vault:/%E7%94%B5%E5%AD%90%E4%B9%A6/Cortex-M3%E6%8A%80%E6%9C%AF%E5%8F%82%E8%80%83%E6%89%8B%E5%86%8C(TRM)..pdf"}],"documentFingerprint":"2a8543ad58433a814357c5c8db4dce7d"},"uri":"vault:/%E7%94%B5%E5%AD%90%E4%B9%A6/Cortex-M3%E6%8A%80%E6%9C%AF%E5%8F%82%E8%80%83%E6%89%8B%E5%86%8C(TRM)..pdf","target":[{"source":"vault:/%E7%94%B5%E5%AD%90%E4%B9%A6/Cortex-M3%E6%8A%80%E6%9C%AF%E5%8F%82%E8%80%83%E6%89%8B%E5%86%8C(TRM)..pdf","selector":[{"type":"TextPositionSelector","start":46262,"end":46310},{"type":"TextQuoteSelector","exact":"处理模式始终使用 SP_main，而线程模式可配置为 SP_main 或 SP_process。","prefix":"了写入位[1:0]的值，因此它自动与字，即 4 字节边界对齐。 ","suffix":" 链接寄存器 寄存器 r14 是子程序的链接寄存器（LR）。 在"}]}]}
>```
>%%
>*%%PREFIX%%了写入位[1:0]的值，因此它自动与字，即 4 字节边界对齐。%%HIGHLIGHT%% ==处理模式始终使用 SP_main，而线程模式可配置为 SP_main 或 SP_process。== %%POSTFIX%%链接寄存器 寄存器 r14 是子程序的链接寄存器（LR）。 在*
>%%LINK%%[[#^zk020dn9y4d|show annotation]]
>%%COMMENT%%
>
>%%TAGS%%
>
^zk020dn9y4d


>%%
>```annotation-json
>{"created":"2025-02-17T09:08:02.837Z","text":"线程模式用于普通的应用程序代码执行，而处理模式则用于中断服务例程（ISR）和系统异常处理。\n\nRTOS通过在线程模式下使用非特权状态来隔离用户任务，确保任务之间和任务与内核之间的安全性和稳定性。比如，FreeRTOS、uC/OS等RTOS可以利用非特权模式来保护内核资源。\n\n裸机开发通常不涉及任务分割和安全隔离的需求，直接在特权状态下运行所有代码。","updated":"2025-02-17T09:08:02.837Z","document":{"title":"Microsoft Word - Cortex-M3技术参考手册.doc","link":[{"href":"urn:x-pdf:2a8543ad58433a814357c5c8db4dce7d"},{"href":"vault:/%E7%94%B5%E5%AD%90%E4%B9%A6/Cortex-M3%E6%8A%80%E6%9C%AF%E5%8F%82%E8%80%83%E6%89%8B%E5%86%8C(TRM)..pdf"}],"documentFingerprint":"2a8543ad58433a814357c5c8db4dce7d"},"uri":"vault:/%E7%94%B5%E5%AD%90%E4%B9%A6/Cortex-M3%E6%8A%80%E6%9C%AF%E5%8F%82%E8%80%83%E6%89%8B%E5%86%8C(TRM)..pdf","target":[{"source":"vault:/%E7%94%B5%E5%AD%90%E4%B9%A6/Cortex-M3%E6%8A%80%E6%9C%AF%E5%8F%82%E8%80%83%E6%89%8B%E5%86%8C(TRM)..pdf","selector":[{"type":"TextPositionSelector","start":44980,"end":45011},{"type":"TextQuoteSelector","exact":"Cortex-M3 处理器支持两种工作模式，线程模式和处理模式","prefix":"用 16/32 位指令，提供更高的性能。 2.1.1 工作模式 ","suffix":"： z 在复位时处理器进入线程模式，异常返回时也会进入该模式。特"}]}]}
>```
>%%
>*%%PREFIX%%用 16/32 位指令，提供更高的性能。 2.1.1 工作模式%%HIGHLIGHT%% ==Cortex-M3 处理器支持两种工作模式，线程模式和处理模式== %%POSTFIX%%： z 在复位时处理器进入线程模式，异常返回时也会进入该模式。特*
>%%LINK%%[[#^ecwjitp5aj|show annotation]]
>%%COMMENT%%
>线程模式用于普通的应用程序代码执行，而处理模式则用于中断服务例程（ISR）和系统异常处理。
>
>RTOS通过在线程模式下使用非特权状态来隔离用户任务，确保任务之间和任务与内核之间的安全性和稳定性。比如，FreeRTOS、uC/OS等RTOS可以利用非特权模式来保护内核资源。
>
>裸机开发通常不涉及任务分割和安全隔离的需求，直接在特权状态下运行所有代码。
>%%TAGS%%
>
^ecwjitp5aj


>%%
>```annotation-json
>{"created":"2025-02-17T09:38:03.677Z","text":"在core_cm3.h的源码，第1383行看到：\n\n> #define SCS_BASE            (0xE000E000UL)\n> ...\n> #define SysTick_BASE        (SCS_BASE +  0x0010UL)\n\n相当于SysTick_BASE = 0xE000E010。\n\n在core_cm3.h的源码，第694行看到：\n> typedef struct\n> {\n>     __IOM uint32_t CTRL;\n>     __IOM uint32_t LOAD;\n>     __IOM uint32_t VAL;\n>     __IM  uint32_t CALIB;\n> };\n从源码看到，SysTick确认只有4个寄存器，且大小是4个字节。\n\n\n","updated":"2025-02-17T09:38:03.677Z","document":{"title":"Microsoft Word - Cortex-M3技术参考手册.doc","link":[{"href":"urn:x-pdf:2a8543ad58433a814357c5c8db4dce7d"},{"href":"vault:/%E7%94%B5%E5%AD%90%E4%B9%A6/Cortex-M3%E6%8A%80%E6%9C%AF%E5%8F%82%E8%80%83%E6%89%8B%E5%86%8C(TRM)..pdf"}],"documentFingerprint":"2a8543ad58433a814357c5c8db4dce7d"},"uri":"vault:/%E7%94%B5%E5%AD%90%E4%B9%A6/Cortex-M3%E6%8A%80%E6%9C%AF%E5%8F%82%E8%80%83%E6%89%8B%E5%86%8C(TRM)..pdf","target":[{"source":"vault:/%E7%94%B5%E5%AD%90%E4%B9%A6/Cortex-M3%E6%8A%80%E6%9C%AF%E5%8F%82%E8%80%83%E6%89%8B%E5%86%8C(TRM)..pdf","selector":[{"type":"TextPositionSelector","start":52179,"end":52186},{"type":"TextQuoteSelector","exact":"SysTick","prefix":"  复位值 中断控制类型寄存器 只读 0xE000E004 a ","suffix":" 控制和状态寄存器 读/写 0xE000E010 0x00000"}]}]}
>```
>%%
>*%%PREFIX%%复位值 中断控制类型寄存器 只读 0xE000E004 a%%HIGHLIGHT%% ==SysTick== %%POSTFIX%%控制和状态寄存器 读/写 0xE000E010 0x00000*
>%%LINK%%[[#^0o3hwohmvp2|show annotation]]
>%%COMMENT%%
>在core_cm3.h的源码，第1383行看到：
>
>> #define SCS_BASE            (0xE000E000UL)
>> ...
>> #define SysTick_BASE        (SCS_BASE +  0x0010UL)
>
>相当于SysTick_BASE = 0xE000E010。
>
>在core_cm3.h的源码，第694行看到：
>> typedef struct
>> {
>>     __IOM uint32_t CTRL;
>>     __IOM uint32_t LOAD;
>>     __IOM uint32_t VAL;
>>     __IM  uint32_t CALIB;
>> };
>从源码看到，SysTick确认只有4个寄存器，且大小是4个字节。
>
>
>
>%%TAGS%%
>
^0o3hwohmvp2
