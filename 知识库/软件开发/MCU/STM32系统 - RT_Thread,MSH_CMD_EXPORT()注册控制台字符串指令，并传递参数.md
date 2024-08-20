# 一、前言
---
注册控制台字符串指令可以非常方便地调试程序。
只需使用宏MSH_CMD_EXPORT()去注册一下即可。指令就是函数名。例如：
![[Pasted image 20240820203658.png]]
![[Pasted image 20240820203716.png]]

# 二、MSH_CMD_EXPORT()
---
`MSH_CMD_EXPORT` 是 RT-Thread 中的一个宏，用于将命令注册到 FinSH 或 MSH（模块 Shell）命令行解释器。这使得你可以创建自定义的 shell 命令，并能在 FinSH 或 MSH 提供的命令行界面中直接执行这些命令。以下是使用 `MSH_CMD_EXPORT` 的基本步骤：
### 使用 MSH_CMD_EXPORT 的步骤
1. **包含必要的头文件**： 确保在源代码中包含所需的头文件。通常需要包含 `finsh.h` 或 `shell.h`，具体取决于你使用的 RT-Thread 版本和配置。
    ```c
    #include <rtthread.h>
    #include <finsh.h> 
    
    ```
2. **定义你的命令函数**： 定义一个函数，当命令被调用时将执行此函数。这个函数应遵循 FinSH 或 MSH 期望的标准格式。
    ```c
    void my_command(int argc, char** argv)
    {
        if (argc == 1)
        {
            rt_kprintf("Hello, RT-Thread!\\n");
        }
        else
        {
            rt_kprintf("Usage: my_command\\n");
        }
    }
    
    ```
3. **导出命令**： 使用 `MSH_CMD_EXPORT` 宏将你的命令注册到 FinSH 或 MSH shell 中。这个宏需要两个参数：函数名和命令字符串。
    ```c
    MSH_CMD_EXPORT(my_command, my_command);
    ```
### 使用示例
以下是一个完整的示例，展示如何定义和导出一个简单的命令：
```c
#include <rtthread.h>#include <finsh.h>   // 或者 #include <shell.h> 对于 MSH// 定义命令函数
void my_command(int argc, char** argv)
{
    if (argc == 1)
    {
        rt_kprintf("Hello, RT-Thread!\\n");
    }
    else
    {
        rt_kprintf("Usage: my_command\\n");
    }
}

// 导出命令
MSH_CMD_EXPORT(my_command, my_command);
```
### 运行命令
在将固件编译并烧录到芯片之后，可以打开 FinSH 或 MSH 控制台并输入你的命令：
```mathematica
msh /> my_command
Hello, RT-Thread!
```
这样，你就可以创建自定义命令来扩展 RT-Thread 中 shell 环境的功能。
### `argc` 和 `argv` 的意义
在 C 语言中，命令行参数通常通过 `argc` 和 `argv` 传递给 `main` 函数。在 RT-Thread 中的命令行函数（例如使用 FinSH 或 MSH 的命令），`argc` 和 `argv` 的意义类似。
- `argc`（argument count）：表示传递给命令的参数个数。
- `argv`（argument vector）：是一个指向字符串数组的指针，每个字符串是一个参数。
### 示例解释
```c
void my_command(int argc, char** argv)
{
    if (argc == 1)
    {
        rt_kprintf("Hello, RT-Thread!\\n");
    }
    else
    {
        rt_kprintf("Usage: my_command\\n");
    }
}
```
在上面的代码中：
- 当 `argc == 1` 时，表示没有传递任何参数（除了命令本身）。
    - 例如，如果你在控制台中输入 `my_command`，`argc` 会是 1，因为只有一个命令名 `my_command`。
    - 此时，函数将打印 `Hello, RT-Thread!`。
- 当 `argc == 2` 时，表示传递了一个参数。
    - 例如，如果你在控制台中输入 `my_command arg1`，`argc` 会是 2，因为有两个字符串：`my_command` 和 `arg1`。
    - 此时，函数将打印 `Usage: my_command`，因为在上面的代码逻辑中，`argc` 不等于 1 时，函数打印用法信息。
### 更详细的示例
假设你希望处理更多的参数，可以这样写：
```c
void my_command(int argc, char** argv)
{
    if (argc == 1)
    {
        rt_kprintf("Hello, RT-Thread!\\n");
    }
    else if (argc == 2)
    {
        rt_kprintf("Argument: %s\\n", argv[1]);
    }
    else
    {
        rt_kprintf("Usage: my_command [arg1]\\n");
    }
}
```
在这个例子中：
- 如果只输入 `my_command`，将打印 `Hello, RT-Thread!`。
- 如果输入 `my_command arg1`，将打印 `Argument: arg1`。
- 如果输入 `my_command arg1 arg2` 或更多参数，将打印 `Usage: my_command [arg1]`。
### 运行示例
假设在控制台输入以下命令：
```makefile
makefile复制代码
msh /> my_command
Hello, RT-Thread!

msh /> my_command arg1
Argument: arg1

msh /> my_command arg1 arg2
Usage: my_command [arg1]

```
这样，你可以根据传递的参数数量和内容，自定义命令的行为。





