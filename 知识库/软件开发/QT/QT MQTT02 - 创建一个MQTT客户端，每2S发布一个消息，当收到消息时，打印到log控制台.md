# 一、前言
---
代码的效果如下：
![[录屏 07-04-2024 11_30_07 AM (1).webm]]

# 二、代码
---
```cpp
#include <QCoreApplication>
#include <QMqttClient>
#include <QDebug>
#include <QTimer>

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    QMqttClient mqttClient; // 创建QMqttClient对象，用于与MQTT代理通信
    mqttClient.setHostname("127.0.0.1"); // MQTT Broker地址
    mqttClient.setPort(1883); // MQTT代理端口
    mqttClient.setClientId("QtClientTest"); // 设置客户端ID

    // 连接状态改变时输出状态信息
    QObject::connect(&mqttClient, &QMqttClient::stateChanged, [](QMqttClient::ClientState state) {
        qDebug() << "Client state changed:" << state;
    });

    // 接收到消息时输出消息内容和主题
    QObject::connect(&mqttClient, &QMqttClient::messageReceived, [](const QByteArray &message, const QMqttTopicName &topic) {
        qDebug() << "Message received:" << message << "from topic:" << topic.name();
    });

    // 成功连接到MQTT代理时进行订阅和定时发布消息
    QObject::connect(&mqttClient, &QMqttClient::connected, &mqttClient, [&mqttClient](){
        qDebug() << "Connected to broker";

        // 订阅主题
        QMqttSubscription *subscription = mqttClient.subscribe(QMqttTopicFilter("test/topic"), 0);
        if (!subscription) {
            qDebug() << "Could not subscribe. Is there a valid connect";
        } else {
            qDebug() << "Subscription successful";
        }

        // 创建QTimer对象并设定定时发布消息
        QTimer *timer = new QTimer(&mqttClient);
        QObject::connect(timer, &QTimer::timeout, &mqttClient, [&mqttClient](){
            mqttClient.publish(QMqttTopicName("test/Hello"), "Hello, World");
        });
        timer->start(2000); // 每隔2秒发布一次消息
    });

    mqttClient.connectToHost(); // 连接到MQTT代理
    return a.exec(); // 进入事件循环，程序开始运行，并处理各种事件
}

```

## 2.1、a.exec()
`return a.exec();` 并不代表程序退出，而是进入事件循环。它是 Qt 应用程序的主事件循环的一部分，负责处理和调度事件，例如定时器事件、网络事件和用户界面事件。在你的控制台应用程序中，`QCoreApplication` 的事件循环确保了 MQTT 客户端能够正常工作，并处理来自 MQTT 代理的消息。
简单来说，`a.exec()` 是让程序一直运行并处理事件，而不是退出程序。
在你的程序中，完整的流程如下：
1. 创建 `QCoreApplication` 对象。
2. 设置 MQTT 客户端参数，如主机名、端口和客户端 ID。
3. 连接 MQTT 客户端的各种信号以处理状态更改、消息接收和连接成功等事件。
4. 调用 `mqttClient.connectToHost()` 连接到 MQTT 代理。
5. 调用 `a.exec()` 进入事件循环，程序开始运行，并处理各种事件。

如果没有 `a.exec()`，程序会在执行完 `mqttClient.connectToHost()` 后立即退出，因为没有进入事件循环来处理异步事件。`a.exec()` 保证了程序会持续运行，并响应定时器事件和来自 MQTT 代理的消息。
## 2.2、QObject::connect(&mqttClient, &QMqttClient::messageReceived, [](const QByteArray &message, const QMqttTopicName &topic)
以下是官方的API文档：
![[Pasted image 20240918192714.png]]

```cpp
// 接收到消息时输出消息内容和主题
QObject::connect(&mqttClient, &QMqttClient::messageReceived, [](const QByteArray &message, const QMqttTopicName &topic) {
    qDebug() << "Message received:" << message << "from topic:" << topic.name();
});
```

以上的代码实际上使用方法`connect(const QObject *sender, PointerToMemberFunction signal, Functor functor)` 。

这是一个重载的`QObject::connect`方法，它接收三个参数：

1. `const QObject *sender`：信号的发送者对象，这里是`&mqttClient`。
2. `PointerToMemberFunction signal`：信号的成员函数指针，这里是`&QMqttClient::messageReceived`。
3. `Functor functor`：槽函数，这里是一个lambda函数`[](const QByteArray &message, const QMqttTopicName &topic)`。

这个重载的方法允许你将信号连接到一个lambda函数或者其它的仿函数（functor）。这在使用C++11及以后的版本时非常常见，因为lambda函数提供了一种简洁和灵活的方式来定义槽函数。

关于C++的lambda函数具体用法：

我稍微把代码改一下，将lambda函数改为一般函数的写法。使用lambda函数来写槽函数，确实比较简洁，不需要另外再编写一个函数。
![[Pasted image 20240918192743.png]]


## 2.3、信号与槽 - QObject::connect()
总的来说，程序有三个回调的槽函数需要编写。
1. 连接状态改变时输出状态信息
    
    ```cpp
     // 连接状态改变时输出状态信息
     QObject::connect(&mqttClient, &QMqttClient::stateChanged, [](QMqttClient::ClientState state) {
         qDebug() << "Client state changed:" << state;
     });
    ```
    
2. 接收到消息时输出消息内容和主题
    ```cpp
    //接收到消息时输出消息内容和主题
    QObject::connect(&mqttClient, &QMqttClient::messageReceived, [](const QByteArray &message, const QMqttTopicName &topic) {
         qDebug() << "Message received:" << message << "from topic:" << topic.name();
    });
    ```
    
3. 成功连接到MQTT代理时
    ```cpp
    // 成功连接到MQTT代理时进行订阅和定时发布消息
    QObject::connect(&mqttClient, &QMqttClient::connected, &mqttClient, [&mqttClient](){
        qDebug() << "Connected to broker";
    
        // 订阅主题
        QMqttSubscription *subscription = mqttClient.subscribe(QMqttTopicFilter("test/topic"), 0);
        if (!subscription) {
            qDebug() << "Could not subscribe. Is there a valid connect";
        } else {
            qDebug() << "Subscription successful";
        }
    
        // 创建QTimer对象并设定定时发布消息
        QTimer *timer = new QTimer(&mqttClient);
        QObject::connect(timer, &QTimer::timeout, &mqttClient, [&mqttClient](){
            mqttClient.publish(QMqttTopicName("test/Hello"), "Hello, World");
        });
        timer->start(2000); // 每隔2秒发布一次消息
    });
    ```
    

### 2.4、接收十六进制数据
---
为了更加高效地通讯，在一些自动化项目、机器人项目上一般使用十六进制。
**字符串格式（String Format）**
**特点**：
1. **可读性强**：字符串格式是人类可读的，便于调试和分析。例如，消息内容为`"Hello World"`。
2. **灵活性高**：字符串可以包含各种字符，可以方便地进行文本处理和解析。

**优势**：
1. **易于理解和调试**：开发和调试过程中，字符串格式的消息更直观，容易检测和理解。
2. **支持多语言**：可以方便地包含和处理各种自然语言文本。

**十六进制格式（Hex Format）**
**特点**：
1. **紧凑性**：十六进制格式的数据通常比字符串更紧凑，每个字节的数据以两个字符表示。例如，`0x48`表示十进制的`72`。
2. **更接近机器码**：十六进制格式与机器码更接近，便于处理底层数据和协议。

**优势**：
1. **节省带宽**：因为十六进制表示更加紧凑，传输相同的信息可以节省带宽，尤其在传输大量数据时更为明显。
2. **准确传输**：避免了字符串编码和解码过程中可能出现的错误，特别是在需要精确传输二进制数据时更为可靠。
3. **性能**：处理十六进制数据时，某些场景下性能可能更高，特别是在嵌入式系统和低带宽环境中。

**适用场景**
- **字符串格式适用场景**：文本消息、日志信息、用户输入数据等。
- **十六进制格式适用场景**：传输二进制数据、低带宽环境、嵌入式系统、需要高精度传输的应用等。

代码只需要简单改一下：
![[Pasted image 20240918192822.png]]
程序调试：
![[Pasted image 20240918192840.png]]