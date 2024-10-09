# 导言
---
[[C++智能指针 - 共享指针std：：share_ptr]] C++也在不断进步，std::share_ptr<>()共享指针可以有效地管理内存泄漏问题。

**使用共享内存方式创建定时器：**
```cpp
#include <QCoreApplication>
#include <QTimer>
#include <QDebug>

// 槽函数，用于打印日志
void printLog() {
    qDebug() << "hello, world";
}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    // 创建一个 QTimer 对象
    //QTimer timer;
    std::shared_ptr<QTimer> pQTimer1 = std::make_shared<QTimer>();

    // 将 QTimer 的 timeout 信号连接到 printLog 槽函数
    QObject::connect(pQTimer1.get(), &QTimer::timeout, &printLog);

    // 启动定时器，设置时间间隔为1000毫秒（1秒）
    pQTimer1->start(1000);

    return a.exec();
}
```

pQTimer1对象，从使用的例子看来，使用.时，可以调用它本身的方法，比如.get()获取管理的对象指针的原始地址。
```cpp
pQTimer1.get();  //get()方法可以获取原始指针，就是QTimer对象的指针，在QObject::connect()里使用；
```

从这句代码看来，使用->时，可以调用管理的对象的方法。比如->start(1000)，它是QTimer对象的方法。
```cpp
pQTimer1->start(1000); // 启动定时器，设置时间间隔为1000毫秒（1秒）
```


**使用传统的方式创建定时器：**
```cpp
#include <QCoreApplication>
#include <QTimer>
#include <QDebug>

// 槽函数，用于打印日志
void printLog() {
    qDebug() << "hello, world";
}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    // 创建一个 QTimer 对象
    QTimer timer;

    // 将 QTimer 的 timeout 信号连接到 printLog 槽函数
    QObject::connect(&timer, &QTimer::timeout, &printLog);

    // 启动定时器，设置时间间隔为1000毫秒（1秒）
    timer.start(1000);

    return a.exec();
}
```

