# 一、前言
---
视频效果：
![[录屏-07-19-2024-06_01_10-PM.mp4]]
QT版本信息：
![[Pasted image 20240918193927.png]]根据官方文档，找到Button空间的信号。如下图所示，要进入父类AbstractButton找到信号。
![[Pasted image 20240918193952.png]]如下图所示，一共有7个信号，其中最常用的是clicked()与toggled()，本次实验使用信号clicked()。
![[Pasted image 20240918194016.png]]

# 二、代码
---
## 2.1、hell_world.h
```cpp
#ifndef HELL_WORLD_H
#define HELL_WORLD_H

#include <QObject>

class hell_world : public QObject
{
    Q_OBJECT
public:
    explicit hell_world(QObject *parent = nullptr);
    Q_INVOKABLE void say_Hello();

signals:
};

#endif // HELL_WORLD_H

```

重点是Q_INVOKABLE关键字，有了它，QML才能调用hell_world类的say_Hello()方法。
![[Pasted image 20240918194357.png]]

## 2.2、hell_world.cpp
```cpp
#include "hell_world.h"
#include <QDebug>
hell_world::hell_world(QObject *parent)
    : QObject{parent}
{}

//只在.h文件用关键字Q_INVOKABLE
void hell_world::say_Hello() {
    qDebug() << "I am class hell_world.";
}

```

## 2.3、main.cpp
```cpp
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext> // 上下文
#include "hell_world.h"

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);

    QQmlApplicationEngine engine;

    hell_world hw; // 创建类
    engine.rootContext()->setContextProperty("helloWorld", &hw); // 建立上下文，让QML认识类hw

    const QUrl url(QStringLiteral("qrc:/button_call_Cpp_Fun/Main.qml"));
    QObject::connect(
        &engine,
        &QQmlApplicationEngine::objectCreationFailed,
        &app,
        []() { QCoreApplication::exit(-1); },
        Qt::QueuedConnection);
    engine.load(url);

    return app.exec();
}

```

## 2.4、Main.qml
```cpp
import QtQuick
import QtQuick.Controls

Window {
    width: 640
    height: 480
    visible: true
    title: qsTr("Hello World")

    Button {
        text: "Say Hello"
        anchors.centerIn: parent
        // clicked信号
        onClicked: {
            helloWorld.say_Hello() // 调用hw类
        }
    }
}

```

# 三、细节补充
---
## 3.1、Q_INVOKABLE关键字
`Q_INVOKABLE`关键字是Qt框架中的一个特性，用于将C++类中的方法暴露给QML，以便从QML中调用这些方法。它的设计目的是提供一种简洁、直观的方式来在QML和C++之间进行交互，从Qt4版本开始被设计出来。

下面是`Q_INVOKABLE`关键字的用途和设计原因：
### 用途：
1. 暴露C++方法给QML：
    1. 在没有`Q_INVOKABLE`关键字的情况下，要在QML中调用C++方法，需要使用信号和槽机制，这会使代码变得复杂。
    2. 使用`Q_INVOKABLE`，可以直接在QML中调用被标记的方法，使得QML和C++的交互更加直接和高效。
2. 简化代码：
    1. 使用`Q_INVOKABLE`可以减少为了调用C++方法而编写的大量中间代码（如信号和槽的连接代码）。
    2. 提高代码的可读性和可维护性。

### 设计原因：
1. 增强QML与C++的集成：
    1. QML是Qt框架用于构建用户界面的声明式语言，而C++则用于实现应用逻辑。为了充分利用两者的优势，需要一种简单的方法将C++逻辑暴露给QML。
    2. `Q_INVOKABLE`关键字使得这一过程变得非常简单，使开发者可以更方便地在QML中调用C++方法。
2. 提供灵活性：
    1. 开发者可以选择将哪些方法暴露给QML，而无需改变方法的签名或增加额外的代码。
    2. 使得C++方法可以被视为QML对象的方法，这样在QML中使用这些方法时就像使用QML自身的方法一样简单。