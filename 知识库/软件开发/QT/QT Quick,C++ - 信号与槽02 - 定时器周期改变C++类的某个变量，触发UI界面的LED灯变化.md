# 一、前言
---
QT版本信息：
![[Pasted image 20240918194518.png]]
代码效果：
![[录屏 07-23-2024 11_37_25 AM.webm]]
# 二、代码
---
## 2.1、hell_world.cpp
```cpp
#include "hell_world.h"

hell_world::hell_world(QObject *parent)
    : QObject{parent}
{}

/**
 * @brief 启动周期1S的定时器
 */
void hell_world::startTimer() {
    /**
     *  发送信号的对象：  &timer
     *  发送的信号：     &QTimer::timeout
     *  接收信号的对象：  this
     *  接收信号的槽函数：&hell_world::timeOut
     *
     *  this指针的作用:
     *  this指针在C++中表示当前对象的指针。在这段代码中，this指针用于指定哪个对象应该接收信号并调用槽函数。
     *  具体来说，它告诉connect函数，当timer对象发出timeout信号时，应该调用当前对象（hell_world实例）的timeOut槽函数
     */
    connect(&timer, &QTimer::timeout, this, &hell_world::timeOut); // 连接信号与槽
    this->timer.start(1000); // 启动定时器
}

/**
 * @brief hell_world::timeOut 定时器超时回调函数
 */
void hell_world::timeOut() {
    this->value = !value;
    qDebug() << "Value is now:" << value;
    emit valueChanged(value); // 发出valueChanged信号，传递当前value值
}

```

## 2.2、hell_world.h
```cpp
#ifndef HELL_WORLD_H
#define HELL_WORLD_H

#include <QObject>
#include <QCoreApplication>
#include <QTimer>
#include <QDebug>

class hell_world : public QObject
{
    Q_OBJECT

public:
    explicit hell_world(QObject *parent = nullptr);
    void startTimer();

private:
    bool   value = false;
    QTimer timer;

public slots:
    void timeOut();

signals:
    void valueChanged(quint8 value);
};

#endif // HELL_WORLD_H

```

## 2.3、main.cpp
```cpp
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include "hell_world.h"
#include <QQmlContext>

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);

    hell_world hw;
    hw.startTimer();

    QQmlApplicationEngine engine;
    engine.rootContext()->setContextProperty("hellWorld",&hw); // 让QML知道实例化的hw对象

    const QUrl url(QStringLiteral("qrc:/Signal_SEND_UI_LED/Main.qml"));
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

Window {
    width: 640
    height: 480
    visible: true
    title: qsTr("Hello World")

    // 定义一个颜色属性，用于控制LED灯的颜色
    property color ledColor: "red"

    Rectangle {
        id: led
        width: 100
        height: 100
        color: ledColor
        radius: 50
        anchors.centerIn: parent

        // 当hellWorld的valueChanged信号发出时，切换LED灯的颜色
        Connections {
            target: hellWorld
            function onValueChanged(value) {
                ledColor = (value === 1) ? "green" : "red";
            }
        }
    }
}
```

























