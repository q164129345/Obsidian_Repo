# 导言
---
![[录屏2024-09-19 07.33.21.mov]]
# 一、main.cpp
---
```c
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QTimer>
#include <QQmlContext>

class HelloValue : public QObject
{
    Q_OBJECT
    Q_PROPERTY(int value READ value WRITE setValue NOTIFY valueChanged)

public:
    HelloValue() : m_value(0) {}

    int value() const { return m_value; }
    void setValue(int newValue) {
        if (m_value != newValue) {
            m_value = newValue;
            emit valueChanged();
        }
    }

signals:
    void valueChanged();

private:
    int m_value;
};

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);

    HelloValue helloValue; // 实例化对象

    QQmlApplicationEngine engine;
    engine.rootContext()->setContextProperty("helloValue", &helloValue); // 让qml认识helloValue对象

    QObject::connect(
        &engine,
        &QQmlApplicationEngine::objectCreationFailed,
        &app,
        []() { QCoreApplication::exit(-1); },
        Qt::QueuedConnection);
    engine.loadFromModule("hello_value", "Main");

	// 创建一个定时器
    QTimer timer;
    QObject::connect(&timer, &QTimer::timeout, [&helloValue]() {
        helloValue.setValue(helloValue.value() + 1);
    });
    timer.start(500); // 回调周期500ms

    return app.exec();
}

#include "main.moc"

```

# 二、main.qml
---
```java
import QtQuick
import QtQuick.Controls

Window {
    width: 640
    height: 480
    visible: true
    title: qsTr("Hello Value")

    Text {
        anchors.centerIn: parent
        font.pixelSize: 24
        text: qsTr("当前值: ") + helloValue.value
    }
}

```

# 三、细节补充
---
## 3.1、为什么main.cpp的最底部增加#include “main.moc”
在main.cpp文件末尾使用 #include "main.moc" 是因为这个文件中包含了一个使用Q_OBJECT宏的类（HelloValue类）。这是Qt框架中的一个特殊处理，主要原因如下：
- Q_OBJECT宏：当一个类使用Q_OBJECT宏时，它需要一些额外的元数据和代码来支持Qt的信号槽机制、属性系统等特性。
- moc（Meta-Object Compiler）：Qt使用moc工具来处理包含Q_OBJECT宏的头文件，生成额外的C++代码。这些生成的代码通常保存在一个名为moc_.cpp的文件中。
- 内联类定义：在这个例子中，HelloValue类直接定义在main.cpp文件中，而不是在单独的头文件中。这种情况下，moc无法正常生成moc_.cpp文件。
- main.moc：为了解决这个问题，Qt提供了一种特殊的处理方式。通过在文件末尾包含 #include "main.moc"，编译器会在适当的位置插入由moc生成的代码。
- 编译过程：当编译器处理到这一行时，它会寻找由moc工具生成的main.moc文件，并将其内容包含到这里。这样就确保了所有必要的元数据和支持代码都被正确地包含在编译过程中。

总之，#include "main.moc" 是一种特殊的Qt技巧，用于确保在直接在.cpp文件中定义使用Q_OBJECT宏的类时，能够正确地包含所有必要的元对象代码。这对于Qt的信号槽机制和属性系统的正常工作是必需的。

