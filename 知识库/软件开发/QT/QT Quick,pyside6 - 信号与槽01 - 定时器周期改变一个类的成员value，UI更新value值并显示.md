# 导言
---
备忘录是C++的实现：[[QT Quick,C++ - 信号与槽03 - 定时器周期改变一个类的成员value，UI更新value值并显示]]
接着，用pyside6版本实现一遍看看，看看python的实现会不会简单一些。
![[录屏2024-09-19 07.51.21.mov]]

# 一、main.py
---
```python
# This Python file uses the following encoding: utf-8
import sys
from pathlib import Path
from PySide6.QtCore import QTimer, QObject, Signal, Slot, Property

from PySide6.QtGui import QGuiApplication
from PySide6.QtQml import QQmlApplicationEngine


class HelloValue(QObject):
    valueChanged = Signal(int)

    def __init__(self):
        super().__init__()
        self._value = 0

    def getValue(self):
        return self._value

    def setValue(self, value):
        if self._value != value:
            self._value = value
            self.valueChanged.emit(value)

    value = Property(int, getValue, setValue, notify=valueChanged)

    @Slot()
    def increment(self):
        self.setValue(self._value + 1)

if __name__ == "__main__":
    app = QGuiApplication(sys.argv)
    engine = QQmlApplicationEngine()

    hello_value = HelloValue()
    engine.rootContext().setContextProperty("helloValue", hello_value)

    timer = QTimer()
    timer.timeout.connect(hello_value.increment)
    timer.start(500)

    qml_file = Path(__file__).resolve().parent / "main.qml"
    engine.load(qml_file)
    if not engine.rootObjects():
        sys.exit(-1)
    sys.exit(app.exec())

```

# 二、main.qml
---
```java
import QtQuick
import QtQuick.Window

Window {
    width: 640
    height: 480
    visible: true
    title: qsTr("Hello World")

    Text {
        anchors.centerIn: parent
        font.pixelSize: 24
        text: "当前值: " + helloValue.value
    }

    Connections {
        target: helloValue
        function onValueChanged(value) {
            console.log("值已更新:", value)
        }
    }
}

```

# 三、py_helloValue.pyproject
---
```python
{
    "files": [
        "main.py",
        "main.qml"
    ]
}

```


