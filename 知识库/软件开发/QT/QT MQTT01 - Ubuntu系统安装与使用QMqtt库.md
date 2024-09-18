# 一、前言
---
网络上大部分都是Windows平台的教程，找一个linux的教程很困难。为此，记录Qt6.7版本安装QtMqtt库的过程。

# 二、安装
---
[https://github.com/qt/qtmqtt/tree/6.7.0](https://github.com/qt/qtmqtt/tree/6.7.0)
首先，进入Qt官方的mqtt库仓库，gitclone对应的版本。我使用Qt6.7版本。
下面是我安装Mqtt库的过程：
```bash
~/Public/Coding/Qt_Project/qtmqtt-6.7.0 » ls                    wallace@wallace
CMakeLists.txt  coin  dependencies.yaml  dist  examples  LICENSES  src  tests
--------------------------------------------------------------------------------
~/Public/Coding/Qt_Project/qtmqtt-6.7.0 » mkdir build           wallace@wallace
--------------------------------------------------------------------------------
~/Public/Coding/Qt_Project/qtmqtt-6.7.0 » cd build              wallace@wallace
--------------------------------------------------------------------------------
~/Public/Coding/Qt_Project/qtmqtt-6.7.0/build » cmake .. -G "Unix Makefiles" -DCMAKE_PREFIX_PATH=~/Qt/6.7.0/gcc_64
-- The CXX compiler identification is GNU 11.4.0
-- The C compiler identification is GNU 11.4.0
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Check for working CXX compiler: /usr/bin/c++ - skipped
-- Detecting CXX compile features
-- Detecting CXX compile features - done
-- Detecting C compiler ABI info
-- Detecting C compiler ABI info - done
-- Check for working C compiler: /usr/bin/cc - skipped
-- Detecting C compile features
-- Detecting C compile features - done
-- Looking for pthread.h
-- Looking for pthread.h - found
-- Performing Test CMAKE_HAVE_LIBC_PTHREAD
-- Performing Test CMAKE_HAVE_LIBC_PTHREAD - Success
-- Found Threads: TRUE  
-- Performing Test HAVE_STDATOMIC
-- Performing Test HAVE_STDATOMIC - Success
-- Found WrapAtomic: TRUE  
-- Found OpenGL: /usr/lib/x86_64-linux-gnu/libOpenGL.so   
-- Found WrapOpenGL: TRUE  
-- Found XKB: /usr/lib/x86_64-linux-gnu/libxkbcommon.so (found suitable version "1.4.0", minimum required is "0.5.0") 
-- Found WrapVulkanHeaders: /usr/include  
CMake Warning at /home/wallace/Qt/6.7.0/gcc_64/lib/cmake/Qt6/QtBuildHelpers.cmake:12 (message):
  The officially supported CMake generator for building Qt is Ninja / Ninja
  Multi-Config.  You are using: 'Unix Makefiles' instead.  Thus, you might
  encounter issues.  Use at your own risk.
Call Stack (most recent call first):
  /home/wallace/Qt/6.7.0/gcc_64/lib/cmake/Qt6/QtBuildHelpers.cmake:381 (qt_internal_validate_cmake_generator)
  /home/wallace/Qt/6.7.0/gcc_64/lib/cmake/Qt6/QtBuild.cmake:4 (qt_internal_setup_build_and_global_variables)
  /home/wallace/Qt/6.7.0/gcc_64/lib/cmake/Qt6/QtSetup.cmake:6 (include)
  /home/wallace/Qt/6.7.0/gcc_64/lib/cmake/Qt6/QtBuildRepoHelpers.cmake:21 (include)
  /home/wallace/Qt/6.7.0/gcc_64/lib/cmake/Qt6/QtBuildRepoHelpers.cmake:164 (qt_build_internals_set_up_private_api)
  /home/wallace/Qt/6.7.0/gcc_64/lib/cmake/Qt6/QtBuildRepoHelpers.cmake:375 (qt_build_repo_begin)
  CMakeLists.txt:19 (qt_build_repo)

-- Force setting build type to 'RelWithDebInfo'.

-- Configuring done
-- Generating done
-- Build files have been written to: /home/wallace/Public/Coding/Qt_Project/qtmqtt-6.7.0/build
--------------------------------------------------------------------------------
~/Public/Coding/Qt_Project/qtmqtt-6.7.0/build » make            wallace@wallace
[  2%] Generating ../../mkspecs/modules/qt_lib_mqtt_private.pri
[  2%] Built target Mqtt_lib_pri
[  5%] Running syncqt.cpp for module: QtMqtt
[  8%] Generating version linker script for target Mqtt
[  8%] Built target Mqtt_version_script
[ 11%] Built target Mqtt_sync_headers
[ 14%] Automatic MOC for target Mqtt
[ 14%] Built target Mqtt_autogen
[ 17%] Running AUTOMOC file extraction for target Mqtt
[ 17%] Built target Mqtt_automoc_json_extraction
[ 20%] Generating prl file for target Mqtt
[ 23%] Generating pc file for target Qt6::Mqtt
[ 26%] Running moc --collect-json for target Mqtt
[ 29%] Building CXX object src/mqtt/CMakeFiles/Mqtt.dir/Mqtt_autogen/mocs_compilation.cpp.o
[ 32%] Building CXX object src/mqtt/CMakeFiles/Mqtt.dir/qmqttauthenticationproperties.cpp.o
[ 35%] Building CXX object src/mqtt/CMakeFiles/Mqtt.dir/qmqttclient.cpp.o
[ 38%] Building CXX object src/mqtt/CMakeFiles/Mqtt.dir/qmqttconnection.cpp.o
[ 41%] Building CXX object src/mqtt/CMakeFiles/Mqtt.dir/qmqttconnectionproperties.cpp.o
[ 44%] Building CXX object src/mqtt/CMakeFiles/Mqtt.dir/qmqttcontrolpacket.cpp.o
[ 47%] Building CXX object src/mqtt/CMakeFiles/Mqtt.dir/qmqttmessage.cpp.o
[ 50%] Building CXX object src/mqtt/CMakeFiles/Mqtt.dir/qmqttpublishproperties.cpp.o
[ 52%] Building CXX object src/mqtt/CMakeFiles/Mqtt.dir/qmqttsubscription.cpp.o
[ 55%] Building CXX object src/mqtt/CMakeFiles/Mqtt.dir/qmqttsubscriptionproperties.cpp.o
[ 58%] Building CXX object src/mqtt/CMakeFiles/Mqtt.dir/qmqtttopicfilter.cpp.o
[ 61%] Building CXX object src/mqtt/CMakeFiles/Mqtt.dir/qmqtttopicname.cpp.o
[ 64%] Building CXX object src/mqtt/CMakeFiles/Mqtt.dir/qmqtttype.cpp.o
[ 67%] Linking CXX shared library ../../lib/libQt6Mqtt.so
[ 67%] Built target Mqtt
[ 70%] headersclean: Checking header src/mqtt/qmqtttype.h
[ 73%] headersclean: Checking header src/mqtt/qmqttauthenticationproperties.h
[ 76%] headersclean: Checking header src/mqtt/qmqttclient.h
[ 79%] headersclean: Checking header src/mqtt/qmqttconnectionproperties.h
[ 82%] headersclean: Checking header src/mqtt/qmqttmessage.h
[ 85%] headersclean: Checking header src/mqtt/qmqttpublishproperties.h
[ 88%] headersclean: Checking header src/mqtt/qmqttsubscription.h
[ 91%] headersclean: Checking header src/mqtt/qmqttsubscriptionproperties.h
[ 94%] headersclean: Checking header src/mqtt/qmqtttopicfilter.h
[ 97%] headersclean: Checking header src/mqtt/qmqtttopicname.h
[100%] headersclean: Checking headers in QtMqtt
[100%] Built target Mqtt_headersclean_check
[100%] Built target headersclean_check
-----------------------------------------------------------------------------------------------
~/Public/Coding/Qt_Project/qtmqtt-6.7.0/build » sudo make install              wallace@wallace
[sudo] wallace 的密码： 
[  2%] Built target Mqtt_lib_pri
[  8%] Built target Mqtt_version_script
[ 11%] Built target Mqtt_sync_headers
[ 14%] Automatic MOC for target Mqtt
[ 14%] Built target Mqtt_autogen
[ 17%] Running AUTOMOC file extraction for target Mqtt
[ 17%] Built target Mqtt_automoc_json_extraction
[ 20%] Running moc --collect-json for target Mqtt
Consolidate compiler generated dependencies of target Mqtt
[ 67%] Built target Mqtt
[ 70%] headersclean: Checking headers in QtMqtt
[100%] Built target Mqtt_headersclean_check
[100%] Built target headersclean_check
Install the project...
-- Install configuration: "RelWithDebInfo"
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/./metatypes/qt6mqtt_relwithdebinfo_metatypes.json
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/lib/cmake/Qt6Mqtt/Qt6MqttConfig.cmake
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/lib/cmake/Qt6Mqtt/Qt6MqttConfigVersion.cmake
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/lib/cmake/Qt6Mqtt/Qt6MqttConfigVersionImpl.cmake
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/lib/libQt6Mqtt.so.6.7.0
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/lib/libQt6Mqtt.so.6
-- Set runtime path of "/home/wallace/Qt/6.7.0/gcc_64/lib/libQt6Mqtt.so.6.7.0" to "$ORIGIN"
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/lib/libQt6Mqtt.so
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/lib/cmake/Qt6Mqtt/Qt6MqttTargets.cmake
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/lib/cmake/Qt6Mqtt/Qt6MqttTargets-relwithdebinfo.cmake
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/lib/cmake/Qt6Mqtt/Qt6MqttVersionlessTargets.cmake
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/lib/Qt6Mqtt.debug
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/./modules/Mqtt.json
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/lib/cmake/Qt6Mqtt/Qt6MqttAdditionalTargetInfo.cmake
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/include/QtMqtt
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/include/QtMqtt/QMqttPublishProperties
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/include/QtMqtt/QMqttTopicName
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/include/QtMqtt/QMqttUnsubscriptionProperties
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/include/QtMqtt/QtMqttVersion
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/include/QtMqtt/QMqttAuthenticationProperties
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/include/QtMqtt/QMqttTopicFilter
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/include/QtMqtt/QMqttUserProperties
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/include/QtMqtt/QMqttSubscription
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/include/QtMqtt/QMqttStringPair
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/include/QtMqtt/QMqttConnectionProperties
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/include/QtMqtt/QMqttSubscriptionProperties
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/include/QtMqtt/QtMqtt
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/include/QtMqtt/qtmqttversion.h
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/include/QtMqtt/QMqttServerConnectionProperties
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/include/QtMqtt/QMqttLastWillProperties
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/include/QtMqtt/QMqttClient
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/include/QtMqtt/QMqttMessage
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/include/QtMqtt/QMqttMessageStatusProperties
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/include/QtMqtt/qtmqttexports.h
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/include/QtMqtt/qmqttauthenticationproperties.h
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/include/QtMqtt/qmqttclient.h
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/include/QtMqtt/qmqttconnectionproperties.h
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/include/QtMqtt/qmqttglobal.h
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/include/QtMqtt/qmqttmessage.h
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/include/QtMqtt/qmqttpublishproperties.h
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/include/QtMqtt/qmqttsubscription.h
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/include/QtMqtt/qmqttsubscriptionproperties.h
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/include/QtMqtt/qmqtttopicfilter.h
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/include/QtMqtt/qmqtttopicname.h
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/include/QtMqtt/qmqtttype.h
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/include/QtMqtt/QtMqttDepends
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/include/QtMqtt/6.7.0/QtMqtt/private/qmqttclient_p.h
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/include/QtMqtt/6.7.0/QtMqtt/private/qmqttconnection_p.h
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/include/QtMqtt/6.7.0/QtMqtt/private/qmqttconnectionproperties_p.h
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/include/QtMqtt/6.7.0/QtMqtt/private/qmqttcontrolpacket_p.h
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/include/QtMqtt/6.7.0/QtMqtt/private/qmqttmessage_p.h
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/include/QtMqtt/6.7.0/QtMqtt/private/qmqttpublishproperties_p.h
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/include/QtMqtt/6.7.0/QtMqtt/private/qmqttsubscription_p.h
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/mkspecs/modules/qt_lib_mqtt.pri
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/mkspecs/modules/qt_lib_mqtt_private.pri
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/lib/pkgconfig/Qt6Mqtt.pc
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/lib/cmake/Qt6Mqtt/Qt6MqttDependencies.cmake
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/lib/cmake/Qt6BuildInternals/StandaloneTests/QtMqttTestsConfig.cmake
-- Up-to-date: /home/wallace/Qt/6.7.0/gcc_64/lib
-- Up-to-date: /home/wallace/Qt/6.7.0/gcc_64/lib/cmake
-- Up-to-date: /home/wallace/Qt/6.7.0/gcc_64/lib/cmake/Qt6BuildInternals
-- Up-to-date: /home/wallace/Qt/6.7.0/gcc_64/lib/cmake/Qt6BuildInternals/StandaloneTests
-- Up-to-date: /home/wallace/Qt/6.7.0/gcc_64/lib/cmake/Qt6Mqtt
-- Up-to-date: /home/wallace/Qt/6.7.0/gcc_64/lib/pkgconfig
-- Installing: /home/wallace/Qt/6.7.0/gcc_64/lib/libQt6Mqtt.prl
-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
~/Public/Coding/Qt_Project/qtmqtt-6.7.0/build »   
```

# 三、步骤
---
### 3.1、从Git仓库下载QMqtt模块
1. **安装必要的工具和依赖**
    首先，确保你已经安装了必要的工具和依赖：
    ```bash
    sudo apt-get update
    sudo apt-get install git build-essential qt6-base-dev
    ```
    
2. **克隆QMqtt模块的Git仓库**
    然后，从Qt官方Git仓库中克隆QMqtt模块：
    ```bash
    git clone <https://code.qt.io/qt/qtmqtt.git>
    cd qtmqtt
    ```
    
3. **创建构建目录**
    为了保持源代码目录的整洁，创建一个单独的构建目录：
    ```bash
    mkdir build
    cd build
    ```
    
4. **配置和编译QMqtt模块**
    使用CMake配置和编译QMqtt模块：
    ```bash
    cmake .. -G "Unix Makefiles" -DCMAKE_PREFIX_PATH=~/Qt/6.7.0/gcc_64
    make
    sudo make install
    ```
请将 `/path/to/qt6/installation` 替换为实际的Qt6安装路径。
我的电脑上Qt6的安装路径：
![[Pasted image 20240918191603.png]]

# 四、代码测试
---
## 4.1、创建一个简单的Console程序，验证Mqtt库是否正常
![[Pasted image 20240918191644.png]]
## 4.2、CMakeLists.txt添加Mqtt库
![[Pasted image 20240918191717.png]]
官方文档的说明：
![[Pasted image 20240918191740.png]]

## 4.3、main.cpp
```c
#include <QCoreApplication>
#include <QMqttClient>
#include <QDebug>

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    QMqttClient mqttClient; // 创建QMqttClient对象，用于与MQTT代理通信
    mqttClient.setHostname("127.0.0.1"); // MQTT Broker地址
    mqttClient.setPort(1883); // MQTT代理端口
    mqttClient.setClientId("QtClientTest"); // 设置客户端ID

    QObject::connect(&mqttClient, &QMqttClient::stateChanged, [](QMqttClient::ClientState state) {
        qDebug() << "Client state changed:" << state; // 当客户端状态改变时输出状态信息
    });

    QObject::connect(&mqttClient, &QMqttClient::messageReceived, [](const QByteArray &message, const QMqttTopicName &topic) {
        qDebug() << "Message received:" << message << "from topic:" << topic.name(); // 当收到消息时输出消息内容和主题
    });

    QObject::connect(&mqttClient, &QMqttClient::connected, &mqttClient, [&mqttClient](){
        qDebug() << "Connected to broker"; // 当成功连接到MQTT代理时输出信息
        QMqttSubscription *subscription = mqttClient.subscribe(QMqttTopicFilter("test/topic"), 0); // 替换为你要订阅的主题
        if (!subscription) {
            qDebug() << "Could not subscribe. Is there a valid connect"; // 如果订阅失败，输出错误信息
        } else {
            qDebug() << "Subscription successful"; // 如果订阅成功，输出成功信息
        }
        mqttClient.publish(QMqttTopicName("test/Hello"), "Hello MQTT"); // 发布消息到主题
    });

    mqttClient.connectToHost(); // 连接到MQTT代理

    return a.exec();
}

```
程序的效果如下：
![[录屏 07-02-2024 08_15_55 PM 1.webm]]