# 导言
---
在C++中，共享指针（`std::shared_ptr`）是一种智能指针，用于管理动态分配对象的生命周期。它通过引用计数的方式，允许多个指针共享同一个对象，当最后一个引用被销毁时，对象才会被自动释放。

**为什么需要共享指针？**
在传统的C++编程中，动态分配的对象需要手动删除（使用`delete`），否则会造成内存泄漏。然而，在复杂的程序中，跟踪何时应该释放对象可能会很困难，尤其是当多个部分需要访问同一个对象时。

**`std::shared_ptr`的工作原理**
• **引用计数**：std::shared_ptr内部维护一个引用计数器，每当有一个新的`std::shared_ptr`指向同一个对象时，计数器加一；当一个`std::shared_ptr`被销毁、重置或重新分配时，计数器减一。
• **自动内存管理**：当引用计数变为零时，std::shared_ptr会自动删除所管理的对象，释放内存。

**如何使用`std::shared_ptr`**
```c++
#include <iostream>
#include <memory>

int main() {
    std::shared_ptr<int> ptr1 = std::make_shared<int>(10); // 在堆里创建变量10，并用共享指针ptr1管理
    {
        std::shared_ptr<int> ptr2 = ptr1; // 引用计数加一
        std::cout << "引用计数：" << ptr1.use_count() << std::endl; // 输出2
    } // ptr2超出作用域，引用计数减一
    std::cout << "引用计数：" << ptr1.use_count() << std::endl; // 输出1
    return 0;
} // ptr1超出作用域，引用计数变为零，自动删除对象
```

**注意事项**
• **循环引用问题**：如果两个`std::shared_ptr`对象相互引用，会导致引用计数永远不为零，造成内存泄漏。解决方法是使用`std::weak_ptr`打破循环引用。
• **性能开销**：由于引用计数的维护，`std::shared_ptr`比普通指针有一定的性能开销。在性能敏感的场合，需要权衡使用。

**适用场景**
• **资源共享**：当多个对象需要共享同一资源时，`std::shared_ptr`可以方便地管理资源的生命周期。
• **动态内存管理**：简化内存管理，减少内存泄漏的风险。

**总结**
`std::shared_ptr`是C++标准库提供的智能指针类型，通过引用计数实现共享所有权，自动管理动态分配对象的生命周期，是现代C++编程中常用的内存管理工具。

# 一、细节补充
---
## 1.1、`std::shared_ptr`避免用`new`创建，应该用`std::make_shared`创建
`std::make_shared` 是用于创建 `shared_ptr` 的工厂函数，它会同时分配对象和引用计数器的内存，效率更高，也更安全。
**优点：**
- 使用 `std::make_shared` 能减少两次内存分配的开销（一次为对象，一次为引用计数），而直接使用 `new` 则可能需要两次分配。
- 它还避免了创建智能指针时抛出异常的潜在风险。
<br>
## 1.2、特别注意std::shared_ptr循环引用问题
```Cpp
#include <iostream>
#include <memory>

struct B;

struct A {
    std::shared_ptr<B> ptrB;
    ~A() { std::cout << "A destroyed" << std::endl; }
};

struct B {
    std::shared_ptr<A> ptrA;
    ~B() { std::cout << "B destroyed" << std::endl; }
};

int main() {
    auto a = std::make_shared<A>();
    auto b = std::make_shared<B>();
    a->ptrB = b;
    b->ptrA = a;

    // 此时 a 和 b 都不会被销毁，因为它们相互引用，引用计数不会为0
    return 0;
}

```
使用 `std::weak_ptr` 解决循环引用：
```Cpp
#include <iostream>
#include <memory>

struct B;

struct A {
    std::shared_ptr<B> ptrB;
    ~A() { std::cout << "A destroyed" << std::endl; }
};

struct B {
    std::weak_ptr<A> ptrA;  // 使用 weak_ptr 打破循环引用
    ~B() { std::cout << "B destroyed" << std::endl; }
};

int main() {
    auto a = std::make_shared<A>();
    auto b = std::make_shared<B>();
    a->ptrB = b;
    b->ptrA = a;  // B 持有 A 的弱引用

    // 现在，a 和 b 都可以正确销毁
    return 0;
}

```
<br>
## 1.3、使`std::shared_ptr`管理的对象或资源线程安全
如果多个线程同时拷贝同一个 shared_ptr 对象，不会有问题，因为 shared_ptr 的引用计数是线程安全的。但是如果多个线程同时修改同一个 shared_ptr 对象，不是线程安全的。因此，如果多个线程同时访问同一个 shared_ptr 对象，并且有写操作，需要使用互斥量来保护。
<br>
## 1.4、如何访问 `std::weak_ptr` 引用的对象？
```cpp
#include <QCoreApplication>

struct B;

struct A {
    std::shared_ptr<B> ptrB;
    ~A(){ qDebug() << "A Destroyed";}
    void sayHello() {qDebug() << " I am A";}
};

struct B {
    std::weak_ptr<A> ptrA;
    ~B() { qDebug() << "B Destroyed";}
    void sayHello() {qDebug() << " I am B";}
};


int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    auto p1 = std::make_shared<int> (100); // 在堆里创建int变量，并赋值100
    {
        std::shared_ptr<int> p2 = p1; // 引用指针
        qDebug() << "p1.use_count:" << p1.use_count() << "p1: " << *p1 << " p2: " << *p2;
    }
    qDebug() << "p1.use_count:" << p1.use_count();

    // 因为share_ptr的循环引用，导致
    {
        auto a1 = std::make_shared<A>();
        auto b1 = std::make_shared<B>();
        a1->ptrB = b1;
        b1->ptrA = a1;

		// 访问std::weak_ptr变量
        if (auto shareA = b1->ptrA.lock()) {
            qDebug() << "Accessing A through weak_ptr";
            shareA->sayHello();
        } else {
            qDebug() << "A has been destroyed";
        }

    }

    return a.exec();
}
```
运行的结果如下：
![[Pasted image 20241010120142.png]]
在上述代码中，`weakA.lock()` 返回一个 `std::shared_ptr`，如果对象 `A` 仍然存在，则可以通过该指针安全地访问对象的方法 `show()`。如果对象已被销毁，`lock()` 将返回空指针，从而避免对无效对象的访问。 要访问 `std::weak_ptr` 引用的对象，需要使用 `lock()` 函数将其转换为 `std::shared_ptr`。`lock()` 函数会返回一个 `std::shared_ptr`，如果对象已经销毁，则返回一个空的 `std::shared_ptr`。在使用 `lock()` 后，应始终检查返回的 `std::shared_ptr` 是否为空，以防止访问无效对象。
1. **访问对象的有效性**：`std::weak_ptr` 不能直接访问对象，而是需要通过调用 `lock()` 来获取一个 `std::shared_ptr`。因此，在使用 `lock()` 后需要检查返回的 `std::shared_ptr` 是否为空，以防止访问已经销毁的对象。
2. **对象的生命周期**：由于 `std::weak_ptr` 不增加引用计数，可能在获取对象时发现对象已经销毁，因此需要确保对象在需要时仍然有效。
3. **性能开销**：`lock()` 的调用有一定的性能开销，因为它需要检查引用对象的状态。在性能敏感的场合，需要注意这点。
<br>
## 1.5、为什么 `std::weak_ptr` 不能直接访问对象？
`std::weak_ptr` 不能直接访问对象的原因在于，它是一种非拥有性引用，不增加引用计数，也不对对象的生命周期进行管理。因此，`std::weak_ptr` 本身无法保证被引用对象在访问时仍然存在。为了安全地访问对象，必须将 `std::weak_ptr` 转换为 `std::shared_ptr`，这样才能确保访问对象时引用计数增加，避免对象在访问过程中被销毁。


