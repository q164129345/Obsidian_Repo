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

## 1.3、使`std::shared_ptr`管理的对象或资源线程安全
如果多个线程同时拷贝同一个 shared_ptr 对象，不会有问题，因为 shared_ptr 的引用计数是线程安全的。但是如果多个线程同时修改同一个 shared_ptr 对象，不是线程安全的。因此，如果多个线程同时访问同一个 shared_ptr 对象，并且有写操作，需要使用互斥量来保护。