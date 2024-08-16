## 导言
---
要想成为嵌入式高手，必须学会且经常查看.map文件。
## 一、.map文件的结构
---
- **Section Cross References (各个文件的函数调用关系）**
- **Removing Unused input sections from the image（被删除的冗余部分）**
- **Image Symbol Table**
    - Local Symbols（局部标号）
        - 用Static声明的全局变量地址和大小
        - C文件中函数的地址和用static声明的函数代码大小
        - 汇编文件中的标号地址（作用域限本文件）
    - Global Symbols（全局标号）
        - 全局变量的地址和大小
        - C文件中函数的地址及其代码大小
        - 汇编文件中的标号地址（作用域全工程）
- **Memory Map of the image（描述ROM与RAM的细节）**
- **Image component sizes（组件大小）**