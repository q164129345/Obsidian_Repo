# 一、基本语法

| 示例                                                                                       | 说明                                        |
| ---------------------------------------------------------------------------------------- | ----------------------------------------- |
| [x = pi](https://ww2.mathworks.cn/help/matlab/matlab_env/create-and-edit-variables.html) | 使用等号（=）创建变量。 左侧（`x`）是变量的名称，其值为右侧（`pi`）的值。 |
| [y = sin(-5)](https://ww2.mathworks.cn/help/matlab/learn_matlab/calling-functions.html)  | 您可以使用括号提供函数的输入。                           |
# 二、桌面管理

| 函数                                                              | 示例              | 说明                  |
| --------------------------------------------------------------- | --------------- | ------------------- |
| [save](https://ww2.mathworks.cn/help/matlab/ref/save.html)      | `save data.mat` | 将当前工作区保存到 MAT 文件中。  |
| [load](https://ww2.mathworks.cn/help/matlab/ref/load.html)      | `load data.mat` | 将 MAT 文件中的变量加载到工作区。 |
| [clear](https://ww2.mathworks.cn/help/matlab/ref/clear.html)    | `clear`         | 清除工作区中的所有变量。        |
| [clc](https://www.mathworks.com/help/matlab/ref/clc.html)       | `clc`           | 清除命令行窗口中的所有文本。      |
| [format](https://www.mathworks.com/help/matlab/ref/format.html) | `format long`   | 更改数值输出的显示方式。        |

# 三、数组类型
| 示例            | 说明  |
| ------------- | --- |
| 4             | 标量  |
| [3 5]         | 行向量 |
| [1;3]         | 列向量 |
| [3 4 5;6 7 8] | 矩阵  |

# 四、等间距向量
| 示例                                                                                | 说明                                                                                                |
| --------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------- |
| 1:4                                                                               | 使用[冒号](https://ww2.mathworks.cn/help/matlab/ref/double.colon.html)（`:`）运算符，创建一个从 1 到 4，间距为 1 的向量。 |
| 1:0.5:4                                                                           | 创建一个从 1 到 4，间距为 0.5 的向量。                                                                          |
| [linspace](https://ww2.mathworks.cn/help/matlab/ref/double.linspace.html)(1,10,5) | 创建一个包含 5 个元素的向量。这些值从 1 到 10 均匀间隔。                                                                 |

# 五、创建矩阵
| 示例                                                                | 说明                 |
| ----------------------------------------------------------------- | ------------------ |
| [rand](https://ww2.mathworks.cn/help/matlab/ref/rand.html)(2)     | 创建一个 2 行 2 列的方阵。   |
| [zeros](https://ww2.mathworks.cn/help/matlab/ref/zeros.html)(2,3) | 创建一个 2 行 3 列的矩形矩阵。 |

# 六、索引
| 示例                                                            | 说明                   |
| ------------------------------------------------------------- | -------------------- |
| A([end](https://ww2.mathworks.cn/help/matlab/ref/end.html),2) | 访问最后一行的第二列中的元素。      |
| A(2,:)                                                        | 访问第二行的所有元素。          |
| A(1:3,:)                                                      | 访问前三行的所有列。           |
| A(2) = 11                                                     | 将数组中第二个元素的值更改为 `11`。 |
# 七、数组运算
| 示例                                        | 说明                                                                                                     |
| ----------------------------------------- | ------------------------------------------------------------------------------------------------------ |
| [1 1; 1 1] * [2 2; 2 2]                   | 执行[矩阵乘法](https://ww2.mathworks.cn/help/matlab/matlab_prog/array-vs-matrix-operations.html#btyv9yp-4)。  |
| **ans** =                                 |                                                                                                        |
| &nbsp;&nbsp;&nbsp; 4 &nbsp;&nbsp;&nbsp; 4 |                                                                                                        |
| &nbsp;&nbsp;&nbsp; 4 &nbsp;&nbsp;&nbsp; 4 |                                                                                                        |
| [1 1; 1 1] .* [2 2; 2 2]                  | 执行[按元素乘法](https://ww2.mathworks.cn/help/matlab/matlab_prog/array-vs-matrix-operations.html#bu90xxy-1)。 |
| **ans** =                                 |                                                                                                        |
| &nbsp;&nbsp;&nbsp; 2 &nbsp;&nbsp;&nbsp; 2 |                                                                                                        |
| &nbsp;&nbsp;&nbsp; 2 &nbsp;&nbsp;&nbsp; 2 |                                                                                                        |
# 八、多个输出
| 示例                                                                                            | 说明                       |
| --------------------------------------------------------------------------------------------- | ------------------------ |
| [xrow, xcol] = [size](https://ww2.mathworks.cn/help/matlab/ref/double.size.html#bvfgzsm-6)(x) | 将 `x` 中的行数和列数分别存为两个不同变量。 |
| [xMax, idx] = [max](https://ww2.mathworks.cn/help/matlab/ref/double.max.html)(x)              | 计算 `x` 的最大值及其对应的索引值。     |

# 九、文档
| 示例                                                             | 说明                 |
| -------------------------------------------------------------- | ------------------ |
| [doc](https://ww2.mathworks.cn/help/matlab/ref/doc.html) randi | 打开 `randi` 函数的文档页。 |
# 十、绘图
| 示例                                                                                                                      | 说明                                          |
| ----------------------------------------------------------------------------------------------------------------------- | ------------------------------------------- |
| [plot](https://ww2.mathworks.cn/help/matlab/ref/plot.html)(x,y,"ro--","LineWidth",5)                                    | 绘制一条红色 (`r`) 虚线 (`--`)，并使用圆圈 (`o`) 标记，线宽很大。 |
| [hold ](https://ww2.mathworks.cn/help/matlab/ref/hold.html)on                                                           | 在现有绘图中新增一行。                                 |
| hold off                                                                                                                | 为下一个绘图线条创建一个新坐标区。                           |
| [title](https://ww2.mathworks.cn/help/matlab/creating_plots/add-title-axis-labels-and-legend-to-graph.html)("My Title") | 为绘图添加标题。                                    |
# 十一、使用表
| 示例                                                                                               | 说明                             |
| ------------------------------------------------------------------------------------------------ | ------------------------------ |
| [data.HeightYards](https://ww2.mathworks.cn/help/matlab/matlab_prog/access-data-in-a-table.html) | 从表 `data` 中提取变量 `HeightYards`。 |
| data.HeightMeters = data.HeightYards * 0.9144                                                    | 从现有数据中派生一个表变量。                 |
# 十二、逻辑运算
| 示例                                                                                                                 | 说明                            |
| ------------------------------------------------------------------------------------------------------------------ | ----------------------------- |
| [[5 10 15] > 12](https://ww2.mathworks.cn/help/matlab/matlab_prog/array-comparison-with-relational-operators.html) | 将向量与值 `12` 进行比较。              |
| [v1(v1 > 6)](https://ww2.mathworks.cn/help/matlab/matlab_prog/find-array-elements-that-meet-a-condition.html)      | 提取 `v1` 中大于 `6` 的所有元素。        |
| x(x == 999) = 1                                                                                                    | 用值 `1` 替换 `x` 中等于 `999` 的所有值。 |
# 十三、编程
| 示例                                                               | 说明                                    |
| ---------------------------------------------------------------- | ------------------------------------- |
|                                                                  | 如果 `x` 大于 `0.5`，则将 `y` 的值设置为 `3`。     |
| [if ](https://ww2.mathworks.cn/help/matlab/ref/if.html)x > 0.5   |                                       |
| &nbsp;&nbsp;&nbsp;y = 3                                          |                                       |
| else                                                             | 否则，将 `y` 的值设置为 `4`。                   |
| &nbsp;&nbsp;&nbsp;y = 4                                          |                                       |
| end                                                              |                                       |
|                                                                  |                                       |
|                                                                  | 循环计数器 `c` 遍历值 `1:3`（即 `1`、`2` 和 `3`）。 |
| [for](https://ww2.mathworks.cn/help/matlab/ref/for.html) c = 1:3 |                                       |
| &nbsp;&nbsp;&nbsp;disp(c)                                        | 循环体显示 `c` 的每个值。                       |
| end                                                              |                                       |
| ```                                                              |                                       |
