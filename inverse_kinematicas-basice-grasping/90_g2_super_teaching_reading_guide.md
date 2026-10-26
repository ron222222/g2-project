# 90版 G2 超级教学注释版阅读指南

## 文件定位

教学版程序包含原程序全部可执行代码，并在原有逐行注释基础上增加：

- 每个核心函数的功能、输入、输出、调用者、结果流向
- 对应的G2真实动作
- 调试时应重点观察的日志与字段
- Python初学者需要理解的语法说明
- 关键参数的单位、作用和修改风险
- 导入模块在本程序中的具体用途

## 推荐阅读顺序

1. 搜索 `def main()`，先通读总流程。
2. 阅读三个视觉函数：`detect_product_batch()`、`detect_product()`、`show_frame()`。
3. 阅读三个运动函数：`move_arm_once()`、`move_tcp_closed_loop()`、`align_orientation_target()`。
4. 阅读放置流程：`turn_release_and_return_waist()`。
5. 最后阅读数学函数：`q_to_r()`、`rotation_to_q()`、`slerp()`。

## 初学者必须掌握的10个概念

1. 变量与赋值
2. 列表、字典和索引
3. `if/elif/else`
4. `for/while`
5. 函数定义与函数调用
6. `try/except/finally`
7. `None/True/False`
8. position与orientation
9. TCP、End、TF和Base坐标系
10. 米、毫米、弧度和角度之间的单位区别

## 校验说明

教学版与附件原文件执行语法树完全一致，并通过Python编译检查。本次没有连接机器人，也没有发送任何GDK命令。
