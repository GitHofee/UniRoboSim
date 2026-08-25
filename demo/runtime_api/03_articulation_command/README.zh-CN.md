# 03 - 类型化铰接控制

[English](README.md) | [Runtime API 目录](../README.zh-CN.md)

## 面向对象

本例适合开发 Controller 桥接、模型服务循环、rule-based executor 或录制器，并需要交换批量关节
控制与状态的开发者。

## 本例说明什么

示例在两个 environment 中创建一个非机器人铰接物体：它有一个旋转铰链和一个直线滑轨。程序解析
可移植 handle，只控制一个 environment 中的一个关节，执行一步后读取类型化状态。

## 前置条件

- 理解案例 01 的 Provider、Session 和 World 所有权
- UniRoboSim Core 0.10；不需要原生仿真器

## 运行

```bash
PYTHONPATH=src python demo/runtime_api/03_articulation_command/main.py
```

## 预期结果

```text
tick=1 joint_positions=((0.1, -0.2), (0.75, -0.2))
```

## 代码解释

1. `EnvironmentSpec(count=2)` 确定数组最前面的 batch 维度。
2. 物理 v0alpha5 World schema 支持明确的旋转轴和直线轴单位；
   `joint_position_units=("rad", "m")` 补全这些语义。
3. `world.resolve(EntityPath("/cabinet"))` 返回绑定当前 generation 的 handle。
4. 只选择一个 environment 和一个自由度，因此 target shape 是 `(1, 1)`。
5. `target_units=("rad",)` 补全所选轴的单位合同。
6. 控制在 `step()` 前提交；step 后读取的状态保持 environment-first 的 batch 顺序。

## 常见错误

- Runtime API 不会隐式广播控制数组，shape 必须与所选 environment 和关节完全匹配。
- target 的行顺序对应 `environment_indices`，列顺序对应 `degree_of_freedom_indices`。
- 旋转位置使用 `rad`，直线位置使用 `m`。
- handle 只属于创建它的 Session、World ID 和 generation。

## 验收级别

FakeProvider 执行确定性的参考积分。本例验证控制 shape、选择、单位、handle 和状态布局，不验证原生
执行器动力学。

## 下一例

继续学习 [04 - 刚体和相机](../04_rigid_body_and_camera/README.zh-CN.md)。
