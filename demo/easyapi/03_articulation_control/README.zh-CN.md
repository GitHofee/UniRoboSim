# 03 — 铰接物体控制

[English](README.md) | [简体中文](README.zh-CN.md)

## 面向对象

适合控制机器人或橱柜、笔记本电脑、家电等普通铰接物体的应用开发者。

## 前置条件

UniRoboSim Core 0.10；不需要真实仿真器 SDK。

```bash
PYTHONPATH=src python demo/easyapi/03_articulation_control/main.py
```

## 预期结果

门铰链到达 `0.6`，没有收到命令的抽屉保持在 `-0.2`。

## 代码讲解

`add_articulation()` 声明两个有名称的自由度及其初始位置。这里的实体是柜子，说明铰接物体不一定是机器人。`cabinet.command((0.6,), joints=("door_hinge",), mode="position")` 通过可移植关节名只控制门铰链。`step()` 应用已经排队的目标。`cabinet.state.joint_positions` 按声明的关节顺序返回以环境为第一维的类型化数组。

## 常见错误

- target 数量必须与所选关节数量一致。
- 关节名必须与 `joint_names` 完全一致，错误名称会在下发到底层前失败。
- 位置、速度和力矩模式的单位及语义不同。

## 验收级别

无第三方依赖的命名关节控制与状态合同测试。

## 下一例

继续学习 [04 — 多环境](../04_multiple_environments/README.zh-CN.md)。
