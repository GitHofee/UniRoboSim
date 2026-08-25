# 09 — 柔性体与流体

[English](README.md) | [简体中文](README.zh-CN.md)

## 面向对象

适合需要对衣物、软体或粒子流体进行可移植声明和点级控制的开发者。

## 前置条件

理解案例 04 的批量命令。本合同示例不需要真实仿真器 SDK。

```bash
PYTHONPATH=src python demo/easyapi/09_deformable_and_fluid/main.py
```

## 预期结果

衣物节点 0 移动到 `(0.1, 0.2, 1.2)`，流体粒子 0 在确定性测试模型中移动到 x `0.1`。

## 代码讲解

`add_deformable()` 声明静止位置、三角面拓扑和一个运动学节点。`add_particle_fluid()` 声明固定数量的粒子位置与半径。这两个方法会自动加入各自所需的点控制能力。`cloth.command(..., nodes=(0,), mode="position")` 只控制一个节点。`water.command(..., mode="velocity")` 把一个 xyz 速度广播给所有粒子。类型化状态始终使用 `[环境, 点, xyz]` 布局。

## 常见错误

- 表面拓扑需要合法的三角形索引；体积拓扑使用四面体。
- 点 target 必须是 xyz、点乘 xyz、或环境乘点乘 xyz 三种形状之一。
- FakeProvider 没有弹性、不可压缩性、碰撞、黏度或表面张力求解器，本案例只验证 API。

## 验收级别

无第三方依赖的软物质声明、命令、状态和能力合同测试；不属于物理精度验收。

## 下一例

继续学习 [10 — 切换真实后端](../10_switch_native_backend/README.zh-CN.md)。
