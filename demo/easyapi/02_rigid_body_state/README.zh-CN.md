# 02 — 刚体状态

[English](README.md) | [简体中文](README.zh-CN.md)

## 面向对象

适合需要发送物理命令并读取可移植状态的初学者。

## 前置条件

先完成案例 01；不需要真实仿真器 SDK。

```bash
PYTHONPATH=src python demo/easyapi/02_rigid_body_state/main.py
```

## 预期结果

执行一个 0.1 秒的测试步后，程序输出 `position_x_m=0.010000` 和 `velocity_x_m_s=0.100000`。

## 代码讲解

`Sim` 把物理步长设为 0.1 秒并关闭重力，让结果容易核对。`add_box(..., mass_kg=2.0)` 把质量意图写入可移植世界。`apply_wrench(force_n=(2,0,0))` 发送力命令。`step()` 推进一步并返回类型化 `Tick`。`box.state` 包含按环境批处理的位置、XYZW 姿态、线速度和角速度。仿真值是浮点数，因此断言使用 `math.isclose()`。

## 常见错误

- `apply_wrench()` 使用 SI 单位：牛顿和牛顿米。
- 状态数组以环境维为第一维，`rows()[0]` 读取第 0 个环境。
- FakeProvider 的点质量更新只是确定性测试行为，不是精度基准。

## 验收级别

无第三方依赖的命令与状态合同测试。

## 下一例

继续学习 [03 — 铰接物体控制](../03_articulation_control/README.zh-CN.md)。
