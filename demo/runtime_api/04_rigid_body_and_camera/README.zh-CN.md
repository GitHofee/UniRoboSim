# 04 - 刚体与相机观测

[English](README.md) | [Runtime API 目录](../README.zh-CN.md)

## 面向对象

本例面向开发可移植物体控制、观测传输或相机录制功能，同时不希望暴露后端 SDK 数组的框架开发者。

## 本例说明什么

World 在两个 environment 中包含一个程序化刚体盒子和一个同步 RGB/depth 相机。程序提交类型化
wrench，然后读取可移植刚体状态和紧凑存储的相机 channel。

## 前置条件

- 完成案例 03，或已经理解 Runtime 的 batch-first 数值
- UniRoboSim Core 0.10；不需要图像库和原生渲染器

## 运行

```bash
PYTHONPATH=src python demo/runtime_api/04_rigid_body_and_camera/main.py
```

## 预期结果

```text
box_x=0.010 rgb=(2, 6, 8, 3)/uint8 depth=(2, 6, 8)/float32
```

## 代码解释

1. `BoxGeometrySpec` 携带可移植的尺寸、质量、材质和颜色意图。
2. 本例把重力设为零，便于隔离提交的 wrench 所产生的效果。
3. `RigidBodyCommand` 为每个所选 environment 提供一行 force 和 torque。
4. `read_rigid_body()` 以不可变 `ArrayValue` 返回位置、XYZW 姿态和速度。
5. `read_sensor()` 在同一个仿真 tick 采集所有请求的 channel。
6. RGB 使用紧凑 uint8 存储；`to_bytes()` 避免把每个像素分量展开成 Python 整数。

## 常见错误

- wrench 数组 shape 是 `[environment, xyz]`，不是 `[xyz]`。
- RGB shape 为 `[environment, height, width, rgb]`，depth 没有最后一个 channel 维度。
- 使用 `CameraModality` 读取 channel，不要传任意字符串。
- FakeProvider 返回确定性测试图案，不能用来证明原生图像质量、相机标定或渲染性能。

## 验收级别

本例验证可移植 wrench/状态语义、相机 shape、dtype 和紧凑字节访问。真实物理和渲染图像仍属于
Adapter 原生验收范围。

## 下一例

继续学习 [05 - 场景控制](../05_scene_control/README.zh-CN.md)。
