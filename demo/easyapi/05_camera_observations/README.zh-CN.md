# 05 — 相机观测

[English](README.md) | [简体中文](README.zh-CN.md)

## 面向对象

适合消费相机观测或录制图像流的开发者。

## 前置条件

UniRoboSim Core 0.10；不需要安装图像库。

```bash
PYTHONPATH=src python demo/easyapi/05_camera_observations/main.py
```

## 预期结果

程序输出 RGB 形状 `(2, 6, 8, 3)`、深度形状 `(2, 6, 8)`、法线形状 `(2, 6, 8, 3)`，以及 288 个紧凑存储的 RGB 字节。

## 代码讲解

`add_camera()` 用 `(8,6)` 声明宽度优先的分辨率和所需模态。`camera.sample()` 返回同一个 tick 的通道。`sample.channel(CameraModality.RGB)` 使用类型化枚举，`camera.read("normals")` 展示字符串简写。RGB 是紧凑存储的 `uint8` 值，使用 `to_bytes()` 可以避免把每个像素展开成 Python 整数。输出维度顺序是环境、高度、宽度、通道。

## 常见错误

- 分辨率写作 `(宽, 高)`，输出形状则是 `[环境, 高, 宽, 通道]`。
- `to_bytes()` 只能用于 `uint8` 数据。
- FakeProvider 图像是确定性测试图样，不是真实场景渲染画面。

## 验收级别

无第三方依赖的相机模态、形状、类型和紧凑存储合同测试。

## 下一例

继续学习 [06 — 能力与 Provider](../06_capabilities_and_provider/README.zh-CN.md)。
