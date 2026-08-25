# 08 — 调试与场景快照

[English](README.md) | [简体中文](README.zh-CN.md)

## 面向对象

适合开发检查工具、可视化调试或浏览器场景控制面的开发者。

## 前置条件

理解前面案例中的实体和批量坐标。

```bash
PYTHONPATH=src python demo/easyapi/08_debug_and_scene_snapshot/main.py
```

## 预期结果

一个调试点被接受，场景快照报告方块尺寸 `(0.2, 0.3, 0.4)`，最后清理一个图元。

## 代码讲解

`ArrayValue.from_nested([[[x,y,z]]])` 创建形状为 `[环境, 点, xyz]` 的几何数据。`DebugPrimitive` 提供稳定 ID、layer、类型、颜色、环境选择和手动生命周期。`sim.debug.publish()` 通过可移植调试端点发布它。`scene_snapshot()` 读取后端无关的实体和视觉信息，不暴露原生对象。`sim.debug.clear(layer="tutorial")` 使用稳定筛选条件删除标记。

## 常见错误

- 几何数据的 batch 大小必须与 `environment_indices` 完全一致。
- 四元数采用 XYZW 顺序，调试位置使用米。
- FakeProvider 只在内存中保存调试数据，不会显示真实渲染覆盖层。

## 验收级别

无第三方依赖的可移植调试与场景快照合同测试。

## 下一例

继续学习 [09 — 柔性体与流体](../09_deformable_and_fluid/README.zh-CN.md)。
