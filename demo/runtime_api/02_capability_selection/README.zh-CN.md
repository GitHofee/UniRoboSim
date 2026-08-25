# 02 - 按能力选择 Provider

[English](README.md) | [Runtime API 目录](../README.zh-CN.md)

## 面向对象

本例面向需要选择仿真后端、但不希望上层框架导入仿真器专用包的开发者。

## 本例说明什么

`ProviderRegistry` 在当前进程内保存 descriptor 和工厂。注册是延迟的：读取 descriptor 不会实例化
Provider。选择过程同时检查 `probe()` 可用性和带版本的能力要求；选择失败时会返回结构化尝试记录。

## 前置条件

- 完成案例 01
- UniRoboSim Core 0.10；不需要原生仿真器

## 运行

```bash
PYTHONPATH=src python demo/runtime_api/02_capability_selection/main.py
```

## 预期结果

```text
selected=reference.fake factory_calls=1
rejected=sensor.lidar@1 attempts=1
```

## 代码解释

1. `create_fake()` 增加计数器，用来证明 Provider 是延迟创建的。
2. `register()` 将 `FAKE_DESCRIPTOR` 与工厂绑定；读取 descriptor 不需要启动原生 SDK。
3. `CapabilityRequirement(CapabilityId("state.articulation@1"))` 表示带明确主版本的硬性要求。
4. `select()` 依次创建、探测、协商，并返回第一个满足要求的 Provider。
5. lidar 能力是故意设置的不支持项；`ProviderSelectionError.details["attempts"]` 记录每个候选失败原因。

## 常见错误

- `state.articulation` 不是合法能力 ID，必须写成 `state.articulation@1`。
- 注册时使用的 descriptor 必须与工厂返回 Provider 的 descriptor 完全一致。
- 重复 Provider ID 会被拒绝。
- `ProviderRegistry` 是显式、进程内 Registry；已安装 Adapter 的发现由 EasyAPI 通过
  `unirobosim.backends` entry point 完成。

## 验收级别

本例在 FakeProvider 上验证确定性的 Registry、可用性和能力协商语义，不代表原生 Adapter 已安装可用。

## 下一例

继续学习 [03 - 类型化铰接控制](../03_articulation_command/README.zh-CN.md)。
