# 01 — Descriptor 与 Probe

[English](README.md) | 简体中文

**面向对象：** 正在定义 Adapter 第一层公共接口的后端开发者。
**验证级别：** 可执行公共合同检查；不会启动仿真器。

## 运行

先完成[一次性准备](../README.zh-CN.md#一次性准备)，再从仓库根目录执行：

```bash
python demo/adapter_spi/01_descriptor_and_probe/main.py
```

预期输出：

```text
provider=example.teaching
capabilities=4
probe_side_effect_free=true
```

## 代码在做什么

1. [`main.py`](main.py) 构造 `TeachingProvider`；发现阶段必须足够轻量，不能打开原生 client。
2. `isinstance(provider, Provider)` 只检查公共结构是否完整，不代表语义已经正确。
3. `probe()` 返回可用性和精确的不可变 Descriptor，并且不会改变 `open_count`。
4. 断言确认 Adapter 声明了受支持的 World schema，而且 probe 没有副作用。

建议同时打开 [`adapter.py`](../teaching_adapter/src/unirobosim_teaching/adapter.py)。
`CAPABILITIES` 只包含教学实现真实支持并验证过的 profile、多环境、铰接体状态与位置控制合同。

## 常见问题

- 非法 Provider ID 会在构造 `ProviderDescriptor` 时被拒绝；应使用稳定的小写点分标识。
- 不要只为检查可用性就在模块导入时加载原生 SDK。
- 不要因为厂商 SDK 理论上有相机、刚体或柔性体功能就提前声明能力；只声明本 Adapter
  已实现并测试的行为。

下一步：[02 — Session 生命周期](../02_session_lifecycle/README.zh-CN.md)。
