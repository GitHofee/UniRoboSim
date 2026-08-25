# 04 — 能力诚实性

[English](README.md) | 简体中文

**面向对象：** 正在定义能力协商和不支持 endpoint 行为的 Adapter 开发者。
**验证级别：** 可执行公共合同负向检查。

## 运行

先完成[一次性准备](../README.zh-CN.md#一次性准备)，再从仓库根目录执行：

```bash
python demo/adapter_spi/04_capability_honesty/main.py
```

预期输出：

```text
build_rejected=unirobosim.capability.negotiation_failed
endpoint_rejected=unirobosim.capability.unsupported
```

## 代码在做什么

1. [`main.py`](main.py) 声明一个程序化铰接体和必需能力 `sensor.camera.rgb@1`。
2. `session.build()` 在分配 World 前协商完整的不可变世界。教学 Adapter 没有相机实现，
   因此抛出结构化 `CapabilityNegotiationError`，并保留后端身份。
3. 同一个仍处于 OPEN 状态的 Session 随后构建受支持的纯铰接体 World，证明被拒绝的
   build 没有留下半成品状态。
4. 程序故意调用 `world.read_sensor(handle)`。基础 World 结构包含该方法，但本 Adapter
   抛出 detail 为 `sensor.camera@1` 的 `UnsupportedCapabilityError`，不会伪造空图像。
5. 两条负向路径验证完成后关闭 World 和 Session。

## 实现原则

- Descriptor 是经过测试的承诺，不是厂商 SDK 理论能力清单。
- 必需能力不支持时，应在分配原生资源前拒绝。
- 基础 World endpoint 必须完整，但不支持的操作应抛出带 operation、backend、World 和
  capability 上下文的结构化公共错误。
- 绝不能静默近似缺失的相机、接触、柔性体或流体结果。

## 常见问题

- 能力的生命周期、校验、状态和清理路径未通过测试前就对外声明，会造成虚假可移植性。
- 返回 `None`、全零数组或占位像素会隐藏“不支持”事实。
- 直接抛厂商异常会泄漏后端细节，并破坏可移植错误处理。
- 协商失败后 Session 必须保持 `OPEN`，不能进入部分 `READY` 状态。

上一步：[03 — World 与控制](../03_world_and_control/README.zh-CN.md)。
下一步：[05 — Entry point 发现](../05_entry_point_discovery/README.zh-CN.md)。
