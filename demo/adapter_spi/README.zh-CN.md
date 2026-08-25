# Adapter SPI 学习路径

[English](README.md) | 简体中文

这条路径面向需要把新仿真器接入 UniRoboSim 的 Adapter 开发者。示例使用
[`teaching_adapter/`](teaching_adapter/) 中一个刻意保持很小的、只支持铰接体的可安装
distribution；不需要厂商 SDK，也能逐项看清生命周期与数据边界。

教学 Adapter 不是物理引擎。它是 Provider → Session → World 公共结构、能力诚实声明、
Handle 归属、事务化构建、Python entry point 发现和打包闸门的可执行参考。

## 一次性准备

在同一个 Python 3.11 或 3.12 环境安装当前仓库的 Core 与教学 Adapter：

```bash
python -m pip install -e .
python -m pip install -e ./demo/adapter_spi/teaching_adapter
```

案例 06 还需要 `pytest`。案例 07 需要当前环境已具备 `pip`、`venv`、setuptools 和
wheel；它会自行创建并删除一个干净的临时 venv。

## 案例

| 案例 | 重点 | 验证级别 |
|---|---|---|
| [01 — Descriptor 与 Probe](01_descriptor_and_probe/README.zh-CN.md) | 只声明真实能力，并让发现过程保持轻量 | 可执行 Core-only 检查 |
| [02 — Session 生命周期](02_session_lifecycle/README.zh-CN.md) | 事务化 build 与明确状态迁移 | 可执行故障注入检查 |
| [03 — World 与控制](03_world_and_control/README.zh-CN.md) | Path 到 Handle 映射及类型化命令/状态闭环 | 可执行语义检查 |
| [04 — 能力诚实性](04_capability_honesty/README.zh-CN.md) | 拒绝不支持的能力要求和 endpoint | 可执行负向检查 |
| [05 — Entry point 发现](05_entry_point_discovery/README.zh-CN.md) | 让 `Sim(backend="teaching")` 找到已安装 Adapter | 已安装 distribution 检查 |
| [06 — 公共合同套件](06_public_contract_suite/README.zh-CN.md) | 验证生命周期、命令、reset、过期 Handle 与错误 | 10 项 public-only pytest 检查 |
| [07 — 干净 Wheel 闸门](07_clean_wheel_gate/README.zh-CN.md) | 构建两个 wheel，并在隔离 venv 验证发现 | 干净安装发布检查 |

## 原生 SDK 代码应该放在哪里

真实接入时，只在 Adapter 包内部把教学 World 的内存关节表替换成原生 SDK 对象。
应用、FastSim 和 Core 仍只能看到 UniRoboSim 可移植值。包导入、工厂构造和 `probe()`
不得启动仿真器；原生资源在 `Session.build()` 中分配，全部验证和分配成功后才提交
World；所有自有资源必须由幂等 `close()` 释放。

Core 0.10 没有发布 conformance helper 模块，因此案例 06 准确称为“公共合同验收套件”。
其中会检查结构化 `isinstance()`，但真正的证据来自语义测试。
