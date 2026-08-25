# 06 — 公共合同套件

[English](README.md) | 简体中文

**面向对象：** 准备在增加真实仿真器验收前，让后端语义可测试的 Adapter 维护者。
**验证级别：** 只使用 Core 公共合同的 10 项可执行 pytest 检查。

## 运行

先完成[一次性准备](../README.zh-CN.md#一次性准备)，安装 `pytest`，再从仓库根目录执行：

```bash
python -m pip install pytest
python demo/adapter_spi/06_public_contract_suite/main.py
```

稳定的预期结果：

```text
10 passed
public_contract_suite=passed
```

Pytest 可能在这些标志周围增加耗时或平台文本。

## 代码在做什么

1. [`main.py`](main.py) 相对自身文件定位教学 Adapter 的 `tests/` 目录，因此可以稳定地
   从仓库根目录启动。
2. 它使用启动 demo 的同一个解释器执行 `python -m pytest -q`。
3. pytest 返回非零状态时，程序把失败状态继续传给调用者；只有全部检查通过才输出成功标志。
4. [`test_public_contract.py`](../teaching_adapter/tests/test_public_contract.py) 只导入公共
   `unirobosim` 符号和教学 distribution，刻意不导入 Core 内部实现或 `FakeProvider`。
5. 九项语义检查覆盖发现安全的 probe、生命周期、事务化重试、能力协商、单一存活 World
   所有权、命令/状态/reset、形状和单位错误、过期 Handle，以及不支持的 endpoint。

## 实现原则

- Core 0.10 **没有**发布 conformance helper 模块。本案例是 Adapter 自己维护的公共合同
  验收套件，不是调用一个不存在的 Core helper。
- 结构化 `isinstance()` 检查有用但不充分；语义行为、错误、状态迁移和清理都需要可执行断言。
- 真实 Adapter 应保留这些可移植测试，并增加原生仿真器的资源分配、真实控制、观测、渲染和
  重复清理测试。
- 负向测试同样是核心证据，因为 fail-closed 行为属于后端可移植合同的一部分。

## 常见问题

- 执行解释器没有同时安装 Core 和教学 distribution 时，会出现导入或发现失败。
- 直接执行裸 `pytest` 可能使用另一个环境，应使用 `python -m pytest`。
- 只通过结构化 Protocol 检查，不能证明命令时序、单位、批处理、过期 Handle 拒绝或清理语义。
- 不要把本套件描述为真实物理验收；教学 Adapter 是内存合同实现。

上一步：[05 — Entry point 发现](../05_entry_point_discovery/README.zh-CN.md)。
下一步：[07 — 干净 Wheel 闸门](../07_clean_wheel_gate/README.zh-CN.md)。
