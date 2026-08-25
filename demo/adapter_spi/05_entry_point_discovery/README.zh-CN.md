# 05 — Entry point 发现

[English](README.md) | 简体中文

**面向对象：** 正在把 Adapter 打包成可被普通应用发现的 distribution 维护者。
**验证级别：** 已安装 distribution 的发现与 EasyAPI 选择检查。

## 运行

本案例要求教学 Adapter 作为 Python distribution 安装。仅把源码目录放进 `PYTHONPATH`
并不等价，因为 entry point 元数据来自已安装 distribution。从仓库根目录执行：

```bash
python -m pip install -e .
python -m pip install -e ./demo/adapter_spi/teaching_adapter
python demo/adapter_spi/05_entry_point_discovery/main.py
```

预期输出：

```text
selected_provider=example.teaching
joint_positions=((0.5,),)
```

## 代码在做什么

1. [`teaching_adapter/pyproject.toml`](../teaching_adapter/pyproject.toml) 在
   `unirobosim.backends` 组注册名为 `teaching` 的 entry point，并指向公共的无参数
   `create_provider` 工厂。
2. [`main.py`](main.py) 读取已安装的 entry point 元数据，并断言 `teaching` distribution
   entry 存在。
3. `Sim(backend="teaching")` 让 EasyAPI 发现逻辑加载这个具名工厂、probe 返回的 Provider，
   并协商场景要求。
4. 后续全部是普通的后端无关 EasyAPI：声明铰接体、启动、发命令、步进和读取状态。
5. 断言同时证明已安装发现链路和所选 Descriptor 身份正确。

## 实现原则

- Entry point 名是简洁的用户选择器；`ProviderDescriptor.provider_id` 是稳定且全局有意义的
  后端身份。
- 注册的工厂必须无参数，并且不能启动仿真器。
- 厂商特定配置应通过显式 Provider 构造或有文档的 launch profile 提供，不能藏在全局发现中。
- 包名、导入包、entry point 名、Provider ID 与文档应有意保持一致性，即使它们不是完全相同的字符串。

## 常见问题

- 未安装教学 distribution 就运行，会触发明确的
  `install demo/adapter_spi/teaching_adapter first` 断言。
- Core 与 Adapter 安装在不同 Python 环境时，执行解释器看不到其元数据。
- 工厂在发现阶段导入或启动原生 SDK，会让后端选择变慢且不安全。

上一步：[04 — 能力诚实性](../04_capability_honesty/README.zh-CN.md)。
下一步：[06 — 公共合同套件](../06_public_contract_suite/README.zh-CN.md)。
