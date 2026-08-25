# Runtime API 示例

[English](README.md)

这些示例面向基于 UniRoboSim 构建框架、服务、调度器和基础设施的开发者。学习路径从显式的
Provider -> Session -> World 生命周期开始，逐步进入类型化控制与观测、可选场景控制、事务化
恢复和经过摘要校验的资产输入。

所有示例默认使用 `unirobosim.testing.FakeProvider`。它是确定性的合同测试后端，不是真实
物理仿真器，也不会产生真实渲染。在理解合同流程后，可以把 Provider 工厂替换成兼容的已安装
Adapter。

## 从源码仓库运行

在仓库根目录使用 Python 3.11 或 3.12：

```bash
PYTHONPATH=src python demo/runtime_api/01_provider_session_world/main.py
```

如果环境中已经安装 UniRoboSim 0.10，可以省略 `PYTHONPATH=src`。

## 学习顺序

| 案例 | 重点 |
|---|---|
| [01](01_provider_session_world/README.zh-CN.md) | 显式管理 Provider、Session 和 World |
| [02](02_capability_selection/README.zh-CN.md) | 延迟 Registry 与按能力选择 Provider |
| [03](03_articulation_command/README.zh-CN.md) | 类型化批量铰接控制和状态 |
| [04](04_rigid_body_and_camera/README.zh-CN.md) | 刚体 wrench 与紧凑相机观测 |
| [05](05_scene_control/README.zh-CN.md) | 能力门控的快照、增量和幂等修改 |
| [06](06_transactional_build/README.zh-CN.md) | 可重试构建与旧 Handle 拒绝 |
| [07](07_verified_build_input/README.zh-CN.md) | 使用摘要固定的本地资产交接 |

每个程序都包含可执行断言。退出码为零表示 FakeProvider 满足该案例展示的可移植合同。真实动力学、
渲染质量、原生 SDK 生命周期和 GPU 行为必须由所选 Adapter 的独立验收测试确认。
