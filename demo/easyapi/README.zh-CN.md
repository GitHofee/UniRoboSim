# EasyAPI 示例

[English](README.md) | [简体中文](README.zh-CN.md)

这些示例面向希望直接创建和控制仿真、但不需要实现上层框架或仿真器适配器的应用开发者。建议从案例 01 开始，按编号依次学习。

案例 01–09 使用 UniRoboSim 的确定性合同测试后端 `FakeProvider`，无需安装任何仿真器 SDK；其中的运动与相机数据不代表真实物理或真实渲染精度。案例 10 使用同一份业务代码，通过显式参数选择已经安装的真实后端。

| 案例 | 学习内容 | 无仿真器 SDK 可运行 |
| --- | --- | --- |
| [01 第一个仿真](01_first_simulation/README.zh-CN.md) | 构建、启动、读取并关闭世界 | 是 |
| [02 刚体状态](02_rigid_body_state/README.zh-CN.md) | 施加力和力矩并读取类型化状态 | 是 |
| [03 铰接物体控制](03_articulation_control/README.zh-CN.md) | 按关节名控制非机器人机构 | 是 |
| [04 多环境](04_multiple_environments/README.zh-CN.md) | 指定环境发命令并局部重置 | 是 |
| [05 相机观测](05_camera_observations/README.zh-CN.md) | 高效读取 RGB、深度和法线 | 是 |
| [06 能力与 Provider](06_capabilities_and_provider/README.zh-CN.md) | 声明能力需求并检查所选 Provider | 是 |
| [07 资产包](07_asset_bundle/README.zh-CN.md) | 解析带摘要锁定的逻辑资产 | 是 |
| [08 调试与场景快照](08_debug_and_scene_snapshot/README.zh-CN.md) | 发布标记并检查可移植场景状态 | 是 |
| [09 柔性体与流体](09_deformable_and_fluid/README.zh-CN.md) | 控制柔性体节点与流体粒子 | 是 |
| [10 切换真实后端](10_switch_native_backend/README.zh-CN.md) | 用一个参数选择 Isaac Lab、MuJoCo 或 PyBullet | Isaac Lab 0.10.1 已通过；MuJoCo/PyBullet 等待 0.10 Adapter |

在仓库根目录运行无 SDK 示例：

```bash
PYTHONPATH=src python demo/easyapi/01_first_simulation/main.py
```

所有代码只导入 Core 公共 API。`FakeProvider` 必须显式从 `unirobosim.testing` 导入，它不会被生产运行时隐式选择。
