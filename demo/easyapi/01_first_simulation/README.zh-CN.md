# 01 — 第一个仿真

[English](README.md) | [简体中文](README.zh-CN.md)

## 面向对象

适合掌握 Python 基础、还没有使用过 UniRoboSim 的开发者。

## 前置条件

- Python 3.11 或 3.12
- UniRoboSim Core 0.10
- 不需要真实仿真器 SDK

在仓库根目录运行：

```bash
PYTHONPATH=src python demo/easyapi/01_first_simulation/main.py
```

## 预期结果

程序会输出 Provider `reference.fake`、方块位置 `((0.0, 0.0, 0.5),)`，以及 `simulation_closed=true`。

## 代码讲解

`Sim(provider=FakeProvider(), world_id=...)` 创建可配置仿真，并显式选择合同测试后端。`add_box()` 在启动前声明一个可移植刚体。`start()` 编译所有声明并返回 `BuildReport`。`box.state` 从运行中的世界读取类型化状态。`with` 代码块退出时一定会关闭所持有的 session，最后的断言确认生命周期到达 `SimState.CLOSED`。

## 常见错误

- `FakeProvider` 必须从 `unirobosim.testing` 导入。
- 实体必须在 `start()` 前添加，EasyAPI 不允许启动后继续编辑声明。
- FakeProvider 用于验证合同，不是真实物理引擎。

## 验收级别

无第三方依赖的可执行合同冒烟测试，不验证真实物理或渲染。

## 下一例

继续学习 [02 — 刚体状态](../02_rigid_body_state/README.zh-CN.md)。
