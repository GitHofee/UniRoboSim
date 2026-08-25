# 01 - Provider、Session 与 World

[English](README.md) | [Runtime API 目录](../README.zh-CN.md)

## 面向对象

如果你准备基于 UniRoboSim 开发上层框架，但还没有使用过严格的 Runtime API，请从这里开始。

## 本例说明什么

Runtime 的资源所有权分为三层：`Provider` 描述并打开后端，`Session` 持有一次后端连接，
`World` 持有一次已经构建的仿真。本例检查三种公开结构协议，并按相反顺序释放资源。

## 前置条件

- Python 3.11 或 3.12
- UniRoboSim Core 0.10
- 不需要原生仿真器；`FakeProvider` 是 Core 自带的合同测试工具

## 运行

在仓库根目录执行：

```bash
PYTHONPATH=src python demo/runtime_api/01_provider_session_world/main.py
```

## 预期结果

```text
provider=reference.fake available=True
world=runtime-lifecycle generation=1 entities=1
lifecycle=closed-cleanly
```

## 代码解释

1. `FakeProvider()` 提供公开合同的确定性实现，`isinstance(provider, Provider)` 检查结构接口。
2. `probe()` 只检查可用性，不打开 Session，也不构建 World。
3. `EntitySpec` 描述一个带门铰链的柜子。铰接物体不一定是机器人。
4. `WorldSpec` 是上层框架编译出的不可变输入。
5. `session.build(spec)` 将 Session 从 `OPEN` 切换到 `READY`，并返回持有仿真状态的 World。
6. 两层 `try/finally` 先关闭 World，再关闭 Session；该写法适用于所有合规 Adapter。

## 常见错误

- articulation 没有 `joint_names` 时配置无效。
- `WorldSpec` 至少需要一个实体。
- 同一 Session 中构建第二个活动 World 会触发 `LifecycleError`。
- 上层不要保存原生 SDK handle，应保存 `EntityPath` 和 `World.resolve()` 返回的可移植 handle。

## 验收级别

这是不依赖原生仿真器的 Core 合同冒烟测试，只验证生命周期和可移植类型，不验证真实物理、渲染或
原生 SDK 清理。

## 下一例

继续学习 [02 - 能力选择](../02_capability_selection/README.zh-CN.md)。
