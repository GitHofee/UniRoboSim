# 02 — Session 生命周期与事务化 Build

[English](README.md) | 简体中文

**面向对象：** 正在实现原生资源分配和清理的 Adapter 开发者。
**验证级别：** 可执行故障注入与生命周期检查。

## 运行

```bash
python demo/adapter_spi/02_session_lifecycle/main.py
```

预期输出：

```text
first_build=unirobosim.world.build_failed
second_build_generation=1
```

## 代码在做什么

1. `TeachingProvider(build_failures=1)` 在提交原生资源前注入一次失败，用来模拟原生场景构建错误。
2. 第一次 `session.build()` 抛出 `WorldBuildError`；Session 必须仍为 `OPEN`，且不能挂着半成品 World。
3. 用同一个不可变 `WorldSpec` 重试；第二次构建提交 generation 1，并让 Session 进入 `READY`。
4. 关闭 World 后 Session 回到 `OPEN`；`finally` 保证即使断言失败也会关闭 Session。

[`TeachingSession.build()`](../teaching_adapter/src/unirobosim_teaching/adapter.py) 的关键顺序是：
验证 → 分配候选资源 → 构造候选 World → 提交 Session 状态。仍可能失败的步骤之前不能修改存活状态。

## 常见问题

- 一个 Session 同时只能拥有一个存活 World；`READY` 状态下再次 build 必须失败。
- 构建失败不能迫使应用丢弃整个 Session。
- `close()` 必须幂等，并级联释放 Session 仍然持有的资源。

下一步：[03 — World 与控制](../03_world_and_control/README.zh-CN.md)。
