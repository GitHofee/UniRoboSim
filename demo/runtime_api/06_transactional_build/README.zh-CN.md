# 06 - 事务化构建与旧 Handle

[English](README.md) | [Runtime API 目录](../README.zh-CN.md)

## 面向对象

本例面向实现重试、Worker 恢复或长时间运行服务的框架开发者；在这些场景中，一次构建失败不能污染
仿真连接。

## 本例说明什么

FakeProvider 注入一次构建失败。Session 保持打开并能够重试。关闭第一个成功构建的 World 后，重新
构建会推进 generation，新 World 会拒绝旧实体 handle。

## 前置条件

- 理解案例 01 的显式生命周期
- UniRoboSim Core 0.10；不需要原生仿真器

## 运行

```bash
PYTHONPATH=src python demo/runtime_api/06_transactional_build/main.py
```

## 预期结果

```text
first_build=failed operation=session.build session=open
retry=passed old_handle=stale generation=1->2
```

## 代码解释

1. `FakeProvider(build_failures=1)` 是确定性的故障注入器。
2. 第一次 `session.build()` 在提交 World 前抛出 `WorldBuildError`。
3. `SessionState.OPEN` 证明失败是事务化且可重试的。
4. 下一次构建成功并得到 generation 1，随后为 `/arm` 创建 handle。
5. 关闭 World 后 Session 回到 `OPEN`；重新构建得到 generation 2。
6. 使用 generation 1 的 handle 读取新 World 会触发 `StaleHandleError`，防止上层误控替代实体。

## 常见错误

- 不要遇到任意 `WorldBuildError` 就直接销毁健康 Session；应检查结构化错误和 Adapter 策略。
- 上一个 World 仍存活时，同一 Session 不能再次 build。
- reset 不会产生新 World generation；关闭并重新构建才会。
- 每次 rebuild 后重新 resolve handle，不要自行修改 generation 字段。

## 验收级别

这是确定性的 Core 事务和 handle 身份测试。原生 Adapter 还必须在自己的资源分配边界注入故障并
证明清理正确。

## 下一例

继续学习 [07 - 校验后的构建输入](../07_verified_build_input/README.zh-CN.md)。
