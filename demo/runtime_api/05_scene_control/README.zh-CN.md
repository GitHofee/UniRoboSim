# 05 - 能力门控的场景控制

[English](README.md) | [Runtime API 目录](../README.zh-CN.md)

## 面向对象

本例适合开发浏览器控制台、交互式 Debugger、场景编辑器，或其他需要可移植快照与受控场景修改的
客户端。

## 本例说明什么

场景控制是可选 Runtime 扩展。程序先协商全部必要能力，再检查 `SceneControlWorld`，读取场景快照，
发送幂等 pose 命令，并消费对应增量。

## 前置条件

- 理解案例 02 的能力协商
- UniRoboSim Core 0.10；不需要浏览器和原生渲染器

## 运行

```bash
PYTHONPATH=src python demo/runtime_api/05_scene_control/main.py
```

## 预期结果

```text
capabilities=accepted initial=0 current=1 duplicate=duplicate
```

## 代码解释

1. 三个硬性要求分别覆盖 snapshot、delta 和 pose 修改。
2. 程序在访问可选协议前调用 `session.negotiate()`；协商未通过时，上层框架必须在此停止。
3. 同一组要求也写入 `WorldSpec`，因此 build 会执行最终的权威协商。
4. 只有门控通过后，代码才检查 `SceneControlWorld` 并调用它的方法。
5. `SceneCommand` 包含稳定 `command_id`、client/lease 身份、当前 World generation、实体路径、
   environment 和目标 pose。
6. 重复提交同一命令返回 `DUPLICATE`，不会再次修改场景。
7. `scene_delta(initial.sequence)` 使客户端可以增量更新现有视图，无需读取后端原生数据重建协议。

## 常见错误

- 不能因为 Python 对象碰巧有某方法就调用可选扩展；能力协商才是语义门禁。
- 必须使用当前 World generation，旧 generation 会被拒绝。
- 每个修改都需要唯一稳定的 `command_id`；重试时复用原 ID。
- Scene pose 命令用于交互和调试，不是 Controller 轨迹。

## 验收级别

本例验证能力门控、场景值合同、sequence 推进和命令幂等性。FakeProvider 不负责场景渲染或浏览器实现。

## 下一例

继续学习 [06 - 事务化构建](../06_transactional_build/README.zh-CN.md)。
