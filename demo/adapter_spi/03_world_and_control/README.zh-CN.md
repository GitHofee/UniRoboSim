# 03 — World 与控制

[English](README.md) | 简体中文

**面向对象：** 正在实现实体查找、类型化命令、步进和类型化状态读取的 Adapter 开发者。
**验证级别：** 可执行公共合同语义检查；不会启动仿真器。

## 运行

先完成[一次性准备](../README.zh-CN.md#一次性准备)，再从仓库根目录执行：

```bash
python demo/adapter_spi/03_world_and_control/main.py
```

预期输出：

```text
joint_positions=((0.0, 0.1), (0.65, 0.1))
tick=1
```

## 代码在做什么

1. [`main.py`](main.py) 创建不可变 `WorldSpec`，其中包含一个柜子铰接体和两个环境。
   柜子说明 articulation 不一定是机器人。
2. `provider.open()` 和 `session.build(spec)` 建立明确的 Provider → Session → World 生命周期。
3. `world.resolve(EntityPath("/cabinet"))` 把稳定逻辑路径转换为归当前 World 所有、绑定
   generation 的 `EntityHandle`。
4. `ArticulationCommand` 选择环境 1、自由度 0、位置模式、一个
   `[环境, 关节]` target，以及匹配的 `rad` 单位。
5. `apply_articulation_command()` 把命令排队；`step()` 提交命令并推进 tick；
   `read_articulation()` 返回以环境为第一维的类型化状态。
6. 断言证明环境 0 和未选择的抽屉关节保持不变。`finally` 保证检查失败时也会关闭 Session。

## 实现原则

- 先解析 Path，再在每次原生调用前校验 Handle 的 Provider、Session、World、generation、
  Path、kind 与 token。
- 调用原生 SDK 前校验命令模式、选择范围、精确 target 形状和单位。
- 明确定义命令时序。本 Adapter 先排队，在下一次 `step()` 应用。
- 只返回不可变可移植状态，绝不能返回厂商关节数组或原生对象。

## 常见问题

- target 形状必须与所选环境和关节完全一致；这里是 `(1, 1)`，不是 `(2, 2)`。
- `target_units=("rad",)` 必须与所选旋转轴匹配。
- 来自已关闭或重建 World 的 Handle 已经过期，必须拒绝。
- 只关闭应用侧对象不够，Adapter 必须释放自己持有的全部原生资源。

上一步：[02 — Session 生命周期](../02_session_lifecycle/README.zh-CN.md)。
下一步：[04 — 能力诚实性](../04_capability_honesty/README.zh-CN.md)。
