# 07 — 资产包

[English](README.md) | [简体中文](README.zh-CN.md)

## 面向对象

适合同一个逻辑机器人或物体需要为不同仿真器 Provider 提供不同原生文件的开发者。

## 前置条件

UniRoboSim Core 0.10。本案例自带一个微型 URDF 和 manifest。

```bash
PYTHONPATH=src python demo/easyapi/07_asset_bundle/main.py
```

## 预期结果

程序输出逻辑名称 `demo_robot`、selector `fake`、URDF 绝对路径及其已校验的 SHA-256 摘要。

## 文件与代码讲解

`assets/demo_robot.urdf` 是一个小型源资产。`demo_robot.asset.json` 把该文件分配给 `fake` Provider selector，并锁定内容摘要。`Path(__file__)` 让案例不依赖当前工作目录。`AssetBundle.from_manifest()` 校验 schema 并解析相对路径。把 bundle 传给 `add_articulation()` 后，会推迟到 `start()` 才按 Provider 选择具体文件。解析后的 URI 和来源信息记录在 `entity.metadata["unirobosim_asset"]` 中。

## 常见错误

- 修改 URDF 后不更新 SHA-256 会按设计触发资产身份错误。
- 真实项目通常会提供 `isaaclab`、`mujoco` 和 `pybullet` 等 selector。
- FakeProvider 只验证选择和来源，不会解析或仿真这个 URDF。

## 验收级别

无第三方依赖的 manifest 解析、Provider 选择、摘要和来源合同测试；不属于真实资产加载测试。

## 下一例

继续学习 [08 — 调试与场景快照](../08_debug_and_scene_snapshot/README.zh-CN.md)。
