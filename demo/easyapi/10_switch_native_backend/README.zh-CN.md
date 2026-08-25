# 10 — 切换真实后端

[English](README.md) | [简体中文](README.zh-CN.md)

## 面向对象

适合准备把同一份 EasyAPI 代码从合同测试迁移到真实仿真器的应用开发者。

## 前置条件

- UniRoboSim Core 0.10
- 已安装与 Core 0.10 匹配的目标后端适配器
- 已满足该适配器所需的原生仿真器 SDK 与运行依赖
- Isaac Lab 必须使用 Python 3.12；不要尝试把它的 Adapter 安装进 Python 3.11 的 FastSim 环境

公开 Isaac Lab Adapter `0.10.1` 已支持 Core `>=0.10,<0.11`。公开的 MuJoCo 与
PyBullet `0.9.1` 仍要求 Core `<0.10`，所以这两个选择仍需等待兼容版本。

已验证的可见 Isaac Lab 路径运行命令是：

```bash
UNIROBOSIM_ISAACLAB_LAUNCH_PROFILE=visible \
  python demo/easyapi/10_switch_native_backend/main.py --backend isaaclab
```

安装兼容版本后，只把参数替换为 `mujoco` 或 `pybullet` 即可；场景和观测代码无需修改。

## 预期结果

程序输出显式 backend 参数、所选 Provider ID、有限的方块位置，以及 RGB 形状 `(1, 720, 1280, 3)`。

## 代码讲解

`argparse` 在应用入口显式确定后端。`Sim(backend=args.backend)` 通过已安装的 `unirobosim.backends` entry point 查找指定 Provider。方块、相机、能力要求、步进、状态读取和图像读取都没有后端分支。`require()` 保证关闭相机的纯物理档位不会静默接受相机世界。断言只检查可移植结果，不接触任何后端原生对象。

## 常见错误

- `ProviderSelectionError` 通常表示适配器未安装、不可用、版本不兼容或缺少必要能力。
- Core 0.10 不应使用 `Sim(headless=False)`；可见或无头模式由适配器公开的 launch profile 选择。
- 原生图像和物理质量需要在本可移植合同冒烟测试之外单独验收。

## 验收级别

已用 Adapter 0.10.1 在可见 Isaac Lab 3.0 / Isaac Sim 6.0.1 中真实通过：执行 30 个
物理 step，选择 Provider `nvidia.isaaclab`，刚体状态为有限值，原生 RGB shape 为
`(1, 720, 1280, 3)`。MuJoCo 与 PyBullet 仍是版本阻塞，不计为通过。

## 下一步

构建直接持有 Provider、Session 和 World 的上层框架时，请学习 [RuntimeAPI](../../runtime_api/README.zh-CN.md)。
