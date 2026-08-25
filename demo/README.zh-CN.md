# UniRoboSim 示例库

[English](README.md) | 简体中文

示例库把三层公共集成接口分开讲解。请选择和自己开发目标相符的路径；应用开发者不需要先学会
Adapter 内部实现。

| 路径 | 面向对象 | 适用场景 |
|---|---|---|
| [EasyAPI](easyapi/README.zh-CN.md) | 仿真应用开发者 | 用最短公共接口创建场景、发送命令并读取观测 |
| [Runtime API](runtime_api/README.zh-CN.md) | 上层框架或服务开发者 | 自主管理后端选择、生命周期、类型化 World、命令和观测 |
| [Adapter SPI](adapter_spi/README.zh-CN.md) | 仿真器接入维护者 | 不修改 Core 即可实现并发布一个新后端 |

## 开始前

Core 支持 Python 3.11 和 3.12。在源码仓库中可以先安装一次：

```bash
python -m pip install -e .
```

也可以显式使用源码运行纯 Core 案例：

```bash
PYTHONPATH=src python demo/easyapi/01_first_simulation/main.py
```

EasyAPI 01–09 和全部 Runtime API 案例使用确定性的 FakeProvider，无需安装原生仿真器即可
验证可移植合同。Fake 的运动和相机图样不代表物理或渲染保真度。

EasyAPI 案例 10 还已用 Adapter 0.10.1 在可见 Isaac Lab 3.0 / Isaac Sim 6.0.1 中通过，
并读取 1280×720 后端相机。公开 MuJoCo 与 PyBullet 版本仍要求 Core 0.9，因此它们在
Core 0.10 上的运行继续标记为待验证。

Adapter SPI 路径自带一个可安装的教学 distribution；运行前请先完成该路径的
[一次性准备](adapter_spi/README.zh-CN.md#一次性准备)。

## 验证级别

- **Core 合同：** 无仿真器 SDK 也能执行，验证 API 行为和确定性断言。
- **已安装 Distribution：** 验证当前环境中的包元数据与 Python entry point 发现。
- **干净 Wheel：** 构建产物并安装到隔离的临时环境中验证。
- **原生待验证：** 代码已面向已安装 Adapter，但本次发布闸门没有运行兼容的原生 Core/Adapter 组合。

每个编号案例都包含可执行 Python、英文 README 和中文 README。命令统一从仓库根目录运行，
程序执行有界，并用断言保证错误结果不会以零退出码悄悄通过。
