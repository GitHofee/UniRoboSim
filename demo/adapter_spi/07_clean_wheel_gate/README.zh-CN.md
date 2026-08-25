# 07 — 干净 Wheel 闸门

[English](README.md) | 简体中文

**面向对象：** 需要验收发布产物、而不是 editable 源码目录的 Adapter 维护者。
**验证级别：** 隔离安装 Core wheel 与 Adapter wheel，并执行发现冒烟测试。

## 运行

在 Linux 上使用 Python 3.11 或 3.12。当前环境必须已经具备 `pip`、`venv`、setuptools
和 wheel。从仓库根目录执行：

```bash
python demo/adapter_spi/07_clean_wheel_gate/main.py
```

最后应看到以下输出标志：

```text
clean_wheel_discovery=passed
clean_wheel_gate=passed
```

在这些标志之前会出现 wheel 构建和 pip 安装日志。

## 代码在做什么

1. [`main.py`](main.py) 不依赖当前工作目录，定位 Core 仓库、教学 Adapter 源码和
   [`discovery_smoke.py`](discovery_smoke.py)。
2. `TemporaryDirectory` 创建隔离且用完即弃的根目录。脚本使用
   `pip wheel --no-deps --no-build-isolation` 分别构建一个 Core wheel 和一个 Adapter wheel。
3. 它创建全新的临时 venv，断言正好生成两个 wheel，再使用 `pip install --no-index`
   只安装这些本地产物。
4. 每个子进程都收到环境副本，并显式删除 `PYTHONPATH`。因此源码 checkout 路径不能把
   有缺陷的 wheel 伪装成可用产物。
5. 干净解释器执行 `discovery_smoke.py`，验证 distribution 版本、已安装 entry point
   元数据、`Sim(backend="teaching")`，以及一次铰接体命令/状态闭环。
6. 离开 `TemporaryDirectory` 后，临时 wheel 和 venv 自动删除。只有全部子进程成功后才
   输出最终闸门标志。

## 实现原则

- 发布验收必须隔离测试构建产物；editable 安装会隐藏缺失包、数据、元数据或 entry point。
- 在干净安装边界清除 `PYTHONPATH` 等源码路径注入。
- 隔离安装时禁止下载依赖，才能证明恰好是哪些本地产物足以运行。
- 安装后既检查元数据发现，也检查可执行语义。
- 真实 Adapter 的发布自动化应在等价干净环境中增加锁定的原生 SDK 安装和原生冒烟测试。

## 常见问题

- `--no-build-isolation` 要求启动环境已经安装 setuptools 和 wheel。
- wheel 配置漏包时，editable 模式可能正常，但临时 venv 中会失败。
- 缺失 `[project.entry-points."unirobosim.backends"]` 元数据会导致已安装 Adapter 无法发现。
- 脚本有意清除 `PYTHONPATH`；依赖仓库源码导入属于打包缺陷，不能通过恢复变量掩盖。
- 临时环境会自动删除；CI 如需保留外部日志，应在进程退出前复制。

上一步：[06 — 公共合同套件](../06_public_contract_suite/README.zh-CN.md)。
返回 [Adapter SPI 学习路径](../README.zh-CN.md)。
