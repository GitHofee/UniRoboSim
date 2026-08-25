# 07 - 使用摘要固定的 BuildInput

[English](README.md) | [Runtime API 目录](../README.zh-CN.md)

## 面向对象

这是面向框架和资产管线开发者的进阶案例，用于把本地或缓存资源交给 Adapter，同时避免资产在校验
完成后、原生构建开始前发生变化。

## 本例说明什么

程序创建临时资产，记录文件身份和 SHA-256，构造规范 `BuildResourceManifest`，把它绑定到物理
`WorldSpec`，并将匹配的 `BuildInput` 传入 `Session.build()`。

## 前置条件

- 完成案例 06
- 理解 SHA-256 和本地文件身份
- UniRoboSim Core 0.10；不需要原生仿真器

## 运行

```bash
PYTHONPATH=src python demo/runtime_api/07_verified_build_input/main.py
```

## 预期结果

摘要前缀由编写的内容和合同确定，临时目录始终会被删除：

```text
manifest=<12 个十六进制字符> world=<12 个十六进制字符> entities=1
temporary_asset=removed
```

## 代码解释

1. `TemporaryDirectory` 提供隔离 source root，并在结束时删除；仓库不会保存机器相关绝对路径。
2. SHA-256 标识 manifest 期望的精确字节内容。
3. `BuildResourceEntry` 记录逻辑归属、媒体类型、角色、用途、规范 bundle 路径、字节数和摘要。
4. `LocalSourceIdentity` 从一个普通文件记录 device、inode、mode、size 和纳秒时间。
5. `BuildSourceEntry` 将 manifest resource 映射到 source root 下的相对文件。
6. `BuildInput` 要求 resource 与 source 严格一一覆盖。
7. 带资产的 v0alpha5 `WorldSpec` 保存 `manifest.sha256`；build 只接受匹配的 `BuildInput`，并在
   原生消费前重新检查源文件。
8. build fingerprint 将最终 World 与不可变 World digest 绑定。

## 常见错误

- 带资产的 v0alpha5/v0alpha6 World 必须设置 `build_resource_manifest_sha256`。
- 每个 manifest resource 必须恰好有一个规范 source entry。
- 相对路径必须是规范 POSIX 路径，不能包含 `..` 或反斜杠。
- 记录身份后修改、替换或重新链接文件，会让 build 失败，而不是消费未校验字节。
- 如果日志要求与机器无关，不要打印或持久化 `source_root`。

## 验收级别

本例使用 FakeProvider 的严格文件复查验证 Core 资产身份和构建交接，不代表原生 Adapter 能解析该
payload。

## 后续学习

你已经完成 Runtime API 路径。接下来可以阅读 [Adapter SPI 示例](../../adapter_spi/README.zh-CN.md)，
了解仿真器如何实现这些合同。
