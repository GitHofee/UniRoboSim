# 06 — 能力与 Provider

[English](README.md) | [简体中文](README.zh-CN.md)

## 面向对象

适合希望在后端缺少必要行为时尽早失败的应用开发者。

## 前置条件

理解案例 01 的构建生命周期。

```bash
PYTHONPATH=src python demo/easyapi/06_capabilities_and_provider/main.py
```

## 预期结果

所选 Provider 为 `reference.fake`。程序输出必需的刚体状态能力和可选的演示能力，以及各自的 required 标志。

## 代码讲解

`require()` 声明缺少后就不能运行的行为。`optional()` 记录允许后端不满足的偏好。reason 字符串会在诊断与锁定后的世界描述中解释需求原因。`start()` 与所选 Provider 协商这些要求。随后，`provider_descriptor` 提供稳定的 Provider 身份和能力，`world_spec.requirements` 保留编译后的声明。

## 常见错误

- 能力 ID 包含显式合同版本，例如 `@1`。
- 同一个能力不能重复声明。
- Core 会自动加入 `profile.core-robotics@1`，不要假设最终只有自己写的两项。

## 验收级别

无第三方依赖的能力声明与 Provider 检查合同测试。

## 下一例

继续学习 [07 — 资产包](../07_asset_bundle/README.zh-CN.md)。
