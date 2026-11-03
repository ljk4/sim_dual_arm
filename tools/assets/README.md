# 资产工具

`fetch_panda.py` 按 `third_party/panda-source.json` 的固定提交 sparse checkout
MuJoCo Menagerie Panda，并生成 `.build/assets/panda.json` 摘要。`verify` 校验已有
checkout 的来源、HEAD、子目录修改和摘要；失败保留现场，不 reset/clean。

目录导航：[上级目录](../README.md)。
