# 配置

按用途分层，运行入口通过命令行选择配置；算法阈值不复制到 shell。

- `simulation/`：场景、Panda、backend、运动/跟随和 stream 配方。
- `runtime/`：mock/native 采集、Policy、回放、任务和 teacher 运行配置。
- `learning/`：数据转换、训练、采样筛选、Policy/fake/replay 配置。
- `deploy/`：启动器配方（`mock_collect.json`）。
- `experiments/`：正式采集活动参数（`stack_campaign.yaml`）。

结构由 `contracts/schemas/` 校验，语义由 `profiles/` 固定；规则见
[项目规范](../docs/standards.md)。

目录导航：[上级目录](../README.md)。
