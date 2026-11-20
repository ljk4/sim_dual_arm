# Fake Policy Provider

- **`fake_provider.py`**：实现确定性的 Policy Provider（声明 `test_only=true`），用于协议连通性、会话生命周期与流式动作块测试。
- **`run_fake.py`**：独立进程启动入口。
- **`fake_policy.yaml`**：与 Marvin Stack 诊断用例匹配的测试替身配置。

回放合格示教时，在测试配置中设置 `fake.replay_labels` 为该局导出的 `labels.json`，并将服务的任务标识和描述与方案配方对齐。运行端显式设置 `policy.allow_test_policy=true` 和 `policy.expected_test_only=true`，用录制时的种子复位。使用标签回放时设置 `policy.warmup_on_settle=false`，让回放时钟从实际执行的第一次预测开始。回放结果用于核对执行链路，成功率统计使用正式训练权重。

目录导航：[上级目录](../README.md)
