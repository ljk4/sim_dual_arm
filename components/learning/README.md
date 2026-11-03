# 学习与策略服务

ROS-free 的 ACT 策略服务、协议适配、bundle/checkpoint 校验、数据转换和训练工具。
`src/dual_arm_learning/` 是唯一包，入口为 `cli.py` 和 `policy_server.py`，由
`tools/run/learning.py`、`tools/run/act_policy.py` 调用。

数据、模型和缓存属于运行产物，不提交。当前阶段见
[项目状态](../../docs/project-status.md)，实现历史见
[DSH-035](../../docs/history/dsh-035.md)、[DSH-036](../../docs/history/dsh-036.md)。

目录导航：[上级目录](../README.md)。
