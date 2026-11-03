# 运行入口

按用途分组：

- mock：`mock.py`、`mock_collect.py`、`mock_collect.sh`、`mock_stack.py`。
- 原生：`native_backend.py`、`native_policy.py`、`sim_check.py`。
- 采集/任务：`collect_run.py`、`collect_config.py`、`collect_artifacts.py`、
  `stack_campaign.py`、`stack_once.py`、`stack_replay.py`、`stack_export.py`、
  `stack_evaluate.py`。
- 学习：`learning.py`、`act_policy.py`。
- 部署：`dev_compose.py`。

入口解析配置、准备只读挂载并调用组件；不内嵌控制/训练算法。

目录导航：[上级目录](../README.md)。
