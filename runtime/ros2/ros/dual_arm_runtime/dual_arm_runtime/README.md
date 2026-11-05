# dual_arm_runtime 包

- `node.py`、`native_node.py`、`cli.py`、`native_cli.py`、`configuration.py`、
  `native_config.py`：进程入口、配置校验和运行装配。
- `observation.py`、`control_client.py`、`native_control_client.py`、`episode.py`、
  `native_episode.py`、`shutdown.py`：观测身份、控制会话、episode 和收尾。
- `recording.py`、`recording_queue.py`、`audit.py`、`native_audit.py`、
  `run_logging.py`：MCAP 写入/队列、审计和结构化事件。
- `policy_client.py`、`policy_audit_rules.py`、`native_stream.py`、
  `native_follower_audit.py`：Policy RPC、连续流和跟随审计。
- `stack_task_client.py`、`task_result.py`、`target_source.py`、`teacher_source.py`、
  `label_export.py`：任务状态、结果 sidecar、目标源和标签导出。
- `native_errors.py`：稳定的原生错误分类。

状态所有权和线程边界见 [项目规范](../../../../../docs/standards.md)。

目录导航：[上级目录](../README.md)。
