# dual_arm_runtime 包

- `runtime_node.py`、`runtime_callbacks.py`、`runtime_identity.py`、`runtime_limits.py`、
  `runtime_cli.py`、`runtime_config.py`：进程入口、订阅回调、执行身份、配置校验和运行装配。
- `observation.py`、`robot_control_client.py`、`follower_submit.py`、`episode_runner.py`、
  `episode_runner_io.py`、`episode_streams.py`、`episode_finalization.py`、`shutdown.py`：
  观测身份、控制会话、episode 生命周期、policy/teacher 流和收尾。
- `recording.py`、`recording_queue.py`、`run_logging.py`：MCAP 写入/队列和结构化事件。
- `policy_client.py`、`policy_audit_rules.py`、`chunk_stream.py`、`stream_protocols.py`、
  `follower_limits.py`、`episode_audit.py`、`audit_stream.py`、`audit_common.py`、
  `follower_audit.py`：Policy RPC、连续跟随流和录制审计。
- `task_client.py`、`task_result.py`、`target_source.py`、`teacher_source.py`、
  `label_export.py`：任务状态、结果 sidecar、目标源和标签导出。
- `control_cli.py`、`control_cli_support.py`：公共 episode 服务的人工客户端、ROS 会话和
  manifest 校验。
- `runtime_errors.py`：稳定的运行时错误分类。

状态所有权和线程边界见 [项目规范](../../../../../docs/standards.md)。

目录导航：[上级目录](../README.md)。
