# dual_arm_runtime 核心模块

按状态所有权与职责分组：

- **节点入口与配置**：`runtime_node.py`、`runtime_callbacks.py`、`runtime_identity.py`、`runtime_limits.py`、`runtime_cli.py`、`runtime_config.py`、`runtime_errors.py`。
- **观测、控制客户端与生命周期**：`observation.py`、`robot_control_client.py`、`follower_submit.py`、`episode_runner.py`、`episode_runner_io.py`、`episode_streams.py`、`episode_finalization.py`、`shutdown.py`。
- **MCAP 录制与结构化日志**：`recording.py`、`recording_queue.py`、`run_logging.py`。
- **Policy 客户端、流式调度与离线审计**：`policy_client.py`、`policy_audit_rules.py`、`chunk_stream.py`、`stream_protocols.py`、`follower_limits.py`、`episode_audit.py`、`audit_stream.py`、`audit_common.py`、`follower_audit.py`。
- **任务管理、示教源与标签导出**：`task_client.py`、`task_result.py`、`target_source.py`、`teacher_source.py`、`label_export.py`。
- **人工控制 CLI 支持**：`control_cli.py`、`control_cli_support.py`。

目录导航：[上级目录](../README.md)
