# dual_arm_runtime 核心模块

按状态所有权与职责分组：

- **节点入口与配置**：`runtime_node.py`、`runtime_callbacks.py`、`runtime_identity.py`、`runtime_limits.py`、`runtime_cli.py`、`runtime_config.py`、`runtime_errors.py`。
- **观测、控制客户端与生命周期**：`observation.py`、`robot_control_client.py`、`follower_submit.py`、`target_source.py`、`episode_runner.py`、`episode_runner_io.py`、`episode_streams.py`、`episode_finalization.py`、`shutdown.py`、`artifact_paths.py`、`scenario_runtime.py`。
- **MCAP 录制与结构化日志**：`recording.py`、`recording_queue.py`、`run_logging.py`。
- **Policy 客户端与流式调度**：`policy_client.py`、`chunk_stream.py`、`stream_protocols.py`、`follower_limits.py`。
- **仿真任务与示教**：[`simulation/`](simulation/) 保存复位与任务选项、任务与 Teacher 客户端、示教源及 `TaskState` 解析（`task_result.py`）。仿真 ROS 类型在启用任务或复位时按需加载；普通 Policy 模式按 Robot API 能力协商。
- **[`offline/`](offline/README.md)**：按 `collection/`、`export/`、`evaluation/`、`audit/` 组织的离线采集、导出、只读评估与 ROS-free 审计规则。
- **人工控制 CLI 支持**：`control_cli.py`、`control_cli_support.py`。

Runtime 为每个执行块计算一条轨迹，把轨迹段和原始请求一道提交给后端；轨迹网格优先对齐 `BackendInfo.control_grid_ns`（Marvin MuJoCo 为 `2 ms`）。取回回执时核对请求和轨迹段，传输重试沿用本次已计算的轨迹。

目录导航：[上级目录](../README.md)
