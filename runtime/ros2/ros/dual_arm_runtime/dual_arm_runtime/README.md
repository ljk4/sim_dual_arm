# dual_arm_runtime 核心模块

按状态所有权与职责分组：

- **节点入口与配置**：`runtime_node.py`、`runtime_callbacks.py`、`runtime_identity.py`、`runtime_limits.py`、`runtime_cli.py`、`runtime_config.py`、`runtime_errors.py`。
- **观测、控制客户端与生命周期**：`observation.py`、`robot_control_client.py`、`follower_submit.py`、`episode_runner.py`、`episode_runner_io.py`、`episode_streams.py`、`shutdown.py`。
- **MCAP 录制与结构化日志**：`recording.py`、`recording_queue.py`、`run_logging.py`。
- **Policy 客户端与流式调度**：`policy_client.py`、`chunk_stream.py`、`stream_protocols.py`、`follower_limits.py`。
- **仿真任务与示教**。[`simulation/`](simulation/) 保存复位与任务选项、任务与 Teacher 客户端及示教源。仿真 ROS 类型在启用任务或复位时加载。普通 Policy 模式按 Robot API 能力协商，可以省略复位配置。
- **[`offline/`](offline/README.md)**：命令、采集、配置准备、导出和评估。`mcap_reader.py` 负责解码；`audit/` 提供 ROS-free 审计、标签和任务结果规则；`episode_finalization.py` 保持在线生命周期编排。
- **人工控制 CLI 支持**：`control_cli.py`、`control_cli_support.py`。

Runtime 为每个执行块计算一条轨迹，把轨迹段和原始请求一道提交给后端。取回回执时核对请求和轨迹段，传输重试沿用本次已计算的轨迹。当前 Runtime 的轨迹时长按 2 ms 网格对齐。

目录导航：[上级目录](../README.md)
