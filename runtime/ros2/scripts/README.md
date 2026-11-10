# Runtime 容器命令入口（Scripts）

`run.sh` 是 `runtime` 容器内的唯一命令分发脚本，在容器内手动调用并将参数透传至底层工具：

| 子命令 | 功能说明 |
| --- | --- |
| `build-ros-sdk` | 编译 `contracts/ros/robot_api_msgs` 及共享包至 `/build/ros-dev` |
| `build-policy-sdk` | 从 `policy.proto` 生成 Python Protobuf/gRPC SDK 至 `/build/generated/policy-sdk` |
| `prepare` | 将 `policy_binding.json`、运行端模板与 `seed`/`run-id` 合成为 `runtime.yaml` 与 `run_manifest.json`（`prepare_run.py`） |
| `collect` | 执行单局 ROS 示教数据采集（须显式指定全新 `--output /data/raw/<run-id>`） |
| `policy` | 执行单局非交互式 Policy 推理闭环（须显式指定全新 `--output /data/raw/<run-id>`） |
| `interactive` | 交互式执行单局 Policy 推理：自动生成唯一 `run-id`，输入 `y` 复位环境并开始，结束后保持 IDLE 直至按 `Ctrl-C` 退出 |
| `export` | 从录制目录导出逐帧 RGB、动作参考标签与图像质量报告（`export_episode.py`、`export_media.py`、`export_quality.py`） |
| `campaign` | 管理采集预算台账、执行带预算的单次尝试（`collect_attempt.py`）并筛选训练/验证集（`campaign.py`、`campaign_ledger.py`、`campaign_execution.py`） |
| `evaluate` | 生成只读评估指标与可视化对比报告（`evaluate.py`、`evaluation_metrics.py`、`evaluation_media.py`） |
| `test` | 运行固定禁用自动插件收集的 `pytest` 测试套件 |

> `run.sh` 在宿主机直接运行会报错退出（返回码 2）；请先通过 `docker compose -f runtime/ros2/docker/docker-compose.yml exec runtime bash` 进入容器。

目录导航：[上级目录](../README.md)
