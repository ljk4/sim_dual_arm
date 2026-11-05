# 运行入口

容器内功能脚本是 `run.sh`，进入空闲容器后手动执行。
它按 case 分发到已有 CLI，并透传参数：

- `build-ros-sdk` / `build-policy-sdk`：调用真实 SDK 生成脚本。
- `prepare`：把通用 Policy binding、runtime 模板、seed/run-id 生成为
  `runtime.yaml` + `run_manifest.json`。
- `collect` / `policy`：运行原生 ROS 采集或 Policy 执行；必须显式给
  `--output /data/raw/<run-id>`，不共享同一输出目录。
- `control`：使用公共 runtime status/start/end 客户端（`--help` 为 argparse 帮助）。
- `audit`：进入 `dual_arm_runtime.audit.main`；原生 v3 由 `inspect_episode` 分发。
- `test`：固定 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` 的 pytest 入口。
- `export`、`campaign`、`evaluate`：导出标签/RGB/质量、预算台账与评估报告。
- `stack_campaign.py`、`stack_export.py`、`stack_evaluate.py`、
  `collect_artifacts.py` 是上述命令复用的业务实现。
- `prepare_run.py` 是纯配置准备，不导入 learning 或 MuJoCo。

`run.sh` 在宿主执行会明确失败；先在 `runtime/ros2/docker` 运行
`docker compose exec runtime bash`，再执行 `runtime/ros2/scripts/run.sh <功能> ...`。
一次性命令可用 `docker compose exec runtime runtime/ros2/scripts/run.sh <功能> ...`。
入口不内嵌控制、训练或模型算法。

目录导航：[上级目录](../README.md)。
