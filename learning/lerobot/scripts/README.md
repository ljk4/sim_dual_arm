# Learning 容器命令入口（Scripts）

`run.sh` 是 `learning` 容器内的唯一命令分发脚本，提供以下子命令：

- **`prepare-policy`**：调用 `prepare_policy.py` 校验 ACT 模型包（不加载 GPU 模型权重），动态计算模型、Processor、Profile 与 Schema 摘要，输出 `policy.yaml`、`policy_binding.json`、`act_contract.json` 与 `run_manifest.json`。
- **`convert` / `validate` / `filter` / `train` / `bundle` / `verify-bundle` / `budget` / `cli`**：调用 `dual_arm_learning.cli`，默认加载 `configs/act_dataset.container.yaml` 与 `configs/act_training.container.yaml`（可通过 `--config` 覆盖）。
- **`policy`**：校验 `/opt/policy-sdk` 后启动 `dual_arm_learning.policy_service_cli` gRPC 推理服务。
- **`test`**：调用 `infrastructure/scripts/test.sh` 执行单元测试。

在宿主机直接执行 `run.sh` 或 `prepare_policy.py` 会报错退出（返回码 2），请先进入 `learning` 容器运行。

目录导航：[上级目录](../README.md)
