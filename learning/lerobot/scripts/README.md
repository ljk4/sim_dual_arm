# Learning 容器命令入口（Scripts）

`run.sh` 是 `learning` 容器内的唯一命令分发脚本，提供以下子命令：

- **`prepare-policy`**：调用 `prepare_policy.py` 校验 ACT 模型包（不加载 GPU 模型权重），动态计算模型、Processor、Profile 与 Schema 摘要，输出 `policy.yaml`、`policy_binding.json`、`act_contract.json` 与 `run_manifest.json`。
- **`convert` / `validate` / `filter` / `train` / `bundle` / `verify-bundle` / `budget` / `cli`**：调用 `dual_arm_learning.cli`，默认加载 `shared/scenarios/panda_stack/configs/learning/act_dataset.yaml` 与 `act_training.yaml`（可通过 `--config` 覆盖）。
- **方案选择**：`convert`/`validate`/`train`/`bundle`/`budget` 支持 `--scenario <id> [--shared-root <dir>]`，通过 `<shared-root>/scenarios/<id>/binding.yaml` 选择 `recipes.dataset` 与 `recipes.training`；`--scenario` 与 `--config` 互斥。选中 `training` 时，其 `data.config` 必须与选中的 `dataset` 是同一文件。选择 `--scenario` 后脚本不再注入 Panda 默认配置。
- **`prepare-policy`**：支持 `--scenario`/`--template`（互斥）。方案模式读取 `recipes.policy_service` 与 binding 中的 robot/observation/action 档案；`--bundle` 覆盖 binding 的 `models.bundle`，实际加载、校验、生成的 `policy.yaml`/`policy_binding.json` 与 manifest 均使用同一有效来源。真实 bundle、processor 与身份校验不变。
- **`policy`**：校验 `/build/generated/policy-sdk` 后启动 `dual_arm_learning.policy_service_cli` gRPC 推理服务；支持 `--scenario`/`--config`（互斥），方案模式读取 `recipes.policy_service`、binding 档案与有效模型（`--bundle-dir` 可覆盖），推理阶段必须有可验证模型。
- **`test`**：调用 `shared/common/tools/scripts/test.sh` 执行单元测试。

在宿主机直接执行 `run.sh` 或 `prepare_policy.py` 会报错退出（返回码 2），请先进入 `learning` 容器运行。

目录导航：[上级目录](../README.md)
