# Learning 容器命令入口（Scripts）

`run.sh` 是 `learning` 容器内的唯一命令分发脚本，提供以下子命令：

- **`convert` / `validate` / `filter` / `train` / `bundle` / `verify-bundle` / `budget` / `cli`**：调用 `dual_arm_learning.cli`。配方必须二选一：`--scenario <id> [--shared-root <dir>]` 通过 `<shared-root>/scenarios/<id>/binding.yaml` 选择 `recipes.dataset` 与 `recipes.training`，或直接 `--config <recipe>`；没有隐式默认配方。选中 `training` 时，其 `data.config` 必须与选中的 `dataset` 是同一文件。重复 `--set dot.path=value` 可覆盖配方中已存在的键。
- **`policy`**：校验 `/build/generated/policy-sdk` 后启动 `dual_arm_learning.policy_service_cli` gRPC 推理服务。必须二选一：`--scenario <id>`（读取 `recipes.policy_service`、binding 档案与有效模型）或 `--config <recipe>`；`--bundle`/`--bundle-dir` 覆盖 `models.bundle`。服务直接从 Bundle manifest 校验模型身份，不需要也不生成 `policy_binding.json`。日志目录默认 `/data/<scenario-id>/logs/policy-service`（direct 模式 `/data/_direct/logs/policy-service`），可用 `--log-directory` 或 `ACT_LOG_DIRECTORY` 修改。
- **`test`**：调用 `shared/common/tools/scripts/test.sh` 执行测试套件。

```bash
# 方案模式
learning/lerobot/scripts/run.sh validate --scenario panda_stack
learning/lerobot/scripts/run.sh convert --scenario panda_stack
learning/lerobot/scripts/run.sh train --scenario panda_stack --mode smoke
learning/lerobot/scripts/run.sh bundle --scenario panda_stack --mode full \
  --run /data/panda_stack/models/runs/<run-dir> --output /data/panda_stack/models/bundles/stack-act
learning/lerobot/scripts/run.sh policy --scenario panda_stack \
  --bundle /data/panda_stack/models/bundles/stack-act

# 直接配置模式
learning/lerobot/scripts/run.sh policy \
  --config /shared/scenarios/panda_stack/configs/learning/policy_temporal.yaml \
  --bundle-dir /data/panda_stack/models/bundles/stack-act
```

在宿主机直接执行 `run.sh` 会报错退出（返回码 2），请先进入 `learning` 容器运行。

目录导航：[上级目录](../README.md)
