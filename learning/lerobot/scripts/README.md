# Learning 容器命令入口（Scripts）

本目录提供 `learning` 容器内的单职责入口脚本：

- **`convert.sh` / `train.sh` / `bundle.sh`**：调用 `dual_arm_learning.cli`。配方必须二选一：`--scenario <id> [--shared-root <dir>]` 通过 `<shared-root>/scenarios/<id>/binding.yaml` 选择 `recipes.dataset` 与 `recipes.training`，或直接 `--config <recipe>`；没有隐式默认配方。选中 `training` 时，其 `data.config` 必须与选中的 `dataset` 是同一文件。重复 `--set dot.path=value` 可覆盖配方中已存在的键。
- **`policy.sh`**：校验 `/build/generated/policy-sdk` 后启动 `dual_arm_learning.policy_service_cli` gRPC 推理服务。必须二选一：`--scenario <id>`（读取 `recipes.policy_service`、binding 档案与有效模型）或 `--config <recipe>`；`--bundle`/`--bundle-dir` 覆盖 `models.bundle`。服务直接从 Bundle manifest 校验模型身份，不需要也不生成 `policy_binding.json`。日志目录默认 `/data/<scenario-id>/logs/policy-service`（direct 模式 `/data/_direct/logs/policy-service`），可用 `--log-directory` 或 `ACT_LOG_DIRECTORY` 修改。
- **`test.sh`**：调用 `shared/common/tools/scripts/test.sh` 执行测试套件。

```bash
# 方案模式
scripts/convert.sh --scenario panda_stack
scripts/train.sh --scenario panda_stack --mode smoke
scripts/bundle.sh --scenario panda_stack --mode full \
  --run /data/panda_stack/models/runs/<run-dir> --output /data/panda_stack/models/bundles/stack-act
scripts/policy.sh --scenario panda_stack \
  --bundle /data/panda_stack/models/bundles/stack-act

# 直接配置模式
scripts/policy.sh \
  --config /shared/scenarios/panda_stack/configs/learning/policy_temporal.yaml \
  --bundle-dir /data/panda_stack/models/bundles/stack-act
```

在宿主机直接执行这些脚本会报错退出（返回码 2），请先进入 `learning` 容器运行。

目录导航：[上级目录](../README.md)
