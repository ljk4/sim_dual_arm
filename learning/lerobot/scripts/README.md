# Learning 容器命令入口（Scripts）

本目录提供 `learning` 容器内的单职责入口脚本：

- **`convert.sh` / `train.sh`** 调用 `dual_arm_learning.cli`。使用 `--robot <robot> --task <task> [--shared-root <dir>]` 从 Binding 选择 `recipes.dataset` 和 `recipes.training`，或使用 `--config <recipe>` 直接选择配方。场景训练配方的 `data.config` 指向绑定中的数据集配方。转换支持 `--select` 和 `--output-root`，训练支持 `--mode`、`--max-steps` 和 `--data-root`。其余参数在配方中修改。
- **`server_policy.sh`**：校验 `/build/generated/policy-sdk` 后启动 `dual_arm_learning.policy_service_cli` gRPC 推理服务。必须二选一：`--robot <robot> --task <task>`（读取 `recipes.policy_service`、binding 档案与模型）或 `--config <recipe>`；`--model` 接受模型目录或 `model.safetensors`，省略时自动选择 `/data/<robot>/<task>/models/` 下最新的 `act_*`。模型身份由 `model.safetensors` 元数据校验，不需要也不生成 `policy_binding.json`。日志目录默认 `/data/<robot>/<task>/logs/policy-service`（direct 模式 `/data/_direct/logs/policy-service`），可用 `--log-directory` 或 `ACT_LOG_DIRECTORY` 修改。
- **`test.sh`**：调用 `shared/common/tools/scripts/test.sh` 执行测试套件。

`convert.sh` 读取合格采集清单的 `selected` 数组，按 `validation_percent` 随机抽取整局作为验证集。验证条数按比例四舍五入，两组均至少保留一局。相同清单顺序和比例可复现划分，转换后的数据集清单记录各组实际成员供训练读取。

试跑小数据集时，在转换配方的 `expected.formal_episodes` 填合格局数，至少两局。训练配方的 `data.config` 引用同一份转换配方。

```bash
# 机器人/任务模式
scripts/convert.sh --robot marvin_sim --task stack
scripts/train.sh --robot marvin_sim --task stack --mode smoke
scripts/server_policy.sh --robot marvin_sim --task stack

# 直接配置模式
scripts/server_policy.sh \
  --config /shared/scenarios/marvin_sim/stack/configs/learning/policy.yaml \
  --model /data/marvin_sim/stack/models/act_20260101_000000
```

在宿主机直接执行这些脚本会报错退出（返回码 2），请先进入 `learning` 容器运行。

目录导航：[上级目录](../README.md)
