# LeRobot ACT 学习与策略服务

提供无 ROS 依赖（ROS-free）的 LeRobot 数据集转换、起点筛选、ACT 混合精度训练、模型包（Bundle）导出与 gRPC Policy 推理服务。

本端持有数据转换、训练、bundle 校验与 Policy 服务实现。公共 Policy 协议与契约在 [`shared/common/interfaces/policy/`](../../shared/common/interfaces/policy/README.md) 与 [`shared/common/python/`](../../shared/common/python/README.md)；数据集、训练与服务配方由 [`shared/scenarios/<id>/`](../../shared/scenarios/) 的 binding 选择，当前实例为 [`panda_stack`](../../shared/scenarios/panda_stack/README.md) 与 [`marvin_handover`](../../shared/scenarios/marvin_handover/README.md)。

## 目录结构

- **[`src/dual_arm_learning/`](src/dual_arm_learning/README.md)**：`dual_arm_learning` 核心 Python 包。
- **[`configs/`](configs/README.md)**：容器环境配置（`container.env`）与测试替身（`fake_policy.yaml`）；真实数据集转换、训练、起点筛选与 Policy 服务配方位于 `shared/scenarios/<id>/configs/learning/`。
- **[`docker/`](docker/README.md)**：学习端 `Dockerfile`、`docker-compose.yml` 与固定源码来源声明（`learning-source.json`）。
- **[`scripts/`](scripts/README.md)**：容器内统一入口 `run.sh`。

## 核心子命令

在 `learning` 容器内通过 `learning/lerobot/scripts/run.sh <command>` 调用。所有配方命令必须二选一：`--scenario <id> [--shared-root <dir>]` 或直接 `--config <recipe>`；没有隐式默认配方。重复 `--set dot.path=value` 可覆盖配方中已存在的键。

- **`convert` / `validate` / `filter`**：将合格 Episode 转换为 45/5 LeRobot 数据集、校验数据完整性、生成静止冗余起点筛选掩码。
- **`train` / `bundle` / `verify-bundle` / `budget`**：执行 ACT 训练（默认启用 PyAV、批张量直出、`num_workers`/`pin_memory` 与 CUDA AMP）、导出并校验 safetensors 模型包、查询训练预算台账。
- **`policy`**：直接启动 gRPC Policy 推理服务（默认监听 `50051` 端口）。方案模式读取 `recipes.policy_service` 与 binding 档案，`--bundle`（或 `--bundle-dir`）覆盖 binding 的 `models.bundle`；服务直接读取 Bundle manifest 校验身份，不再生成或消费 `policy_binding.json`。

```bash
# 方案模式
learning/lerobot/scripts/run.sh convert --scenario panda_stack
learning/lerobot/scripts/run.sh train --scenario panda_stack --mode full
learning/lerobot/scripts/run.sh policy --scenario panda_stack \
  --bundle /data/panda_stack/models/bundles/stack-act

# 直接配置模式
learning/lerobot/scripts/run.sh policy \
  --config /shared/scenarios/panda_stack/configs/learning/policy_temporal.yaml \
  --bundle-dir /data/panda_stack/models/bundles/stack-act
```

`convert` 和 `train` 默认显示配置和单行进度（完成数量、百分比、用时、预计剩余时间）；训练同时显示 loss 和验证状态。编码器与库的详细输出写入启动时提示的日志文件，错误会在终端显示摘要。可通过 `--log-file <路径>` 指定日志，通过 `--no-progress` 恢复原始输出和 JSON 结果。重定向终端输出时仅打印阶段摘要。

`convert` 读取 campaign 自动写出的 `formal-select.json`，产物位于 `var/<scenario-id>/datasets/`；训练预算、checkpoint 与日志位于 `var/<scenario-id>/models/`，容器内对应 `/data/<scenario-id>/`。

目录导航：[上级目录](../README.md)
