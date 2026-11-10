# LeRobot ACT 学习与策略服务

提供无 ROS 依赖（ROS-free）的 LeRobot 数据集转换、起点筛选、ACT 混合精度训练、模型包（Bundle）导出与 gRPC Policy 推理服务。

## 目录结构

- **[`src/dual_arm_learning/`](src/dual_arm_learning/README.md)**：`dual_arm_learning` 核心 Python 包。
- **[`configs/`](configs/README.md)**：容器环境配置（`container.env`）、数据集转换（`act_dataset*.yaml`）、训练超参（`act_training*.yaml`）、起点筛选（`act_sample_filter.yaml`）与 Policy 服务模板（`policy_temporal.yaml`）。
- **[`docker/`](docker/README.md)**：学习端 `Dockerfile`、`docker-compose.yml` 与固定源码来源声明（`learning-source.json`）。
- **[`scripts/`](scripts/README.md)**：容器内统一入口 `run.sh` 与策略配置生成脚本 `prepare_policy.py`。

## 核心子命令

在 `learning` 容器内通过 `learning/lerobot/scripts/run.sh <command>` 调用：

- **`prepare-policy`**：校验 ACT 模型包并生成 `policy_binding.json`、`policy.yaml`、`act_contract.json` 与 `run_manifest.json`。
- **`convert` / `validate` / `filter`**：将合格 Episode 转换为 45/5 LeRobot 数据集、校验数据完整性、生成静止冗余起点筛选掩码。
- **`train` / `bundle` / `verify-bundle` / `budget`**：执行 ACT 训练（默认启用 PyAV、批张量直出、`num_workers`/`pin_memory` 与 CUDA AMP）、导出并校验 safetensors 模型包、查询训练预算台账。
- **`policy`**：启动 gRPC Policy 推理服务（默认监听 `50051` 端口）。
- **`test`**：运行固定禁用自动插件收集的 `pytest` 测试套件。

目录导航：[上级目录](../README.md)
