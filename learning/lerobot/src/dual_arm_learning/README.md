# dual_arm_learning

ACT 推理、训练和数据处理包：

- `adapters/lerobot_act.py`、`pipeline.py`、`policy_service_cli.py`、`policy_config.py`、
  `chunk_admission.py`、`temporal_ensemble.py`：纯模型适配器、分块推理流水线、gRPC
  服务入口、单层配置/契约、夹爪准入，以及按绝对仿真时间对齐的有界预测融合。
- `policy_server.py`、`policy_service.py`、`policy_types.py`、`policy_config.py`：
  Policy 服务装配、配置与公共类型。
- `dataset.py`、`sample_filter.py`、`training.py`、`checkpoints.py`、`bundle.py`、
  `budget.py`：数据转换、起点筛选、训练、检查点、模型包和预算台账。
- `cli.py`、`config.py`：命令行入口与配置解析。

训练不得下载 backbone 或依赖网络；模型、数据和缓存路径由配置和只读挂载决定。

目录导航：[上级目录](../README.md)。
