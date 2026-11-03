# 学习配置

- `act_dataset*.yaml`：MCAP 到 LeRobot 数据集的转换与容器路径。
- `act_training*.yaml`、`act_training_b16_20k.container.yaml`：ACT 训练配方。
- `act_sample_filter.yaml`：训练起点静止冗余筛选。
- `act_policy.yaml`、`act_policy_temporal.yaml`、`policy.yaml`、`fake_policy.yaml`、
  `replay_policy.yaml`：ACT/fake/replay Policy 服务与评估；temporal 变体仅额外打开
  按绝对仿真时间对齐的预测融合（`model.temporal_ensemble_coeff: 0.01`）。

训练配置只引用只读数据/模型路径，输出和缓存由运行挂载提供；预算记入共享台账。

目录导航：[上级目录](../README.md)。
