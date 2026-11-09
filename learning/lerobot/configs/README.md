# 学习配置

`container.env` 集中声明容器内源码、缓存和离线运行设置，由 Compose 加载；
宿主 bind 来源由本端 Compose 决定。

- `act_dataset*.yaml`：MCAP 到 LeRobot 数据集的转换与容器路径。
- `act_training*.yaml`：ACT 训练配方，`.container.yaml` 使用 `/data` 共享产物
  与 `/opt/learning-cache` 只读权重。
- `act_sample_filter.yaml`：训练起点静止冗余筛选。
- `act_policy_temporal.yaml`：真实 ACT Policy 服务模板；`prepare-policy` 读取它并
  写出带真实 bundle 路径/摘要的服务配置。它打开按绝对仿真时间对齐的预测融合
  （`model.temporal_ensemble_coeff: 0.01`）。
- `fake_policy.yaml`：仅测试使用的 fake provider 模板，不能冒充正式 ACT。

训练配置只引用只读数据/模型路径，输出和缓存由 `/data` 挂载提供；预算记入共享台账。
`FROM_VERIFIED_ACT_BUNDLE` 只允许出现在 runtime 模板中，并由
`runtime/ros2/scripts/run.sh prepare` 用真实 binding 替换。

目录导航：[上级目录](../README.md)。
