# 仿真配置

- `panda_stack*.yaml`：双 Panda 场景、学习相机变体和任务/推理/teacher 变体。
- `backend*.yaml`：MuJoCo 后端、跟随和任务后端选择。
- `panda_control.yaml`、`panda_follower*.yaml`、`panda_stream.yaml`：控制、受限
  跟随和连续流参数。
- `kinematics_check.yaml`、`scene_check.yaml`：一次性场景/可达性检查。

场景加载会校验 profile 的关节/相机语义；修改几何或相机需同步更新对应 profile。

`backend_stack_act_chunked.yaml` 与 `panda_follower_act_chunked.yaml` 是已验收的 ACT
实验组合，须配 runtime 的同名 chunked 配方和 temporal Policy。0.30rad/0.04m
是用户批准的策略目标—参考偏差预算，原机器人硬限位和实测跟踪阈值保持不变。

目录导航：[上级目录](../README.md)。
