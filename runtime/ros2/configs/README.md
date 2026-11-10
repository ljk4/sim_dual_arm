# Runtime 配置说明

- **`container.env`**：声明容器内项目根路径（`PROJECT_ROOT=/workspace`）、ROS 2 域 ID（`ROS_DOMAIN_ID=42`）、Fast DDS 配置路径与 Python 源码路径，由 Compose `env_file` 自动加载。
- **`teacher_collect.yaml`**：程序示教（Teacher）采集配置模板。
- **`policy_rollout.yaml`**：分块 ACT 策略推理运行模板（配置 0.5 s 执行窗口与任务开始前 2.0 s 仿真稳定等待，与仿真侧 `backend_stack_act_chunked.yaml` 及学习侧 `policy_temporal.yaml` 配套使用）。
- **[`experiments/`](experiments/README.md)**：正式数据采集活动预算与台账配置。

## 引用与绑定规则

1. **控制配置单一数据源**：配置通过相对路径引用 [`simulation/mujoco/configs/scene/`](../../../simulation/mujoco/configs/scene/README.md) 下的场景文件、[`profiles/control/`](../../../profiles/control/README.md) 下的共享控制/跟随参数以及 [`profiles/`](../../../profiles/README.md) 下的机器人语义档案。`target_count` 唯一定义于 `profiles/control/panda_follower_v1.yaml`。
2. **动态 Policy 绑定**：`policy_rollout.yaml` 中的 `FROM_VERIFIED_ACT_BUNDLE` 占位符必须通过 `runtime/ros2/scripts/run.sh prepare --policy-binding <policy_binding.json>` 替换为真实模型包摘要后方可运行；直接加载含未替换占位符的模板会立即报错拒绝。

目录导航：[上级目录](../README.md)
