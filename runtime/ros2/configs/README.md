# Runtime 配置

`container.env` 集中声明容器内项目路径、ROS 设置与 Python 源码路径，由 Compose
加载；宿主 bind 来源由本端 Compose 决定。

`teacher_collect.yaml` 是 teacher 采集入口；`policy_rollout.yaml` 是当前
连续 ACT Policy 运行配方。配置引用 `simulation/mujoco/configs/` 的场景文件、
`profiles/control/` 的共享控制/跟随配置和 `profiles/` 的机器人语义；
`target_count` 只在 `profiles/control/panda_follower_v1.yaml` 中定义。

`policy_rollout.yaml` 中的 `FROM_VERIFIED_ACT_BUNDLE` 是显式占位，只能由
`runtime/ros2/scripts/run.sh prepare` 从 learning 端真实 bundle 生成的
`policy_binding.json` 替换；直接启动未绑定模板必须失败。

`policy_rollout.yaml` 是当前通过一次真实堆叠回归的实验配方：0.5s 执行窗口、
任务开始前 2s 仿真稳定等待；与 `backend_stack_act_chunked.yaml` 和学习侧
`act_policy_temporal.yaml` 成套使用。成功记录与适用边界见
[DSH-048](../../../docs/history/README.md#dsh-048)，新运行仍须单独分配 campaign 种子和次数。

目录导航：[上级目录](../README.md)。
