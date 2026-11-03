# Runtime 配置

`mock_collect.yaml` 与 `native_collect.yaml` 是采集入口；`native_policy.yaml`、
`native_replay.yaml`、`native_task_act*.yaml`、`native_teacher.yaml` 分别选择连续
Policy、回放、任务 ACT 和 teacher 运行。所有配置引用 `configs/simulation/` 的
场景/控制文件和 `profiles/` 的机器人语义。

`native_task_act_chunked.yaml` 是当前通过一次真实堆叠回归的实验配方：0.5s执行窗口、
任务开始前2s仿真稳定等待；与 `backend_stack_act_chunked.yaml` 和学习侧
`act_policy_temporal.yaml` 成套使用。成功记录与适用边界见
[DSH-048](../../docs/history/dsh-048.md)，新运行仍须单独分配 campaign 种子和次数。

目录导航：[上级目录](../README.md)。
