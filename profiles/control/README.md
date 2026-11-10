# 控制与跟随配置（Control SSOT）

跨 `simulation` 与 `runtime` 共享的控制参数单一数据源（Single Source of Truth）：

- **`panda_control_v1.yaml`**：执行器最大加速度、位置/速度跟踪容差、静止判定阈值与双指接触力/速度门限。
- **`panda_follower_v1.yaml`**：受限跟随窗口点数（`target_count`，全仓唯一定义处）、目标-参考允许偏差预算与末端延伸上限。
- **`panda_stream_v1.yaml`**：连续动作块流式超时与夹爪接触卸载过渡时限。

仿真后端配置通过 `control` / `stream` / `follower` 字段引用上述文件；运行端配置通过 `control_constraints` / `follower_config` 字段引用。修改后须同时重新执行 `simulation` 与 `runtime` 的 `prepare` 命令。

目录导航：[上级目录](../README.md)
