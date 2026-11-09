# 控制与跟随配置

跨 simulation 与 runtime 共享的控制配置单一数据源：

- `panda_control_v1.yaml`：执行器加速度、位置/速度容差、稳定判定和接触阈值。
- `panda_follower_v1.yaml`：跟随窗口 `target_count`、目标偏差预算和末端延伸上限。
- `panda_stream_v1.yaml`：连续动作块流超时与接触卸载过渡。

backend YAML 通过 `control`/`stream`/`follower` 引用这些文件；runtime YAML 通过
`control_constraints`/`follower_config` 引用。`target_count` 只在
`panda_follower_v1.yaml` 中定义。

目录导航：[上级目录](../README.md)。
