# Runtime

ROS 2 运行层：订阅 backend 状态与相机、组装观测、执行控制会话、写入 MCAP、
导出标签并处理停止和收尾。ROS 包在 `ros/dual_arm_runtime/`。

Runtime 不导入 backend 物理实现，也不训练模型。容器内统一入口为
`runtime/ros2/scripts/run.sh`，进入空闲容器后手动执行，提供 `build-ros-sdk`、
`build-policy-sdk`、`prepare`、`collect`、`policy`、`control`、`audit`、`export`、
`campaign`、`evaluate`、`test`；配置在 `configs/`，空闲环境在
`docker/docker-compose.yml`。宿主进入 `docker/` 后用 `docker compose exec runtime bash`，
不运行本目录 Python。
接口边界见[DSH-000 接口材料](../../docs/history/README.md#dsh-000)。

目录导航：[上级目录](../README.md)。
