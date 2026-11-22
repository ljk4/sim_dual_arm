# 公共契约与共享实现（Common）

本目录存放跨端稳定、与具体任务和实现无关的公共内容，不持有仿真、运行或学习业务生命周期，也不选择方案。任务、机器人、后端与学习算法的选择在 [`shared/scenarios/<robot>/<task>/binding.yaml`](../scenarios/marvin_sim/stack/README.md)。

## 目录结构

- **[`interfaces/policy/policy.proto`](interfaces/policy/policy.proto)**：gRPC Policy 推理服务与动作块（Action Chunk）协议定义。
- **[`interfaces/ros/`](interfaces/ros/README.md)**：通用 `robot_api_msgs` 与可选仿真任务 `sim_api_msgs`；机器人/方案专用 ROS 扩展位于 `shared/scenarios/<robot>/extensions/` 或 `shared/scenarios/<robot>/<task>/extensions/`。
- **[`interfaces/schemas/`](interfaces/schemas/README.md)**：机器人 Profile、观测、动作、运行事件、Episode Manifest、正式采集清单（`formal-select.json`）与导出结构（`labels.json`/`quality.json`）的 JSON Schema 定义。运行端私有配方由所属端的数据模型校验。
- **[`python/`](python/README.md)**：`robot_contracts`、`robot_ros_support`、`robot_trajectory`、`robot_run_logging` 四个独立 Python 包。
- **[`tools/`](tools/README.md)**：跨端容器配置、SDK 构建、测试入口、binding 只读校验、边界检查与宿主机并行编排（`host/`）工具。

生成的 SDK 与构建产物输出至运行时创建的 `build/`，不提交生成物或空目录，也不在各运行端复制接口源码。初始化与操作见[根 README](../../README.md)。

目录导航：[上级目录](../README.md)
