# 公共契约

协议源码的唯一维护位置；职责与数据语义见 [接口设计](../docs/architecture-and-interfaces.md)，目录规则见 [组织规范](../docs/repository-layout.md)。

- `policy/v1/policy.proto`：推理协议；`JointTarget.time_from_observation_ns` 相对原 Observation 机器人时间，`PolicyInfo` 显式 60 点、50 ms、完整 16 维顺序和 `test_only`。SDK 由 `tools/contracts/generate_policy_sdk.py` 生成，ROS-free 服务/typed provider 见 `components/learning`。
- `ros/robot_api_msgs/`：已实现的 ROS msg/srv/action 包；DSH-019 新增 `CommandStatus`、`QueryCommand` 与独立 `command_feedback` 流，既有消息字段和 `ExecuteTrajectory.action` 保持不变。
- `schemas/v1/`：历史 mock profile、观测、动作、运行配方与 episode manifest 的结构定义。
- `schemas/v2/`：具名相机集合的 RobotProfile 与 v2 episode manifest；v2 profile 支持单/多相机和显式 `test_only`。
- `schemas/v3/`、`schemas/v4/`：原生 MOTION 与连续 policy follower 的 manifest；v4 只由 policy 运行显式选择，默认和历史版本不变。
- `python/`：无 ROS/ML 依赖的有类型数据模型、配置校验、命令状态校验与语义哈希；`robot_contracts.status` 集中校验身份、数组顺序、时间和 `reference - measured`。`robot_contracts.reference` 保存显式升幂三次段，提供不可变采样、严格区间采样、连续极值和约束验证，支持显式起止参考速度但默认仍为零端点速度，不导入NumPy/SciPy；`robot_contracts.action_chunks` 提供仅用标准库的观测时间锚定`ActionWindow`与交接`ChunkAnchor`。仿真侧`reference_compiler`负责一次性SciPy编译，`chunk_plan`在物理循环外预编译并验证不可变`PreparedChunk`，不接状态机。

机器人语义实例放在仓库根 profiles，其版本、绑定顺序和三相机校准摘要算法见
[profiles/README.md](../profiles/README.md)。部署绑定放 deploy。生成的 ROS/Protobuf
文件通过构建生成，不在各组件里复制或手工修改。
