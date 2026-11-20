# Policy 协议定义

[`policy.proto`](policy.proto) 定义 `bimanual.policy.PolicyService`。Runtime 通过 `GetPolicyInfo` 获取模型与档案身份，通过 `OpenSession` 建立会话，再调用 `Predict`、`ResetSession` 和 `CloseSession`。Learning 接收按档案排序的关节状态和相机帧，返回关节位置动作块。

- **校验**。客户端核对模型与处理器摘要、观测与动作档案哈希、关节和相机顺序、块长度、周期及 `test_only`。
- **时间基准**。`JointTarget.time_from_observation_ns` 是从 `Observation.reference_robot_time_ns` 起算的未来偏移。
- **生成 SDK**。在 Runtime 容器运行 `scripts/build_sdk.sh --policy-only`，Runtime 与 Learning 共用生成的 `dual_arm_policy` 包。修改协议后重建 SDK，再重新启动双方进程。[生成脚本](../../tools/scripts/generate_policy_sdk.py)

目录导航：[上级目录](../../README.md)
