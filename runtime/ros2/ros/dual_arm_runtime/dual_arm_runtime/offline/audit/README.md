# 纯 Python 审计规则

`records.py` 定义读取层提供的不可变状态、相机、观测和任务事实。命令、反馈和参考优先使用 `robot_contracts` 模型。

- `audit_common`：边界、相机校验和数值比较。
- `audit_stream`、`episode_audit`、`follower_audit`：流、episode 与 follower 审计状态机。
- `policy_audit_rules`：策略完成约束。
- `label_export`：按实际应用参考生成标签。
- `task_result`：任务结果与 sidecar 序列化规范（在线 `TaskState` 解码位于上层 `dual_arm_runtime.simulation.task_result`）。

禁止导入 ROS、MCAP 或消息转换库。读取与解码由上级 `export/mcap_reader.py` 处理；在线生命周期由包根 `episode_finalization.py` 编排。

验证：`python3 -m pytest -p no:cacheprovider -q tests/unit/runtime/test_offline_audit.py tests/unit/runtime/test_policy_audit_rules.py tests/unit/runtime/test_stack_label_export.py tests/unit/runtime/test_task_result_sidecar.py`。
