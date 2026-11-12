# 阶段 08 交接：轨迹表示与生产者来源分离

- 日期：2026-11-11（Asia/Shanghai）。状态：完成（代码与验证）；未进入阶段 09。
- 依据：总要求、阶段 08、progress、阶段 07 交接。未改控制参数、数值门禁、Panda 模型/物理、旧 MCAP、prepared、模型或正式台账。

## 字段与版本语义

- `FollowerReference.curve_duration_ns`（ROS `uint64 curve_duration_ns`，原 `ruckig_duration_ns`）：生产者完整曲线的总时长，在非最终块中先按 hand-off 截断后导出，最终块可覆盖窗口并延伸到终态。原关系保持不变：必须覆盖 `plan.duration_ns`；最终块的该值落在 `duration_grid_ns` 上；非最终导出时长等于请求窗口并落在网格上。`extension_ns`、计划段、采样和终态检查均未改。
- `PreparedFollowerPlan` 新增必填的 `algorithm_version`，由生产者填写并原样带入 `FollowerReference`。simulation 的 `RUCKIG_ALGORITHM_VERSION = "bounded-target-follower/ruckig-0.15.3/v1"` 保留真实来源；公共层只要求非空字符串，不再与固定常量比较。
- 公共数值校验仍只依赖曲线表示、采样和归一化约定：`sampler_version`、`normalization_version` 继续严格校验共享值；`algorithm_version` 仅保留与展示，不构成接受条件。关节数 1～64、段数 1～128、3 秒 horizon、时间与系数检查均未改。
- simulation 控制指纹通过 `PreparedFollowerPlan` 的规范化内容覆盖 `algorithm_version`，不再单独引用共享常量。
- `robot_api_msgs` 0.2.0 → 0.3.0：仅 `FollowerReference` 字段改名与注释更新，CDR 字段类型、顺序、边界和嵌套定义不变。能力标识 `execution_feedback.gripper_set.v1` 未变，未新增能力。
- `FOLLOWER_REFERENCE_DIGEST_VERSION` → `follower-reference-v2`；摘要仍覆盖同一组身份、来源、曲线、限位与时间字段。新增的 `follower-reference-v1` 载荷只用于校验旧记录，旧记录解码后携带按当前载荷重算的摘要。

## 入口与调用方

- 契约：`shared/common/python/contracts/src/robot_contracts/trajectory/follower_reference.py`；常量文件不再导出 `FOLLOWER_ALGORITHM_VERSION`。
- 共享数学：`shared/common/python/trajectory/src/robot_trajectory/follower.py`、`follower_reference.py`。
- IDL：`shared/common/interfaces/ros/robot_api_msgs/msg/FollowerReference.msg`；包版本 0.3.0。
- ROS 转换：`shared/common/python/ros_support/src/robot_ros_support/follower_conversions.py`，`follower_reference_from_message(..., legacy_digest=False)`。
- 生产者：`simulation/mujoco/src/robot_backend_sim/follower/follower_plan.py` 使用 `RUCKIG_ALGORITHM_VERSION` 并写入 `curve_duration_ns`；`control/control_state.py` 指纹改用计划自带的算法信息。
- 消费者：runtime 在线回调与 `robot_control_client` 使用默认（当前）摘要；导出与审计在读取定义后显式选择旧摘要载荷。

## 历史读取

- 新增 `runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/offline/follower_definition.py`，复用 `feedback_definition.normalized_definition`。录制定义等于安装定义时按当前格式读取；仅在整份定义与安装定义只差 `uint64 ruckig_duration_ns` → `uint64 curve_duration_ns` 这一行时识别为已知旧格式，其他差异明确拒绝。旧 CDR 与当前生成类型布局一致，直接由当前类型解码，不另行实现旧 SDK 或迁移框架。
- `mcap_reader._run_audit` 与 `export_episode._read_metadata` 在解码任何 `FollowerReference` 前调用该检查，并把识别结果传给解码器。旧记录用 v1 载荷校验存储摘要，随后按 v2 载荷重建记录摘要；不带旧摘要标志读取旧记录会以 `FOLLOWER_DIGEST_MISMATCH` 拒绝。
- 真实样本 `var/raw/interactive-20261111T025539Z-1047/.../recording`：识别为旧定义；`/robot/follower_references` 首块解码得到 `algorithm_version="bounded-target-follower/ruckig-0.15.3/v1"`、`curve_duration_ns=530000000`、`plan_duration_ns=490000000`、`requested_window_ns=490000000`、`is_final=false`、`duration_grid_ns=2000000`。用录制 manifest 内 profile 与 follower/control 配置重建 `validate`：计划一致、标志一致、起点应用位置一致。证据在 `.build/phase-08-20261111/legacy-follower-sample.json` 与 `legacy-follower-validation.json`。
- 现有 bag 同时带 0.1.0 反馈布局，完整导出/审计仍会被反馈入口拒绝；本阶段只验证轨迹字段读取与数值重建，完整历史 episode 留阶段 12。

## SDK

```bash
# runtime 容器，先 source /opt/ros/jazzy/setup.bash
python3 /shared/common/tools/scripts/dev_workspace.py build --project-root /workspace \
  --shared-root /shared --build-root /build/ros-core
PYTHONPATH=/shared/common/python/contracts/src:$PYTHONPATH \
python3 /shared/common/tools/scripts/dev_workspace.py build --project-root /workspace \
  --shared-root /shared --scenario panda_stack --build-root /build/ros-dev
```

- 生成文件未手改。核心 `robot_api_msgs`：`14aa6360…`；完整 3 包：`b79e8a7e…`；两次 `check` 均通过。生成类型字段为 `curve_duration_ns`。Policy SDK 未改未重建。

## 验证结果

- ruff：本阶段改动文件全部通过。
- 契约/轨迹/tests：`test_follower_reference.py`、`test_follower_contracts.py`、`test_mcap_feedback_guard.py` 81 passed。覆盖同种曲线携带不同生产者信息、算法信息不参与门禁、空算法标识拒绝、v1/v2 摘要区别、旧定义识别与不相关布局变化拒绝。
- 真实 Ruckig：`test_follower_protocol.py`、`test_follower_control_state.py` 13 passed；`test_follower_plan.py`、`test_control_state.py` 55 passed。验证当前曲线的 `curve_duration_ns ≥ plan.duration_ns`、网格关系、非最终窗口关系和采样/标志重建。
- 真实生成类型与 CDR：`tests/integration/test_follower_protocol_ros.py::test_follower_reference_really_serializes_and_roundtrips` 1 passed，覆盖当前摘要往返、旧摘要旧标志读取成功、默认读取拒绝旧摘要、篡改旧摘要拒绝。
- 受影响 runtime 消费者：`test_contract_consistency`、`test_offline_audit`、`test_stack_export_build`、`test_stack_export_quality`、`test_mcap_feedback_guard`、`test_feedback_capability` 42 passed。
- runtime 全量单测：937 passed、28 skipped、3 failed；3 个失败均为 `test_learning_console_progress.py` 在 runtime 缺 `dual_arm_learning`（与阶段 06/07 基线一致）。
- 真实旧样本读取与数值重建通过，见“历史读取”。日志与脚本在 `.build/phase-08-20261111/`。

## 已知问题

- 没有 0.3.0 新录制；在线生产/消费路径只由单元测试和真实生成类型 CDR 往返覆盖，未启动真实 backend 发布新轨迹。
- 未跑完整旧 episode 的导出/审计；现有 bag 的反馈布局仍是 0.1.0，需按阶段 07 的 legacy 入口处理。阶段 12 负责完整历史 episode 与业务闭环。
- simulation 全量测试未运行；本阶段只改生产者字段/版本与相关共享契约，运行了受影响的 follower、control 与协议测试。
- 旧轨迹读取只覆盖一个真实样本；未批量重读仓库内全部 MCAP。
- 未 commit/push/stash/reset/clean。
