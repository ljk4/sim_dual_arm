# 阶段 07 交接：末端反馈容器解耦

- 日期：2026-11-11（Asia/Shanghai）。状态：完成（代码与验证）；未进入阶段 08。
- 依据：总要求、阶段 07、progress、阶段 06 交接。未改业务数值、Panda 模型/物理、旧 MCAP、prepared、模型、正式台账或 Policy API。

## 类型边界

- `ExecutionFeedback.grippers`：0～8 个命名夹爪（ROS `GripperFeedback[<=8]`），允许空集合。
- `GripperFeedback.fingers`：1～8 个命名手指（ROS `FingerFeedback[<=8]`），不再固定两个。
- `side` 字段名保留，值为 1～128 字符的非空标识；同一层唯一，不要求 left/right、不要求顺序。
- `finger_index`：正整数 1～8，层内唯一；不要求连续、不要求 1、2 或升序。`uint8` wire 类型未变。
- 原有校验保留：对象名唯一、对象/力/计数等长、非有限数值拒绝、模式与 `position_ok/contact_ok/contact_object/ambiguous` 关系、`target_error` 关系、关节向量长度与误差关系。
- Panda 规则仍在实现侧：`contacts.validated_panda_completions` 要求恰好 left、right 两夹爪、每夹爪手指 1、2 且 side 对应；由 `GripperContactTracker.completions()`/`hold_completions()` 调用。同物体接触与完成门禁仍由 `evaluate_gripper_completion` 与现有完成逻辑检查，输出顺序不变。

## 协议与能力版本

- `robot_api_msgs` 由 0.1.0 升到 0.2.0；新的能力标识 `execution_feedback.gripper_set.v1`。
- simulation `BackendInfo.capabilities` 发布该标识；runtime `REQUIRED_CAPABILITIES` 与 prepare 快照要求它；`StreamAudit.required_capabilities` 也要求它。缺该能力的 backend/记录在消费者侧明确失败（`UNSUPPORTED_CAPABILITY` / `recorded backend lacks required capabilities`），不是只改发布端版本。
- 未改 Policy proto、Policy SDK 或阶段 08 轨迹协议。

## 入口与调用方

- 契约：`shared/common/python/contracts/src/robot_contracts/execution_feedback.py` 新常量 `MAX_GRIPPER_FEEDBACK=8`、`MAX_GRIPPER_FINGERS=8`、`MAX_FINGER_INDEX=8`。
- IDL：`shared/common/interfaces/ros/robot_api_msgs/msg/{ExecutionFeedback,GripperFeedback}.msg`。
- 生产者：`simulation/.../runtime/protocol.py` 移除固定两条检查，只复制冻结快照；Panda 检查前移到 `physics/contacts.py`。
- 消费者：`runtime_identity`、`audit_stream`、`mcap_reader`、`export_episode` 均按标识遍历，无固定 [0]/[1] 或左右顺序假设；Panda 专用日志仍是方案实现。
- ROS 转换 `stream_conversions.py` 的结构化 list/tuple 逻辑无需改分支，已用真实生成类型验证。

## SDK

```bash
# runtime 容器
python3 /shared/common/tools/scripts/dev_workspace.py build --project-root /workspace \
  --shared-root /shared --build-root /build/ros-core
PYTHONPATH=/shared/common/python/contracts/src:$PYTHONPATH \
python3 /shared/common/tools/scripts/dev_workspace.py build --project-root /workspace \
  --shared-root /shared --scenario panda_stack --build-root /build/ros-dev
```

- 生成文件未手改。核心：`robot_api_msgs`，`82b77139…`；完整：3 包，`39fc5baa…`；两次 `check` 均通过。生成类型显示 `sequence<GripperFeedback, 8>` / `sequence<FingerFeedback, 8>`。

## 历史读取

- 0.1.0 旧类型是固定 `[2]` 数组，CDR 无长度前缀；当前生成类型不能安全反序列化。布局识别集中在 runtime 模块 `offline/feedback_definition.py`：把录制内嵌定义与安装定义、已知旧定义分别规范化后直接比较。
- 普通导出 `export_episode._read_metadata` 与审计 `mcap_reader._run_audit` 都在 `get_message`/`deserialize_message` 之前调用 `require_current_feedback_definition`。已安装布局放行；完整旧布局明确拒绝并指向 `python -m dual_arm_runtime.offline.legacy_feedback`；其他未识别定义也明确拒绝。两种情况都不会把旧 payload 交给当前生成类型。
- `legacy_feedback._verify_legacy_definition` 不再只检查两个数组声明，而是比较完整已知布局：字段类型、顺序、`[2]`/`[<=64]`/`[<=8]` 边界及三个嵌套消息定义。注释、空行和 `=` 分段线在比较前统一去掉；旧样本仍输出原定义文本的 SHA-256 `aa0c67c6…`。
- 导出、审计和 legacy 读取均在异常路径 `close()` 本次 reader。定义拒绝发生在任何解码与业务输出之前。
- 真实样本：`var/campaigns/panda_stack_v1/attempts/formal-10014-record-1794304035694470020-worker-03/.../recording`，topic `/record_pool_1794303940600369712_3/execution_feedback`。重跑 legacy 读取得到与 `.build/phase-07-20261111/legacy-sample.json` 完全相同的关节数与夹爪值；同一 bag 经导出与审计入口均以 `RECORDING_ERROR` 拒绝。
- 完整旧 episode 导出/审计仍留阶段 12；本次只验证一个真实反馈样本，不能标为完整历史兼容通过。

## 验证结果

- 读取收尾 ruff：本次改动文件全部通过。
- 读取收尾定向：`test_legacy_feedback.py`、`test_mcap_feedback_guard.py` 13 passed。覆盖完整旧布局在带注释/分隔线时接受、仅有两个数组声明时拒绝、单字段类型或顺序或数组边界变化时拒绝、导出入口在反序列化前拒绝旧布局、当前布局正常读取。
- 相关消费者 `test_offline_audit`、`test_stack_export_build`、`test_stack_export_quality`、`test_feedback_capability`、`test_contract_consistency` 与上述两文件合计 44 passed。
- 真实旧样本 legacy 读取输出与既有 `legacy-sample.json` 逐字段一致；同一 bag 在导出与审计入口均于解码前被 `RECORDING_ERROR` 拒绝。
- 阶段 07 主体 ruff：改动文件全部通过。
- 阶段 07 runtime 定向：66 passed（契约/一致性/能力/legacy/audit/gripper completion）；相关 prepare/record/worker/campaign/dev_workspace：76 passed。
- 阶段 07 runtime 全量单测：925 passed、25 skipped、3 failed；3 个失败均为 `test_learning_console_progress.py` 在 runtime 缺 `dual_arm_learning`，同文件 learning 容器 7 passed（与阶段 06 基线一致）。
- 阶段 07 真实生成类型与 CDR：`tests/integration/test_feedback_contract_ros.py` 8 passed，覆盖 0/1/2/3/8 夹爪、非 left/right、非连续手指、重复 side 拒绝、超界拒绝。
- 阶段 07 simulation 定向：90 passed（gripper completion/contact unload/control/follower state/task execution/shutdown/braking）。Panda 接触/首窗回归 2 passed；真实 backend 能力发布 1 passed。
- 真实 `record_preflight` exit 0。证据目录 `.build/phase-07-20261111/`（pytest 日志、marker 副本、legacy 样本、本次 `legacy-sample-rerun.json` 与拒绝日志）。

## 未验证与下一阶段

- 没有 0.2.0 新录制：导出与审计读取当前布局只由单元测试覆盖，未在真实当前格式 bag 上运行。
- 未跑完整旧 episode 审计/导出（阶段 12）；legacy 读取只覆盖一个真实样本。
- 未启动真实 Teacher 后端 `get_task_chunk` 端到端调用；阶段 06 的该项仍未验证，不能把类型断言写成调用通过。
- 未运行完整 simulation/learning 测试集；本轮只改反馈读取路径，learning 无改动路径。未 commit/push/stash/reset/clean。
- runtime 容器中先 `source /opt/ros/jazzy/setup.bash` 再运行 pytest 会加载 `launch_testing.pytest.hooks`，普通单测被跳过（`found no collectors`）。单元测试在容器默认环境运行，需要 ROS 的脚本单独 source。
- 未启动真实 Teacher 后端 `get_task_chunk` 端到端调用；阶段 06 的该项仍未验证，不能把类型断言写成调用通过。
- 未运行完整 simulation/learning 测试集；本轮只改反馈契约与对应消费者，learning 无改动路径。未 commit/push/stash/reset/clean。
