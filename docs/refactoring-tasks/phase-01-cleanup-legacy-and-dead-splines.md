# Phase 1 任务书：彻底清除 S1 Mock 链路、S2/S3 非受限样条接口与仿真 SciPy 样条死代码

## 【执行模型必读指令（System Rules & Anti-Shortcut Redlines）】
1. **阶段边界**：你当前只负责执行 **Phase 1**。不要跨阶段提前执行后续阶段的任务，也不要自行发散修改本文件未列出的核心算法逻辑。
2. **【严禁留空壳与残留死分支红线】**：
   - 删除废弃模块时，**必须物理删除文件并同步更新所有导入方与测试文件**，严禁保留空壳兼容文件；
   - **必须连根拔除仿真端残留的旧版 SciPy 非受限样条规划链路**（`chunk_plan.py`、`reference_compiler.py`、`protocol.py:prepare_request`、`motion.py:start`、`_step_legacy`），严禁因为某些旧单测还在调用它们就保留这些生产环境已 100% 废弃的死代码！
3. **【严禁依赖残留 `.pyc` 字节码红线】**：
   - 每次删除 `.py` 文件后，必须同步清理对应的 `__pycache__/*.pyc`，确保不存在“源文件已删但靠旧 `.pyc` 导包通过”的假象。

---

## 1. 本阶段核心目标
当前仓库的唯一真实生产闭环是 **MuJoCo 双臂仿真后端 + ROS 2 流式有界跟随控制（Ruckig `SubmitFollower`） + LeRobot ACT 策略服务**。
本阶段将历史遗留的 **S1 Mock 假后端链路**、**S2/S3 单次非受限样条轨迹链路（`SubmitNative` / `ExecuteTrajectory` / SciPy `chunk_plan`）**、**Fake/Replay 假策略服务配置** 以及 **v2/v3/v4 过渡期 JSON Schema** 彻底清零。

---

## 2. 必须物理删除的文件与目录清单

### 2.1 删除 S1 Mock 后端、通用假引擎与旧运行时模块
1. 删除整个 `libraries/backend_core/` 目录：
   - `libraries/backend_core/pyproject.toml`
   - `libraries/backend_core/README.md`
   - `libraries/backend_core/src/README.md`
   - `libraries/backend_core/src/robot_backend_core/__init__.py`
   - `libraries/backend_core/src/robot_backend_core/engine.py`
   - `libraries/backend_core/src/robot_backend_core/README.md`
2. 删除整個 `tests/doubles/ros/mock_robot_backend/` 目录（连同 `tests/doubles` 空目录一并删除）。
3. 删除 `runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/` 下的 S1 旧运行时模块：
   - `audit.py`
   - `configuration.py`
   - `control_client.py`
   - `episode.py`
   - `node.py`
   - `native_audit.py`（v2/v3 旧版审计器，当前 v5 已使用 `native_follower_audit.py`）
4. 删除 S1 与过渡期废弃配置文件：
   - `infrastructure/examples/sim_collect.yaml`（连同空的 `infrastructure/examples/` 目录）
   - `runtime/ros2/configs/mock_collect.yaml`
   - `runtime/ros2/configs/native_collect.yaml`
   - `runtime/ros2/configs/native_policy.yaml`
   - `runtime/ros2/configs/native_replay.yaml`
   - `runtime/ros2/configs/native_task_act.yaml`
   - `runtime/ros2/configs/native_task_act_fast.yaml`
   - `learning/lerobot/configs/policy.yaml`
   - `learning/lerobot/configs/replay_policy.yaml`
   - `learning/lerobot/configs/act_policy.yaml`
   - `profiles/actions/mock_joint_position_v1.json`

### 2.2 删除 S2/S3 废弃 ROS 2 消息、服务与 Action（`contracts/ros/robot_api_msgs/`）
1. 删除 Action 目录及文件：
   - `contracts/ros/robot_api_msgs/action/ExecuteTrajectory.action`
   - `contracts/ros/robot_api_msgs/action/README.md`
2. 删除废弃 Msg 文件：
   - `contracts/ros/robot_api_msgs/msg/AcceptedReference.msg`
   - `contracts/ros/robot_api_msgs/msg/CommandEvent.msg`
   - `contracts/ros/robot_api_msgs/msg/CommandStatus.msg`
   - `contracts/ros/robot_api_msgs/msg/TrajectoryCommand.msg`
3. 删除废弃 Srv 文件：
   - `contracts/ros/robot_api_msgs/srv/QueryCommand.srv`
   - `contracts/ros/robot_api_msgs/srv/QueryNativeChunk.srv`
   - `contracts/ros/robot_api_msgs/srv/SubmitNative.srv`

### 2.3 删除仿真端残留的旧版 SciPy 非受限样条规划死代码
生产环境已 100% 采用 Ruckig 有界跟随规划（`follower_plan.py`），以下旧版基于 SciPy `CubicSpline` 的非受限样条模块必须直接删除：
1. 删除 `simulation/mujoco/src/robot_backend_sim/chunk_plan.py`（272 行）
2. 删除 `simulation/mujoco/src/robot_backend_sim/reference_compiler.py`（143 行）
3. 删除 `simulation/mujoco/src/robot_backend_sim/chunk_types.py`（50 行，因为删掉 `PreparedChunk` 后流式块只有 `PreparedFollowerPlan` 一种类型，直接用 `PreparedFollowerPlan` 即可）

### 2.4 删除废弃契约模块与过渡期 Schema（保留 v1，并将 v5 重编号归位至 v1）
1. 删除 `contracts/python/src/robot_contracts/status.py`。
2. 删除 `contracts/schemas/v2/`、`contracts/schemas/v3/`、`contracts/schemas/v4/` 整个目录。
3. 将 `contracts/schemas/v5/episode_manifest.schema.json` 覆盖合并至 `contracts/schemas/v1/episode_manifest.schema.json`（并将内部 `"const": 5` 统一改为 `"const": 1`），随后删除 `contracts/schemas/v5/` 目录。

### 2.5 删除专门针对上述已废除代码的旧单元测试与集成测试
直接删除以下测试文件：
- `tests/unit/test_backend_engine.py`
- `tests/unit/test_command_status.py`
- `tests/unit/test_configuration.py`
- `tests/unit/test_control_contracts.py`
- `tests/unit/test_fake_replay_provider.py`
- `tests/unit/test_native_reference_record.py`
- `tests/integration/test_mock_backend_ros.py`
- `tests/integration/test_native_backend_control.py`
- `tests/integration/test_native_execution_reordering.py`
- `tests/integration/test_native_motion.py`
- `tests/integration/test_native_physical_chunks.py`
- `tests/integration/test_native_protocol_physics.py`
- `tests/integration/test_native_protocol_ros.py`
- `tests/integration/test_native_runtime_ros.py`
- `tests/integration/test_native_v3_audit.py`
- `tests/integration/test_reference_compiler_native.py`
- `tests/integration/test_recording.py`
- `tests/integration/test_recording_backpressure.py`
- `tests/integration/test_runtime_cleanup.py`
- `tests/integration/test_runtime_cli.py`
- `tests/integration/test_runtime_logging.py`
- `tests/integration/test_runtime_shutdown.py`

---

## 3. 存留模块的同步清理与改造要求

1. **[`simulation/mujoco/src/robot_backend_sim/follower_plan.py`](../../simulation/mujoco/src/robot_backend_sim/follower_plan.py)**：
   - 移除对 `chunk_plan.prepare_chunk` 的冗余调用！直接在 `prepare_follower_plan()` 中校验 `ActionWindow`、`ChunkAnchor`、`RobotProfile`、`RobotControlConfig` 与 `FollowerConfig`，直接调用一次 Ruckig 生成 `PreparedFollowerPlan`（不再先白跑一次 SciPy 三次样条求解）。
2. **[`simulation/mujoco/src/robot_backend_sim/motion.py`](../../simulation/mujoco/src/robot_backend_sim/motion.py)**：
   - **彻底删除 `NativeMotion.start(prepared: PreparedMotion)`、`_step_legacy()`** 以及所有单次非流式 `_active_plan` 分支，只保留流式块调度（`submit_stream_chunk` / `_step_stream` / `stop`）。
   - 将原 `PreparedStreamChunk` 类型注解全部直接替换为 `PreparedFollowerPlan`。
3. **[`simulation/mujoco/src/robot_backend_sim/protocol.py`](../../simulation/mujoco/src/robot_backend_sim/protocol.py)、[`control.py`](../../simulation/mujoco/src/robot_backend_sim/control.py)、[`control_state.py`](../../simulation/mujoco/src/robot_backend_sim/control_state.py)、[`worker.py`](../../simulation/mujoco/src/robot_backend_sim/worker.py)、[`worker_owner.py`](../../simulation/mujoco/src/robot_backend_sim/worker_owner.py)、[`node.py`](../../simulation/mujoco/src/robot_backend_sim/node.py)**：
   - 删除 `protocol.py` 中的 `PreparedControlCommand`、`PreparedMotion`、`prepare_request()`（仅保留 `validate_run_context` 等仍被 `follower_protocol.py` 调用的校验函数，或将其并入 `follower_protocol.py`）。
   - 删除 `control.py` / `worker.py` / `node.py` 中的 `submit_request`、`SubmitNative`、`QueryNativeChunk` 服务处理函数及相关队列分支，仅保留 `SubmitFollower`、`GetNativeAnchor`、`QueryNativeCommand`、`Stop`。
4. **[`contracts/ros/robot_api_msgs/CMakeLists.txt`](../../contracts/ros/robot_api_msgs/CMakeLists.txt) 与 [`libraries/ros_support/`](../../libraries/ros_support/src/robot_ros_support/)**：
   - 从 `CMakeLists.txt` 中移除已删的 `.msg` / `.srv` / `.action`；
   - 清理 `conversions.py` 和 `native_conversions.py` 中转换已删除消息的函数。
5. **Runtime 端与 Docker 构建脚本同步**：
   - 将全仓 `episode_manifest` 的 `schema_version` 常量统一更新为 `1`；
   - 更新 `runtime/ros2/ros/dual_arm_runtime/setup.py`，移除指向已删模块的 `console_scripts`；
   - 更新 `simulation/mujoco/docker/Dockerfile`、`runtime/ros2/docker/Dockerfile`、`infrastructure/scripts/dev_workspace.py`、`infrastructure/scripts/check_dockerfiles.py` 及 `tests/unit/test_container_build.py`，移除对 `libraries/backend_core` 和 `tests/doubles/ros/mock_robot_backend` 的挂载与安装。
   - 更新存留的单元测试（如 `test_follower_plan.py`、`test_action_windows.py`、`test_native_contracts.py`），移除对 `chunk_plan.prepare_chunk` 和 `NativeMotion.start` 的调用，统一使用 `prepare_follower_plan` 与 `submit_stream_chunk`。

---

## 4. 本阶段量化验收标准（必须逐条执行并全部达标）

1. **清理 `__pycache__` 并验证废弃文件已 100% 物理删除**：
   ```bash
   find . -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
   test ! -d libraries/backend_core
   test ! -d tests/doubles
   test ! -d contracts/schemas/v2
   test ! -d contracts/schemas/v5
   test ! -f simulation/mujoco/src/robot_backend_sim/chunk_plan.py
   test ! -f simulation/mujoco/src/robot_backend_sim/reference_compiler.py
   test ! -f simulation/mujoco/src/robot_backend_sim/chunk_types.py
   test ! -f contracts/ros/robot_api_msgs/srv/SubmitNative.srv
   ```
2. **反作弊检查（确认 `_step_legacy` 与 `prepare_chunk` 已从全仓彻底消失）**：
   ```bash
   # 输出必须为空
   grep -rn "_step_legacy\|prepare_chunk\|compile_reference_plan\|SubmitNative" \
     contracts/ libraries/ simulation/ runtime/ learning/ tests/
   ```
3. **代码检查与三端测试全部通过**：
   ```bash
   ruff check .
   python3 infrastructure/scripts/check_dockerfiles.py
   ./simulation/mujoco/scripts/run.sh test
   ./runtime/ros2/scripts/run.sh test
   ./learning/lerobot/scripts/run.sh test
   ```
