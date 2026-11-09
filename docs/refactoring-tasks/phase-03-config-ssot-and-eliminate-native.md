# Phase 3 任务书：跨端配置收敛（SSOT）与全仓彻底去 `Native*` 概念正名

## 【执行模型必读指令（System Rules & Anti-Shortcut Redlines）】
1. **前置条件**：确认 **Phase 1 & Phase 2** 已完成且测试通过。你当前只负责执行 **Phase 3**。
2. **【严禁只改文件名不改类名/消息名红线】**：
   - 此前重构只把文件名去掉了 `native_`，但里面的类名（`NativeRequest`、`NativeExecution`、`NativeMotion`、`NativeController`）和 ROS 2 消息名（`NativeCommand.msg` 等）全没改！
   - **本阶段必须将文件名、Python 类名、ROS 2 IDL 消息/服务名一次性同步正名**，且**严禁在任何文件中留下 `NativeRequest = StreamRequest` 或 re-export 空壳文件**！

---

## 1. 建立跨端共享控制配置单一数据源（`profiles/control/`）

消除 `panda_control.yaml`、`panda_follower_act_chunked.yaml`、`panda_task_stream.yaml` 在 `simulation/` 与 `runtime/` 之间的重复定义：

1. **新建 `profiles/control/` 目录并迁移公共控制配置**：
   - `simulation/mujoco/configs/control/panda_control.yaml` $\rightarrow$ `profiles/control/panda_control_v1.yaml`
   - `simulation/mujoco/configs/control/panda_follower_act_chunked.yaml` $\rightarrow$ `profiles/control/panda_follower_v1.yaml`
   - `simulation/mujoco/configs/control/panda_task_stream.yaml` $\rightarrow$ `profiles/control/panda_stream_v1.yaml`
   - 删除 `simulation/mujoco/configs/control/` 下其余冗余变体（`panda_follower.yaml`、`panda_follower_act.yaml`、`panda_stream.yaml`）及空目录。
2. **消除 `target_count` 跨文件重复定义**：
   - 将 `target_count: 10` 的唯一数据源收敛在 `profiles/control/panda_follower_v1.yaml` 中；
   - 从 Runtime YAML 配置（`native_teacher.yaml`、`native_task_act_chunked.yaml`）中移除内联的 `follower:` 块和 `policy.target_count`，改为直接引用 `follower_config: profiles/control/panda_follower_v1.yaml`。

---

## 2. 全仓彻底消除 `Native*` 命名（模块名 + 类名 + ROS IDL 一步到位）

### 2.1 ROS 2 IDL 消息与服务重命名（`contracts/ros/robot_api_msgs/`）
同步重命名以下文件并更新 `CMakeLists.txt` 及全仓引用（删除旧文件）：
| 原 IDL 文件 | 新 IDL 文件 | 说明 |
| :--- | :--- | :--- |
| `msg/NativeCommand.msg` | `msg/StreamCommand.msg` | 流式控制指令请求消息 |
| `msg/NativeExecution.msg` | `msg/ExecutionFeedback.msg` | 控制器执行状态反馈消息 |
| `msg/NativeChunkStatus.msg` | `msg/StreamChunkStatus.msg` | 流式块状态消息 |
| `msg/NativeControlEvent.msg` | `msg/ControlEvent.msg` | 控制事件消息 |
| `msg/NativeFinger.msg` | `msg/FingerFeedback.msg` | 单指接触反馈消息 |
| `msg/NativeGripper.msg` | `msg/GripperFeedback.msg` | 夹爪完成与接触反馈消息 |
| `srv/GetNativeAnchor.srv` | `srv/GetStreamAnchor.srv` | 获取流式块衔接锚点服务 |
| `srv/QueryNativeCommand.srv` | `srv/QueryStreamCommand.srv` | 查询流式控制指令状态服务 |

### 2.2 Python 模块文件重命名与消除 `_impl/` 碎片目录（使用 `git mv`，不留旧文件）
- **消除 `contracts/python/src/robot_contracts/_impl/` 隐藏目录**：
  - 将 `_impl/yaml.py`（50 行）直接合回 `validation.py`（删除 `validation.py` 末尾的 `__getattr__` 动态导入）；
  - 将 `_impl/native_reference.py` 直接合并作为 `contracts/python/src/robot_contracts/stream_request.py`（并删除尚未被生产链路使用的 `AcceptedReference`、`Trajectory` 死类，使其保持在 $\le 220$ 行）；
  - 将 `_impl/native_feedback.py` 合回 `contracts/python/src/robot_contracts/execution_feedback.py`（合回后约 310 行）；
  - **彻底删除 `contracts/python/src/robot_contracts/_impl/` 目录**！
- **禁止为凑行数新建 `*_sections.py` 或 `_impl` 碎片文件**：将 `native_config_sections.py` 合回重命名后的 `runtime_config.py`（合计 $\le 380$ 行）。

| 原文件路径 | 新文件路径 |
| :--- | :--- |
| `contracts/python/src/robot_contracts/native_reference.py` (+ `_impl/native_reference.py`) | `contracts/python/src/robot_contracts/stream_request.py` |
| `contracts/python/src/robot_contracts/native_feedback.py` (+ `_impl/native_feedback.py`) | `contracts/python/src/robot_contracts/execution_feedback.py` |
| `contracts/python/src/robot_contracts/native.py` | **直接删除**（将常量并入 `stream_request.py` 与 `execution_feedback.py`） |
| `libraries/ros_support/src/robot_ros_support/native_conversions.py` | `libraries/ros_support/src/robot_ros_support/stream_conversions.py` |
| `runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/native_config.py` (+ `native_config_sections.py`) | `runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/runtime_config.py` |
| `runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/native_node.py` | `runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/runtime_node.py` |
| `runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/native_episode.py` | `runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/episode_runner.py` |
| `runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/native_stream.py` | `runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/chunk_stream.py` |
| `runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/native_control_client.py` | `runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/robot_control_client.py` |
| `runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/native_follower_audit.py` | `runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/episode_audit.py` |
| `runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/native_cli.py` | `runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/runtime_cli.py` |
| `runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/cli.py` | `runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/control_cli.py` |
| `runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/native_errors.py` | `runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/runtime_errors.py` |
| `runtime/ros2/configs/native_teacher.yaml` | `runtime/ros2/configs/teacher_collect.yaml` |
| `runtime/ros2/configs/native_task_act_chunked.yaml` | `runtime/ros2/configs/policy_rollout.yaml` |

### 2.3 Python 核心类名同步正名（全仓替换，严禁留 `Native*` 别名！）
| 所在模块 | 原类名 | 新类名 |
| :--- | :--- | :--- |
| `robot_contracts.stream_request` | `NativeRequest` | `StreamRequest` |
| `robot_contracts.execution_feedback` | `NativeFinger` | `FingerFeedback` |
| `robot_contracts.execution_feedback` | `NativeGripper` | `GripperFeedback` |
| `robot_contracts.execution_feedback` | `NativeExecution` | `ExecutionFeedback` |
| `robot_contracts.execution_feedback` | `NativeControlEvent` | `ControlEvent` |
| `robot_contracts.execution_feedback` | `NativeChunkStatus` | `StreamChunkStatus` |
| `robot_backend_sim.motion`（Phase 4 将拆分为 `motion_controller`） | `NativeMotion` | `MotionController` |
| `robot_backend_sim.control` | `NativeController` | `ControlSession` |
| `dual_arm_runtime.runtime_config` | `NativeRuntimeConfiguration`（及 `NativeReset` 等内部类） | `RuntimeConfiguration`（及 `ResetConfig` 等） |

---

## 3. 本阶段量化验收标准（必须逐条执行并全部达标）

1. **验证旧 `native_*` 文件、`_impl/` 目录与 `Native*` 类名/消息名已 100% 清零**：
   ```bash
   find . -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
   test ! -d contracts/python/src/robot_contracts/_impl
   test ! -f runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/native_config_sections.py
   # 1. 检查 runtime 与 contracts 下是否还有 native_*.py
   find contracts/ libraries/ runtime/ -name "native*.py" | grep . && exit 1 || true

   # 2. 检查全仓是否还残留 NativeRequest / NativeExecution / NativeMotion / NativeController / NativeCommand（输出必须为空！）
   grep -rn "NativeRequest\|NativeExecution\|NativeControlEvent\|NativeChunkStatus\|NativeFinger\|NativeGripper\|NativeMotion\|NativeController\|NativeCommand\|GetNativeAnchor\|QueryNativeCommand" \
     contracts/ libraries/ simulation/ runtime/ learning/ tests/
   ```
2. **代码检查与三端测试全部通过**：
   ```bash
   ruff check .
   python3 infrastructure/scripts/check_dockerfiles.py
   ./simulation/mujoco/scripts/run.sh test
   ./runtime/ros2/scripts/run.sh test
   ./learning/lerobot/scripts/run.sh test
   ```
