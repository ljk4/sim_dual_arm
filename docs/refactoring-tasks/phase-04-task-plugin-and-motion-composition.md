# Phase 4 任务书：任务插件解耦、运动控制器显式组合拆分与仿真后端全面瘦身

## 【执行模型必读指令（System Rules & Anti-Shortcut Redlines）】
1. **前置条件**：确认 **Phase 1 ~ Phase 3** 已完成且测试通过。你当前只负责执行 **Phase 4**。
2. **【严禁使用 `*Mixin` 假拆分 `motion.py` 红线】**：
   - **绝对禁止**用 `BrakingMixin` / `GripperContactMixin` 多重继承的方式把 `MotionController` 的方法切到不同文件但依然读写宿主 `self._*` 的 30 个私有属性！
   - 必须使用**显式组件组合（Composition）**：`MotionController` 内部持有独立的 `self._braking = BrakingController(...)` 和 `self._gripper = GripperContactTracker(...)` 实例，通过明确的方法参数与返回值通信。
   - **严禁保留 `motion.py` 兼容空壳文件**！
3. **【严禁在 `tasks/stack/__init__.py` 顶层导入 `robot_api_msgs` 红线】**：
   - 仿真核心包 `robot_backend_sim` 必须在无 ROS 2（无 `robot_api_msgs`）的纯 MuJoCo 环境下可正常导入并运行算法单测！`ros_conv.py` 中对 `robot_api_msgs` 的导入必须放在消息转换函数内部按需延迟执行。

---

## 1. 泛化 ROS 2 任务接口并抽离仿真 `SimTaskPlugin` 插件架构

### 1.1 泛化 ROS 2 任务消息与服务（`contracts/ros/robot_api_msgs/`）
将专属 `Stack` 的消息与服务替换为与具体任务无关的通用契约（同步更新 `CMakeLists.txt` 并删除旧文件）：
- `msg/StackTaskState.msg` $\rightarrow$ `msg/TaskState.msg`（用 `string task_type`、`string phase`、`bool succeeded`、`bool terminal`、`string failure_reason`、`string metrics_json` 承载任务特定指标）
- `srv/PrepareStackTask.srv` $\rightarrow$ `srv/PrepareTask.srv`
- `srv/StartStackTask.srv` $\rightarrow$ `srv/StartTask.srv`
- `srv/GetStackTaskState.srv` $\rightarrow$ `srv/GetTaskState.srv`
- 新建 `contracts/python/src/robot_contracts/task_state.py`（继承 `StrictContractModel`，**目标 $\le 120$ 行**）。

### 1.2 将 `stack_*` 移入 `simulation/mujoco/src/robot_backend_sim/tasks/stack/` 插件包
1. 新建 `simulation/mujoco/src/robot_backend_sim/tasks/base.py`（定义 `SimTaskPlugin` Protocol 与通用快照类型）与 `registry.py`（`create_task_plugin(task_type, ...)`）。
2. 将原 `task_config.py`、`stack_task.py`、`teacher.py`、`stack_ros.py` 移入 `tasks/stack/` 子目录：
   - `tasks/stack/config.py`（$\le 200$ 行）
   - `tasks/stack/evaluator.py`（将原 961 行的 `stack_task.py` 精简至 **$\le 680$ 行**）
   - `tasks/stack/teacher.py`（将原 812 行的 `teacher.py` 精简至 **$\le 680$ 行**）
   - `tasks/stack/ros_conv.py`（函数内延迟导入 `robot_api_msgs`）
   - `tasks/stack/__init__.py`（实现 `StackTaskPlugin`，**目标 $\le 550$ 行**）
3. 重构 `worker.py`、`worker_owner.py`、`node.py` 与 Runtime 端 `stack_task_client.py`（重命名为 `task_client.py`），使宿主层只依赖通用 `SimTaskPlugin` 与 `TaskState`，彻底消除 `stack` 硬编码。
4. 将 Runtime 脚本去 `stack_` 前缀正名并消除 `STACK_*` 环境变量硬编码：
   - `stack_campaign.py` $\rightarrow$ `campaign.py`（`STACK_SEED` / `STACK_ATTEMPT_DIR` 改为 `CAMPAIGN_SEED` / `CAMPAIGN_ATTEMPT_DIR`）
   - `collect_stack_attempt.py` $\rightarrow$ `collect_attempt.py`
   - `stack_evaluate.py` $\rightarrow$ `evaluate.py`
   - `stack_export.py` $\rightarrow$ `export_episode.py`

---

## 2. 将 `motion.py` 重构为显式组合的 4 个高内聚模块（彻底删除 `motion.py` 与 `contact_unload.py`）

将原 `motion.py`（Phase 1 清理后约 1,500 行）+ `braking.py` + `contact_unload.py` 重构为以下 4 个职责单一、无 Mixin 继承的模块：

1. **[`motion_types.py`](../../simulation/mujoco/src/robot_backend_sim/motion_types.py)**（**目标 $\le 150$ 行**）：
   - 定义 `Phase`、`StopReason`、`FailedReason`、`MotionSnapshot`、`MotionResult`。
2. **[`motion_math.py`](../../simulation/mujoco/src/robot_backend_sim/motion_math.py)**（**目标 $\le 420$ 行**）：
   - 纯函数形式承载控制器初始化参数校验、锚点计算（`compute_idle_chunk_anchor` / `compute_active_chunk_anchor`）、流式首块对齐校验、跟踪误差计算与 `build_motion_snapshot()`。
3. **[`gripper_contact.py`](../../simulation/mujoco/src/robot_backend_sim/gripper_contact.py)**（合并原 `contact_unload.py` 并删除 `contact_unload.py`，**目标 $\le 320$ 行**）：
   - 实现独立组件类 `GripperContactTracker`（严禁写 `GripperContactMixin`！），封装接触力测量、双指完成判定、受限轴计算与张开卸载（`ContactUnloader`）。
4. **[`braking.py`](../../simulation/mujoco/src/robot_backend_sim/braking.py)**（**目标 $\le 480$ 行**）：
   - 实现独立组件类 `BrakingController`（严禁写 `BrakingMixin`！），封装 Ruckig 制动规划（`BrakingPlan`）与静止/稳定窗口判定逻辑。
5. **[`motion_controller.py`](../../simulation/mujoco/src/robot_backend_sim/motion_controller.py)**（**硬性红线：必须 $\le 550$ 行**）：
   - 实现 `MotionController`（原 `NativeMotion`），通过组合 `self._schedule: StreamSchedule`、`self._braking: BrakingController`、`self._gripper: GripperContactTracker` 驱动 500Hz 闭环。

---

## 3. 移动离线诊断工具、修复基线残留细节并瘦身仿真后端其余超标模块（全部压至 $\le 700$ 行）

1. **修复两处 Phase 1/2 迁移遗留细节与合并 `*_config_sections.py`**：
   - 在重构 `motion.py` $\rightarrow$ `motion_controller.py` 时，修复原 `motion.py:1350` 的 `if self._stream is not None and self._stream.kind == "FOLLOWER":`（因为现在所有流式块均为 `PreparedFollowerPlan`，`StreamSchedule` 已无 `.kind` 属性，直接改为 `if self._stream is not None:`）。
   - 在 [`backend_cli.py`](../../simulation/mujoco/src/robot_backend_sim/backend_cli.py) 第 130 行中，将对 Pydantic 模型 `BackendConfig` 调用 `asdict(config)` 改为 `config.canonical_dict()`（并删除 `from dataclasses import asdict`）。
   - 将 Phase 2 拆出的 `backend_config_sections.py`、`kinematics_config_sections.py`、`scene_config_sections.py`、`task_config_sections.py` 分别合回对应的 `backend_config.py`、`kinematics_config.py`、`scene_config.py`、`tasks/stack/config.py`，消除零碎的 `*_sections.py` 伴生文件。
2. **将 3 个离线检查脚本移入 `simulation/mujoco/src/robot_backend_sim/tools/` 子目录**：
   - `check_kinematics.py`、`check_scene.py`、`probe_environment.py` 移至 `tools/`（同步更新 `backend_cli.py` 与测试导入）。
2. **瘦身 `control.py` 与 `control_state.py`（两文件合计必须 $\le 920$ 行）**：
   - `control_state.py`（**目标 $\le 400$ 行**）：直接引用不可变契约对象，删除重复字段平铺拷贝与非流式死代码。
   - `control.py`（**目标 $\le 520$ 行**）：合并重复的租约/上下文检查，精简 `ControlSession` 方法层级。
3. **瘦身 `node.py`、`worker.py`、`worker_owner.py`、`kinematics.py`（每个文件均必须 $\le 680$ 行）**：
   - 在 `node.py` 中提取统一的 ROS Service 异常捕获包装器，消除 10+ 个回调中的重复样板；
   - 在 `worker.py` / `worker_owner.py` / `kinematics.py` 中清理冗余包装与重复的手写类型防御。

---

## 4. 本阶段量化验收标准（必须逐条执行并全部达标）

1. **反作弊与无 ROS 独立导入检查**：
   ```bash
   find . -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
   test ! -f simulation/mujoco/src/robot_backend_sim/motion.py
   test ! -f simulation/mujoco/src/robot_backend_sim/contact_unload.py
   find simulation/mujoco/src/robot_backend_sim -name "*_sections.py" | grep . && exit 1 || true
   # 1. 严禁存在任何 Mixin 类（输出必须为空）
   grep -rn "class .*Mixin" simulation/mujoco/src/robot_backend_sim/
   # 2. 验证 motion_controller.py <= 550 行，control.py + control_state.py <= 920 行
   wc -l \
     simulation/mujoco/src/robot_backend_sim/motion_controller.py \
     simulation/mujoco/src/robot_backend_sim/control.py \
     simulation/mujoco/src/robot_backend_sim/control_state.py
   ```
2. **仿真包单文件 720 行红线检查**：
   ```bash
   python3 -c '
   import pathlib, sys
   violations = [
       (len(p.read_text().splitlines()), str(p))
       for p in pathlib.Path("simulation/mujoco/src").rglob("*.py")
       if "/tools/" not in str(p) and len(p.read_text().splitlines()) > 720
   ]
   for l, p in sorted(violations, reverse=True): print(f"VIOLATION: {l} {p}")
   sys.exit(1 if violations else 0)
   '
   ```
3. **代码检查与三端测试全部通过**：
   ```bash
   ruff check .
   ./simulation/mujoco/scripts/run.sh test
   ./runtime/ros2/scripts/run.sh test
   ```
