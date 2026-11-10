# robot_backend_sim

MuJoCo 双 Panda 物理仿真后端 Python 包，按状态所有权与职责划分为六个子域：

- **节点、工作线程与通信桥**：`backend_cli.py`、`backend_config.py`、`node.py`、`node_services.py`、`worker.py`、`worker_owner.py`、`ros_bridge.py`、`feedback_mailbox.py`、`viewer.py`、`diagnostics.py`。
- **场景构建、资产映射与相机渲染**：`scene.py`、`scene_config.py`、`assets.py`、`panda_io.py`、`rendering.py`、`render_worker.py`、`visibility.py`、`kinematics.py`、`kinematics_config.py`、`kinematics_clearance.py`。
- **运动控制、接触跟踪与制动**：`motion_controller.py`、`motion_runtime.py`、`motion_types.py`、`motion_math.py`、`motion_snapshot.py`、`motion_transitions.py`、`motion_config.py`、`gripper_contact.py`、`braking.py`、`contacts.py`。
- **控制会话与受限跟随规划**：`control.py`、`control_state.py`、`control_events.py`、`control_history.py`、`protocol.py`、`planner.py`、`follower_plan.py`、`follower_protocol.py`、`follower_config.py`、`stream_config.py`、`stream_schedule.py`、`ruckig_export.py`。
- **可插拔任务与示教插件（`tasks/`）**：
  - `base.py`、`registry.py`、`ros_conv.py`：无 ROS 依赖的通用 `SimTaskPlugin` 协议、插件注册表与延迟 ROS 转换器。
  - `stack/`：双 Panda 方块堆叠插件，包含配置（`config.py`）、物理状态机评估器（`evaluator.py`）、准入门禁（`gate.py`）、程序示教器（`teacher.py`）与 ROS 适配（`ros_conv.py`）。
- **离线检查工具（`tools/`）**：`probe_environment.py`、`check_scene.py`（及 `check_scene_common.py`、`check_scene_views.py`）、`check_kinematics.py`（及 `check_kinematics_common.py`、`check_kinematics_checks.py`、`check_kinematics_report.py`）。

目录导航：[上级目录](../README.md)
