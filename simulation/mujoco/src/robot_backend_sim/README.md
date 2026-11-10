# robot_backend_sim

MuJoCo 后端 Python 包。按状态所有权分组：

- `node.py`、`worker.py`、`worker_owner.py`、`ros_bridge.py`、`feedback_mailbox.py`：
  ROS 节点、物理 worker 所有权、服务桥和反馈邮箱。
- `scene.py`、`scene_config.py`、`assets.py`、`panda_io.py`、`rendering.py`、
  `render_worker.py`、`visibility.py`：场景组装、Panda 映射、渲染和视野检查。
- `motion_controller.py`、`motion_runtime.py`、`motion_types.py`、`motion_math.py`、
  `motion_snapshot.py`、`motion_transitions.py`、`gripper_contact.py`、
  `motion_config.py`、`braking.py`、`contacts.py`、`control.py`、
  `control_state.py`：运动、接触完成、刹车和控制状态。
- `follower_plan.py`、`follower_protocol.py`、`follower_config.py`、
  `stream_schedule.py`：受限跟随、连续流调度和预编译。
- `tasks/`：通用 `SimTaskPlugin` 协议与 `tasks/stack/` 堆叠插件（判定、teacher、
  ROS 适配、配置）。
- `tools/check_scene*.py`、`tools/check_kinematics*.py`、`tools/probe_environment.py`、
  `diagnostics.py`、`ruckig_export.py`：一次性检查、诊断和 Ruckig 导出；检查工具
  按公共失败类型、视图/布局检查和报告发布拆分为小型模块。

入口 `backend_cli.py` 解析配置并启动 node；所有长期状态由组合根显式创建。

目录导航：[上级目录](../README.md)。
