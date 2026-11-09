# robot_backend_sim

MuJoCo 后端 Python 包。按状态所有权分组：

- `node.py`、`worker.py`、`worker_owner.py`、`ros_bridge.py`、`feedback_mailbox.py`：
  ROS 节点、物理 worker 所有权、服务桥和反馈邮箱。
- `scene.py`、`scene_config.py`、`assets.py`、`panda_io.py`、`rendering.py`、
  `render_worker.py`、`visibility.py`：场景组装、Panda 映射、渲染和视野检查。
- `motion.py`、`motion_config.py`、`braking.py`、`contacts.py`、`control.py`、
  `control_state.py`：运动、接触完成、刹车和控制状态。
- `follower_plan.py`、`follower_protocol.py`、`follower_config.py`、
  `stream_schedule.py`：受限跟随、连续流调度和预编译。
- `stack_task.py`、`stack_ros.py`、`teacher.py`、`task_config.py`、
  `contact_unload.py`：堆叠判定、ROS 任务接口、teacher 和接触卸载。
- `check_scene.py`、`check_kinematics.py`、`probe_environment.py`、`diagnostics.py`、
  `ruckig_export.py`：一次性检查、诊断和 Ruckig 导出。

入口 `backend_cli.py` 解析配置并启动 node；所有长期状态由组合根显式创建。

目录导航：[上级目录](../README.md)。
