# 集成测试（Integration Tests）

覆盖跨模块与运行环境的端到端行为验证：

- **MuJoCo 物理与场景**：`test_sim_scene.py`、`test_panda_io.py`、`test_scene_visibility.py`、`test_scene_kinematics.py`。
- **受限跟随与 ROS 2 协议**：`test_follower_execution.py`、`test_follower_protocol_ros.py`、`test_backend_ros.py`。
- **学习端 CPU 生命周期**：`test_learning_cpu_lifecycle.py`。

集成测试须在提供对应依赖（ROS 2、MuJoCo/EGL 或 PyTorch/LeRobot）的容器环境中执行。

目录导航：[上级目录](../README.md)
