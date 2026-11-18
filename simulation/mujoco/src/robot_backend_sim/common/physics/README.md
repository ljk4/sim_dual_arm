# physics

MuJoCo 场景通用机制。

通用模块：`scene_container.py`、`robot_io.py`、`robot_adapters.py`、`robot_geometry.py`、`assets.py`、`contacts.py`、`contact_snapshot.py`、`gripper_contact.py`。

`robot_adapters.py` 按 `robot.assembly` 选择具体方案端的场景构建与 I/O 实现；本子包不包含任何机器人专属实现。

本子包不依赖 ROS。

目录导航：[上级目录](../../README.md)
