# physics

MuJoCo 场景通用机制与对称的机器人适配层。

通用模块：`scene_container.py`、`robot_io.py`、`robot_adapters.py`、`robot_geometry.py`、`assets.py`、`contacts.py`、`contact_snapshot.py`、`gripper_contact.py`。

机器人适配层：`panda/`（`scene.py` 场景构建、`io.py` 关节/夹爪映射）与 `marvin/`（`assembly.py`、`environment.py`、`scene.py`、`io.py`）。顶层不包含任何具体机器人专属实现。

本子包不依赖 ROS。

目录导航：[上级目录](../README.md)
