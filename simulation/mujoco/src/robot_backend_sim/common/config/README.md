# config

声明式配置及校验。

模块：`backend_config.py`、`camera_config.py`、`follower_config.py`、`motion_config.py`、`scene_loader.py`、`stream_config.py`。

`scene_loader.py` 通过 `common/plugins.py` 按 `robot.assembly` 动态加载对应机器人的 `scene_config` 解析器，不硬编码机器人名。

本子包不依赖 ROS。

目录导航：[上级目录](../../README.md)
