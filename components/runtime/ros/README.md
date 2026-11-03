# ROS 包根

ROS 2 工作空间的源码根，目前只有 `dual_arm_runtime` 一个运行包。构建、生成契约和
安装由 `tools/build/` 与 `deploy/docker/ros-base/` 负责；生成文件写到 `.build/`，
不回到源码树。包说明见 [dual_arm_runtime](dual_arm_runtime/README.md)。

目录导航：[上级目录](../README.md)。
