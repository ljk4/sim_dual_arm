# 镜像与依赖锁

各组件 Dockerfile、入口脚本和固定依赖清单。`ros-base/` 提供 Jazzy 基础层，
`runtime/`、`backend-sim/`、`learning/` 和 `dev/` 分别构建运行、仿真、学习与开发
镜像；`entrypoints/` 是容器内 shell 入口。

`*.lock`、`*.in`、`sim-packages.json`、`learning-source.json` 和
`ruckig-source.json` 固定版本/来源，构建时不得静默升级。上游 LeRobot、Ruckig 和
MuJoCo Menagerie 的许可随各自源码保留。

目录导航：[上级目录](../README.md)。
