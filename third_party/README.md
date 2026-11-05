# 外部源码

Panda 模型现直接位于 [仿真资产目录](../simulation/mujoco/assets/README.md)，
随仿真项目挂入容器。本目录保留其他外部源码。随源码提供的固定 Ruckig
依赖包位于 [simulation/mujoco/docker/wheels](../simulation/mujoco/docker/wheels/README.md)，
保留来源、SHA256、平台说明和许可证，用于无需编译的镜像安装。

- `src/lerobot-act-v0.6.1/`：学习环境使用的固定 LeRobot checkout，来源见
  `learning/lerobot/docker/learning-source.json`。

`src/` 已由根 `.gitignore` 排除；上游 README、LICENSE 和原始文件保持不动。

目录导航：[上级目录](../README.md)。
