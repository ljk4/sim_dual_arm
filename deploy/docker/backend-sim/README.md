# backend-sim 镜像

`Dockerfile` 提供三个主要运行 target：

- `mock`：CPU mock 依赖镜像，源码和生成的 ROS 工作空间通过只读挂载提供。
- `mujoco-dev`：固定 MuJoCo/mink/EGL 依赖，项目源码、SDK、资产和配置运行时挂载。
- `follower-dev`：在 `mujoco-dev` 上安装固定 Ruckig wheel，供连续受限跟随使用。

三个 target 都不 COPY 业务源码。Ruckig wheel 及其来源记录属于必要的构建产物复制。
依赖元数据由 `tools/build/image_metadata.py` 生成，构建时通过 BuildKit 只读挂载
调用；Dockerfile 不内嵌 Python 程序。运行时的物理/业务实现仍来自仓库挂载。

目录导航：[上级目录](../README.md)。
