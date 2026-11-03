# ROS 基础镜像

`Dockerfile` 以 `ros:jazzy-ros-base-noble` 为基础，安装 ROS/colcon/MCAP 等系统
依赖及 `../requirements-dev.txt` 的 Python 依赖，供 runtime、backend-sim 和 dev
镜像复用。`sim-packages.json` 和仿真锁文件属于仿真依赖层，不是 ROS base 的锁文件。

目录导航：[上级目录](../README.md)。
