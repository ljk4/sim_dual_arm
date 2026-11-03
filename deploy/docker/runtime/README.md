# Runtime 镜像

`runtime` 与 `test` target 共用 dev ROS 依赖层，不 COPY 项目源码或测试。
运行时挂载仓库到 `/workspace:ro`、已生成的 ROS 工作空间到 `/opt/ros-dev:ro`。

继承的 `dev_ros.sh` 检查契约摘要并把挂载源码放在导入路径前位；SDK 缺失或过期时
拒绝启动。修改 Python/配置后重启对应进程；协议改变后先重新生成 SDK。
日常开发使用 Compose 共用 dev 依赖镜像，不再构建内嵌源码的另一套镜像。

目录导航：[上级目录](../README.md)。
