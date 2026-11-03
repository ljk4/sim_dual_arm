# 容器入口

`ros.sh` 为 ROS 运行容器加载环境并执行传入命令；`dev_ros.sh` 供开发容器使用。
入口只做环境和权限准备，不启动业务生命周期。

`policy.sh` 调用 `tools/build/policy_sdk.py check` 校验挂载 SDK，设置其导入优先级，
随后 exec 实际进程。检查实现留在工具中；入口不内嵌业务或构建程序。

目录导航：[上级目录](../README.md)。
