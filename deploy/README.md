# 部署

镜像、容器组合、设备/网络/挂载绑定和公开示例；业务逻辑属于 `components/`。

开发入口统一挂载源码、配置与生成 SDK，镜像保存依赖。必要 COPY 限于依赖清单、
最小 ROS bootstrap、DDS 配置、来源信息和编译产物；Dockerfile 不内嵌程序代码。
修改应用后重启进程即可使用挂载版本，接口改变须先重新生成 SDK。

- `compose/`：mock、原生 backend/runtime、Policy/ACT、任务和一次性检查组合。
- `docker/`：ROS base、runtime、backend-sim、learning、dev 镜像、入口和锁文件。
- `examples/`：公开的环境示例与部署绑定草案，不是可执行 Compose。

`local/` 保存本机非公开绑定，已由根 `.gitignore` 排除，不提交。构建入口见
[tools/build](../tools/build/README.md)，规则见 [项目规范](../docs/standards.md)。

目录导航：[上级目录](../README.md)。
