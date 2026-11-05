# 公共基础设施

只放跨实现共享的环境、依赖、构建/SDK 工具和公开部署示例，不放机器人业务、
场景参数或训练算法。

- `docker/`：公共 requirements、Fast DDS 配置和唯一可 source 的 `setup.sh`。
- `scripts/`：容器内 ROS/Policy SDK 构建、依赖镜像元数据、Dockerfile/Compose
  边界检查。
- `examples/`：公开的部署绑定草案，不是可执行 Compose。

组件专属 Dockerfile、锁文件、`docker-compose.yml` 和 `scripts/run.sh` 归各自实现
目录；宿主在各端 `docker/` 目录直接用 `docker compose` 构建和进入环境。入口与边界见各子目录 README 及
[项目规范](../docs/standards.md)。

目录导航：[上级目录](../README.md)。
