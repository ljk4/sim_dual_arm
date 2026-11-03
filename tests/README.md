# 测试

- `unit/`：纯 Python 单元测试和共享夹具，默认不依赖 ROS/GPU。
- `integration/`：ROS 服务、原生物理、记录、任务和运行入口的集成检查。
- `doubles/`：test-only fake Policy 与 mock ROS backend，不冒充正式数据。

测试按 `pyproject.toml` 和容器配置运行；ROS 测试由监督者在对应镜像中执行。
测试产物和缓存写到 `.build/`/`var/`，不提交。

目录导航：[上级目录](../README.md)。
