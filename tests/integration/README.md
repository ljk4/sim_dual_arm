# 集成测试

覆盖 ROS 服务/action、运动与连续块、受限跟随、MCAP 审计、运行时收尾、任务、
学习和 ACK 唤醒。`probe_backend.py`、`runtime_cli_harness.py` 等是测试辅助入口，
不是生产启动器。

需要 ROS、MuJoCo、GPU 或 Docker 的用例由对应镜像/监督者执行；纯 Python 环境会
跳过或明确失败，不把未执行当通过。

目录导航：[上级目录](../README.md)。
