# 单元测试

纯 Python 测试与夹具，覆盖 contracts、runtime 状态机、backend 配置/运动/跟随、
日志/记录、堆叠任务、学习数据/训练/策略和场景配置。`conftest.py` 提供共享路径；
`act_fixtures.py`、`learning_fixtures.py` 是 test-only 合成数据。

默认不启动 ROS、MuJoCo、GPU 或网络；需要外部依赖的测试应在对应环境单独运行。

目录导航：[上级目录](../README.md)。
