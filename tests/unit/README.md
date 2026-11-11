# 单元测试（Unit Tests）

覆盖 `robot_contracts`、轨迹数学（三次样条与受限跟随）、`runtime` 状态机与审计规则、`simulation` 配置/运动/接触/制动、堆叠任务插件以及 `learning` 数据集/筛选/训练/时间融合的纯 Python 单元测试。

- `test_contract_consistency.py`：纯文本对比 Pydantic/Schema/ROS/Proto，验证正式 Profile 与删字段变异。
- `test_offline_audit.py`：纯数据审计、ROS 依赖隔离及 contracts/仿真核心架构边界。
- `conftest.py`：提供跨测试共享的仓库路径与环境配置。
- `act_fixtures.py`、`learning_fixtures.py`、`model_helpers.py`：提供仅限测试使用的合成数据与模型辅助构造方法。

单元测试默认不启动外部 ROS 守护进程或请求网络；涉及特定运行端依赖（如 PyTorch 或 MuJoCo）的测试在对应容器内执行。

目录导航：[上级目录](../README.md)
