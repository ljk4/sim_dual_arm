# 单元测试（Unit Tests）

覆盖 `robot_contracts`、轨迹数学、`runtime` 状态机与离线审计导出、`simulation` 配置/运动/接触/制动、各场景插件以及 `learning` 数据集/筛选/训练/策略服务的纯 Python 单元测试。

- [`conftest.py`](conftest.py)：在 `unit/` 根目录提供跨测试共享的仓库路径与源码 `sys.path` 配置。
- `fixtures/`：测试专用的合成数据与方案夹具（`act_fixtures.py`、`collection_fixtures.py`、`follower_fixtures.py`、`learning_fixtures.py`、`learning_scenario_fixtures.py`、`model_helpers.py`、`scenario_fixtures.py`）。
- `shared/`：公共契约一致性（`test_contract_consistency.py`）、Profile/相机/路径、方案绑定 CLI 与轨迹数学测试。
- `simulation/`：仿真配置、制动、接触卸载、夹爪完成、场景可见性与堆叠/交接任务及示教测试。
- `runtime/`：运行配置、流式策略调度、观测配对、采集池与离线审计导出评估测试。
- `learning/`：数据集构建、静止锚点过滤、训练、模型单文件包与 gRPC 策略服务测试。
- `infra/`：容器构建声明、入口脚本边界及 ROS/Policy SDK 工作区校验测试。

单元测试默认不启动外部 ROS 守护进程或请求网络；涉及特定运行端依赖（如 PyTorch、ROS 或 MuJoCo）的测试在对应容器内执行。

目录导航：[上级目录](../README.md)
