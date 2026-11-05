# 学习与策略服务

ROS-free 的 ACT 策略服务、协议适配、bundle/checkpoint 校验、数据转换和训练工具。
`src/dual_arm_learning/` 是唯一包，公共 CLI 为 `cli.py`，策略服务为
`act_service.py`。容器内唯一入口为 `learning/lerobot/scripts/run.sh`：

- `prepare-policy`：验证真实 ACT bundle，生成通用 `policy_binding.json` 和可执行
  `policy.yaml`；模型/processor/profile/schema 摘要均由 bundle 与 shipped 文件计算。
- `convert`、`validate`、`filter`、`train`、`bundle`、`verify-bundle`、`budget`：
  复用原 `dual_arm_learning.cli` 的转换、数据校验、筛选、训练预算、模型包能力。
- `policy`：启动 ROS-free Policy 服务；runtime 默认绑定 `learning:50051`，binding 可配置一致端口的 host:port。
- `test`：固定关闭自动插件收集的 pytest 入口。

配置在 `configs/`，镜像/Compose 在 `docker/`；数据、模型和缓存属于运行产物，
不提交。学习环境不挂 ROS，也不导入 runtime。当前阶段见
[项目状态](../../docs/project-status.md)，实现历史见
[DSH-035](../../docs/history/README.md#dsh-035)、[DSH-036](../../docs/history/README.md#dsh-036)。

目录导航：[上级目录](../README.md)。
