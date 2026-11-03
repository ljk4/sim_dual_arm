# Compose 组合

- `mock.yaml`、`mock_collect.yaml`、`mock_audit.yaml`：CPU mock 开发、自动采集和
  只读审计。
- `native_backend.yaml`、`native_runtime.yaml`、`native_policy.yaml`：MuJoCo
  backend、原生诊断 runtime 和连续 Policy overlay。
- `native_act_policy.yaml`、`native_task.yaml`、`act_training.yaml`、
  `label_replay.yaml`：ACT 服务/任务、训练和标签重放。
- `sim_check.yaml`：一次性依赖/资产/EGL 检查。

组合声明镜像、命令与挂载，不复制控制或模型算法。源码、数据输入、配置与资产
优先只读；日志、输出、缓存以及构建服务生成的 SDK 使用各自的显式可写位置。
本机覆盖文件放 `deploy/local/`。

目录导航：[上级目录](../README.md)。
