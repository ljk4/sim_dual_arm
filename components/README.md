# 组件

按运行职责划分的业务组件；构建、部署和薄入口在 `deploy/`、`tools/`。

- `runtime/`：ROS 2 运行层，负责观测、控制会话、记录和收尾。
- `backend_sim/`：MuJoCo 双 Panda 物理后端、渲染、任务与 teacher。
- `learning/`：ROS-free 的 ACT 策略服务、训练、数据转换和模型包。

公共语义在 `contracts/`，共享实现按用途放 `libraries/`；组件不复制协议定义。
规则见 [项目规范](../docs/standards.md)，当前阶段见 [项目状态](../docs/project-status.md)。

目录导航：[上级目录](../README.md)。
