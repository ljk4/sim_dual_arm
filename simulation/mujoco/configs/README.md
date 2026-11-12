# MuJoCo 配置说明

本目录只保留仿真端私有运行环境；Panda 实例配方已迁至方案目录。

| 目录/文件 | 用途说明 |
| --- | --- |
| `container.env` | `simulation` 容器内运行路径、ROS 2 域 ID 与 NVIDIA EGL 环境变量 |
| [`shared/scenarios/panda_stack/configs/simulation/`](../../../shared/scenarios/panda_stack/configs/simulation/) | 后端启动、物理场景/任务与静态检查配方 |
| [`tests/fixtures/simulation/`](../../../tests/fixtures/simulation/README.md) | 历史回归对照专用配置夹具（测试归属，不进入方案） |

- 方案选择由 [`binding.yaml`](../../../shared/scenarios/panda_stack/binding.yaml) 声明，数值仍只维护在对应 recipe。
- 跨端共享的控制限值、受限跟随与流式超时配置集中维护在 [`shared/scenarios/panda_stack/profiles/control/`](../../../shared/scenarios/panda_stack/profiles/control/README.md)。

目录导航：[上级目录](../README.md)
