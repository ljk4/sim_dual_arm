# MuJoCo 配置

| 目录 | 用途 |
| --- | --- |
| [backend/](backend/README.md) | 后端启动配方，选择场景、控制、profile 与任务 |
| [scene/](scene/README.md) | 物理场景、学习相机变体与堆叠任务 |
| [control/](control/README.md) | 控制、跟随和流式执行参数 |
| [checks/](checks/README.md) | 场景可见性与关键位姿检查门限 |
| `container.env` | simulation 容器的运行路径及 ROS/EGL 环境设置 |

`container.env` 由 Compose 的 `env_file` 加载；宿主 bind 来源由本端
`docker/docker-compose.yml` 指定。修改场景/动作语义时同步校验对应 profile。
当前运行配方与参数保留；两份仅供历史对照的后端配置已移至
[测试夹具](../../../tests/fixtures/simulation/README.md)，不进入日常配置目录。

目录导航：[上级目录](../README.md)。
