# 部署文件

负责镜像构建、容器组合、设备/网络/挂载绑定。目录规划见 [组织规范](../docs/repository-layout.md)。

- `examples/sim_collect.yaml`：已有的设计示例，不是可执行 Compose 文件。
- `docker/`：已有 ROS 基础层、backend-sim 的 mock target、runtime/test、learning 的 protocol target。完整 Gazebo/训练/真机镜像仍待实现。
- `compose/mock.yaml`：阶段 2 的可选 Compose 定义；默认启动器直接使用 Docker CLI，已实际验证。
- `local/`：当前机器的非公开绑定与覆盖，已通过根 .gitignore 排除。

控制、采集、模型和 episode 逻辑属于 components；deploy 只启动它们。

实际构建/运行命令见 [使用指南](../docs/stage2-quickstart.md)。examples 下的通用绑定文件仍为设计示例，当前执行配方是 configs/runtime/mock_collect.yaml。
