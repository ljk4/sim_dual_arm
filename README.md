# sim_dual_arm

双臂机器人真实/仿真采集、训练与推理系统。当前已实现阶段 1～2 的 CPU mock 控制与采集链路，物理仿真、训练和真机接入按路线图继续完善。

当前暂停新增功能，先处理 [可维护性审查](docs/maintainability-review.md) 中的中断收尾、观测身份和控制清理问题，再进入阶段 3。现有成功演示仍可运行；已知限制及整改顺序见审查记录。

推荐分为 backend-real、backend-sim、runtime、learning 四类环境；真实与仿真后端二选一，应用通过统一 Robot API 接入设备，通过 Policy API 调用模型。

从仓库根构建并运行：

```bash
bash tools/build/stage2.sh
bash tools/run/mock_collect.sh
```

启动器会运行两个独立 ROS 容器，执行脚本动作、生成测试图像、保存并离线审计 MCAP，然后把 episode 导出到 var/runs。无需 GPU、宿主机 ROS 或 Compose 插件。

- [阶段 1～2 使用与验证指南](docs/stage2-quickstart.md)

设计入口：

- [逐步实现路线图](docs/implementation-roadmap.md)
- [目录划分与组织规范](docs/repository-layout.md)
- [环境边界与通信接口](docs/architecture-and-interfaces.md)
- [环境与依赖分析](docs/docker-dependency-analysis.md)
- [仿真环境设计](docs/simulation-design.md)
- [全部文档](docs/README.md)

主要实现与契约：

- [Policy API v1 草案](contracts/policy/v1/policy.proto)
- [仿真采集部署绑定示例](deploy/examples/sim_collect.yaml)
- [公共 ROS 接口](contracts/ros/robot_api_msgs/CMakeLists.txt)
- [后端控制核心](libraries/backend_core/src/robot_backend_core/engine.py)
- [runtime 与采集](components/runtime/ros/dual_arm_runtime/dual_arm_runtime/node.py)

数据与实验产物放 var，可重建结果放 .build。mock 记录明确标记为测试数据；learning:protocol 目前只验证 SDK，尚未实现模型推理服务或训练。
