# 公共契约

协议源码的唯一维护位置；职责与数据语义见 [接口设计](../docs/architecture-and-interfaces.md)，目录规则见 [组织规范](../docs/repository-layout.md)。

- `policy/v1/policy.proto`：推理协议，已在 ROS 与独立 Python 镜像中生成并导入 SDK，服务实现属于阶段 3。
- `ros/robot_api_msgs/`：已实现的阶段 1～2 ROS msg/srv/action 包。
- `schemas/v1/`：mock profile、观测、动作、运行配方与 episode manifest 的结构定义。
- `python/`：无 ROS/ML 依赖的有类型数据模型、配置校验与语义哈希。

机器人语义实例放在仓库根 profiles，部署绑定放 deploy。生成的 ROS/Protobuf 文件通过构建生成，不在各组件里复制或手工修改。
