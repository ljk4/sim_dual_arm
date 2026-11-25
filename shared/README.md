# 公共层与方案绑定（Shared）

本目录只放跨端复用的公共内容、机器人本体资产与三层方案绑定。业务实现和运行生命周期位于 `simulation/`、`runtime/`、`learning/`、`hardware/` 四端。

## 目录职责

```text
shared/
├── common/
│   ├── interfaces/      # 公共接口约束：Policy Protobuf、ROS 2 IDL、JSON Schema
│   ├── python/          # 公共 Python 包：contracts、ros_support、trajectory、run_logging
│   └── tools/           # 容器环境、SDK 构建、测试、binding 校验、边界检查与宿主机并行编排
├── robots/
│   └── marvin/          # 机器人本体资产：URDF、meshes、共享 profiles 与扩展 ROS IDL（marvin_teacher_api_msgs）
└── scenarios/
    ├── _template/       # 新方案说明与不可直接运行的 binding.example.yaml
    └── marvin/
        └── mujoco/
            ├── stack/    # 实例：Marvin + MuJoCo 单臂堆叠
            └── handover/ # 实例：Marvin + MuJoCo 抓取—空中交接—入箱
```

| 层 | 回答的问题 | 不承担 |
| --- | --- | --- |
| `common/interfaces/` | 跨机器人、跨算法都必须遵守的协议与结构约束 | 具体关节顺序、相机标定、训练数值 |
| `common/python/` | 可被多端复用的纯数据契约、数学与日志实现 | ROS 运行生命周期、仿真物理、模型训练 |
| `common/tools/` | SDK、容器、测试与宿主机并行编排入口 | 业务选择、模型权重、数据产物 |
| `robots/<robot>/` | 单一机器人的几何、网格与跨后端/跨任务共享档案和接口 | 特定仿真器物理步进、任务评价器与训练超参 |
| [`scenarios/<robot>/<backend>/<task>`](scenarios/marvin/mujoco/stack/README.md) | 当前三层方案选择哪套档案、配方与专用接口 | 四端业务实现和数值门禁本身 |
| `simulation` / `runtime` / `learning` / `hardware` | 实际能力、依赖、容器与入口脚本 | 公共协议的私有副本 |

方案只通过 `binding.yaml` 做选择；限位、相机尺寸、训练参数仍由档案与配方持有。公共生产包不按机器人、后端、任务或算法名硬编码分支，也不反向导入四端。

## 公共接口与运行摘要

- Robot API（`common/interfaces/ros/robot_api_msgs/`）定义观测、控制、跟随参考与后端能力。
- Policy API（`common/interfaces/policy/policy.proto`）定义模型信息、会话及观测与动作块交换。
- JSON Schema（`common/interfaces/schemas/`）定义档案与 Episode Manifest 结构。
- 方案选择记录使用 `ResolvedReference.record()` 生成 `field/declared/path/sha256`，有语义摘要时附 `semantic_sha256`（`RobotProfile` 额外提供剥离 `profile_id`/`initial`/`calibration_digest` 的 `interface_digest`）。`--robot <robot> --backend <backend> --task <task>` 启动时，后端和 runtime 快照记录配方选择与覆盖项，worker 清单在顶层 `selection.profiles` 记录档案。

`output/`（含生成的 `output/build/`）是运行时输出目录，由使用者或 `./env.sh up` 初始化，不纳入 Git。初始化与操作步骤见[根 README](../README.md)。

目录导航：[上级目录](../README.md) · [Agent 指导与规范](../AGENTS.md) · [方案模板](scenarios/_template/README.md) · [跨端架构分工与解耦审查](../docs/features/跨端架构分工与解耦审查.md) · [架构扩展与演进指南](../docs/features/架构扩展与演进指南.md)
