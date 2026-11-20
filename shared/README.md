# 公共层与方案绑定（Shared）

本目录只放跨端复用的公共内容，以及把当前任务、机器人和实现选择起来的方案绑定。业务实现和运行生命周期在四端（`simulation/`、`runtime/`、`learning/`、`hardware/`）。

## 目录职责

```text
shared/
├── common/
│   ├── interfaces/      # 公共接口约束：Policy Protobuf、ROS 2 IDL、JSON Schema
│   ├── python/          # 公共 Python 包：contracts、ros_support、trajectory、run_logging
│   └── tools/           # 容器环境、SDK 构建、测试、binding 校验与部署检查
└── scenarios/
    ├── _template/       # 新方案说明与不可直接运行的 binding.example.yaml
    └── marvin_sim/
        ├── stack/       # 实例：Marvin 固定基座双臂单臂堆叠、ROS 2 运行端、LeRobot ACT
        └── handover/    # 实例：Marvin 抓取—空中交接—入箱
```

| 层 | 回答的问题 | 不承担 |
| --- | --- | --- |
| [`common/interfaces`](common/interfaces/policy/README.md) | 跨机器人、跨算法都必须遵守的协议与结构约束 | 具体关节顺序、相机标定、训练数值 |
| [`common/python`](common/python/README.md) | 可被多端复用的纯数据契约、数学与日志实现 | ROS 运行生命周期、仿真物理、模型训练 |
| [`common/tools`](common/tools/README.md) | SDK、容器、测试与只读校验入口 | 业务选择、模型权重、数据产物 |
| [`scenarios/<robot>/<task>`](scenarios/marvin_sim/stack/README.md) | 当前方案选择哪套档案、配方与专用接口 | 四端业务实现和数值门禁本身 |
| `simulation` / `runtime` / `learning` / `hardware` | 实际能力、依赖、容器与入口脚本 | 公共协议的私有副本 |

方案只通过 `binding.yaml` 做选择；限位、相机尺寸、训练参数仍由档案与配方持有。公共生产包不按 Marvin、stack、handover、ACT 或 MuJoCo 名称分支，也不反向导入四端。

## 公共接口与运行摘要

- Robot API 定义观测、控制、跟随参考与后端能力。 [ROS 2 接口](common/interfaces/ros/robot_api_msgs/README.md)
- Policy API 定义模型信息、会话及观测与动作块交换。 [Policy 协议](common/interfaces/policy/README.md)
- JSON Schema 定义档案、Episode Manifest 和运行日志的结构。 [JSON Schema](common/interfaces/schemas/README.md)
- 方案选择记录使用 `ResolvedReference.record()` 生成 `field/declared/path/sha256`，有语义摘要时附 `semantic_sha256`。`--robot <robot> --task <task>` 启动时，后端和 runtime 快照记录配方选择与覆盖项，worker 清单在顶层 `selection.profiles` 记录档案。

`build/` 和 `var/` 是运行时输出目录，由使用者初始化、由命令创建子目录，均不纳入 Git。初始化步骤见[根 README](../README.md)。

目录导航：[上级目录](../README.md) · [Agent 指导与规范](../AGENTS.md) · [方案模板](scenarios/_template/README.md)
