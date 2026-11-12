# 阶段 02：方案绑定模型与只读工具

状态：待实施。本阶段遵守[总要求](README.md)，只完成本文件范围。

## 前置与读取范围

上一阶段 01 的必要验收通过；读取 `deliveries/phase-01.md` 的接口与限制。

必读总要求、本文件、已有 progress 和上阶段交接；源码按下面入口查找。除明确引用外，不要求阅读其他阶段全文。

## 目标与范围

实现轻量、严格、ROS-free 的 binding 模型和只读解析工具；不接入四端业务、不自动启动服务。真实方案入口在阶段 03～05 接入。

## binding 规范

### 示例结构

下面是待实现的规范性示例。目标路径应由迁移实际落地，不得仅把本例放进 README 就视为已支持。模型为空时允许验证采集配置，推理准备必须报出缺失模型。

```yaml
schema_version: 1
scenario_id: panda_stack
description: MuJoCo 双 Panda 方块堆叠，支持程序示教与 ACT 策略评估

backend:
  kind: simulation
  implementation: mujoco
  task_type: stack

runtime:
  implementation: ros2

learning:
  implementation: lerobot
  adapter: act

profiles:
  robot: profiles/robots/panda_stack_learning_v1/profile.json
  observation: profiles/observations/panda_rgb_joint_v1.json
  action: profiles/actions/panda_joint_position_v1.json

recipes:
  simulation_teacher: configs/simulation/backend/backend_stack_teacher.yaml
  simulation_policy: configs/simulation/backend/backend_stack_act_chunked.yaml
  runtime_teacher: configs/runtime/teacher_collect.yaml
  runtime_policy: configs/runtime/policy_rollout.yaml
  campaign: configs/runtime/stack_campaign.yaml
  dataset: configs/learning/act_dataset.container.yaml
  training: configs/learning/act_training.container.yaml
  policy_service: configs/learning/policy_temporal.yaml

interfaces:
  common_ros: [robot_api_msgs, sim_api_msgs]
  extension_ros: [extensions/ros/panda_stack_api_msgs]
  policy_api: bimanual.policy.v1

models:
  bundle: null
```

`implementation` 和 `adapter` 是声明值，由对应端校验支持性；公共解析器不根据它们导入业务代码，也不猜测源码路径。`backend.kind` 支持 simulation/hardware 的结构表达，不代表本仓库已具备真机实现。

### 路径与输入规则

1. 方案内的配置路径以 `binding.yaml` 所在目录为基准，不依赖当前工作目录。公共 Schema 路径从显式 shared root 解析。
2. 仿真和学习配方仍保留各自现有相对路径语义；移动时更新到唯一来源，解析器不得静默改变旧配方解释方式。
3. 仓库内的可跟踪配方不写 `/home/ljk/...` 等主机绝对路径。容器部署位置由部署配置提供。
4. 运行数据与模型可使用 `/data/...` 等部署路径，必须显式区分源配置引用与运行产物引用。模型路径不纳入机器人语义摘要。
5. `--bundle` 可覆盖 `models.bundle`，但必须在有效配置和运行记录中保存覆盖后的真实来源与摘要。禁止伪造占位摘要。
6. `--scenario` 与直接选择同类 `--config`/`--template` 的参数互斥，冲突时报错；seed、run-id、output、模型引用等明确运行参数可覆盖并记录。
7. 对于不适用的端或阶段，字段可以显式为空，并按阶段校验；例如采集不要求模型，推理不要求训练输出存在。
8. 禁止方案自动发现、隐式选中 Panda、YAML 深层继承、任意 shell 命令执行和自动启动服务。模板目录不参与真实方案选择。
9. 从任意引用文件递归收集依赖时，要检测不存在路径、循环引用和路径类别错误；允许明确的跨方案复用，实际读取范围限于声明的源根及产物根。

### 最小公共 API 与工具

在 `robot_contracts` 中提供独立的方案绑定模型/解析模块，仅负责严格输入校验、路径解析、配置引用和摘要。结构尽量直接，复用现有 Pydantic 基础模型，不创建双层 DTO。

工具统一放 `shared/common/tools/`，提供以下等效能力；脚本名称可采用示例名称，但实现记录和全部文档须一致：

```text
python3 shared/common/tools/scenario.py validate --binding <binding.yaml> --stage <collection|learning|inference> [--bundle <path>]
python3 shared/common/tools/scenario.py show --binding <binding.yaml>
```

- `validate` 只读输入，不创建 campaign 台账、不预留种子、不加载权重、不连接 ROS、不启动训练或后端。
- `show` 输出有结构的有效绑定，包含来源、解析路径和阶段要求。不能靠输出 shell 代码供 `eval` 执行。
- 缺失字段、未知字段、未知格式版本、重复 YAML 键、文件缺失、档案绑定不一致、未满足的阶段条件都有明确错误位置。
- Profile、observation、action 的校验调用已有公共模型；端私有配方的完整语义校验继续由所属端执行。
- 对有内容的文件计算摘要；模型目录摘要沿用现有 bundle 校验，不增加一套不兼容规则。

## 当前阶段补充

- 用临时测试目录和合成源文件验证完整结构；示例中的目标 recipe/扩展包尚未全部存在，不能据此报告真实方案已可运行。
- 支持声明 simulation/hardware，不等于接入硬件；支持 `implementation` 字符串不等于公共层导入该实现。
- 声明可读取的源根至少能覆盖仓库内现有端配置和 shared 配置，阶段 03/04 移动后仍用同一规则；跨根引用须显式，不能按 cwd 猜根。
- 路径字段中的模型目录只做阶段需求及存在性检查，模型内容交学习端 bundle 校验；不加载权重。

## 验收

- 测试正确输入、重复键、未知字段/版本、缺文件、非法路径类别、循环引用、显式跨方案复用与阶段条件。
- 从不同 cwd 得到相同结果；无隐式 Panda 或深层 YAML 继承。
- 证明 validate/show 不启动进程、不导入 ROS/MuJoCo/PyTorch、不创建台账或预留种子。
- 交接记录模型/API 名、返回结构、错误类型、路径根与摘要规则，供下阶段直接调用。

## 交付与停止点

- 更新 `progress.md`，写 `deliveries/phase-02.md` 简短交接；基线清单只查询/补充相关行。
- 交接列出实际入口/接口、关键决定、命令与结果、未运行项及原因，状态必须真实。
- 完成本阶段后停止，不自动进入下一阶段。必要验证未完成时保留“待验证”，不得写通过。

目录导航：[总要求与阶段索引](README.md)
