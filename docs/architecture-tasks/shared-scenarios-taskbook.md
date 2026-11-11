# 公共层与方案绑定迁移任务书

- 日期：2026-11-11（Asia/Shanghai）
- 仓库：`/home/ljk/work/sim_dual_arm`
- 状态：设计任务书，尚未实施。本文中的目标路径、命令和格式均是待交付要求，不能当作现有功能使用。
- 配套材料：[实现提示词](shared-scenarios-prompts.md)。

## 1. 目标与判定标准

将当前分散的 `contracts/`、`libraries/`、`infrastructure/`、`profiles/` 收拢到一个 `shared/` 入口，按以下两类组织：

1. `shared/common/`：稳定公共接口、公共 Python 包和通用工具。
2. `shared/scenarios/<scenario_id>/`：具体任务、机器人、后端与学习方案的配置绑定，以及确实专用的接口扩展。

首个真实方案命名为 `panda_stack`，README 说明它对应当前 MuJoCo、双 Panda、方块堆叠及 LeRobot ACT 配方。名称不包含 checkpoint、随机种子或主机信息。另提供 `_template/`，说明换什么时改什么。

验收目标是：**在已支持的能力范围内，新增或切换方案通过显式绑定完成，不修改既有公共实现，不在公共代码中增加方案名称分支。**

不承诺公共协议永久不变；新增触觉、力矩控制或新的曲线表示时，应按能力及协议版本演进。新增尚未接入的机器人或后端，仍需在对应端实现驱动或适配器。

本任务同时包含目录归并与有限的接口边界整理，必须分阶段执行，不能用一次全仓文本替换完成。

## 2. 范围、保护对象与非目标

### 2.1 必须交付

- 一个公共入口、一个真实方案、一个示例方案模板。
- 严格且轻量的方案绑定模型、只读校验与解析工具。
- 采集、转换/训练准备、策略部署及推理入口对方案绑定的实际消费。
- SDK 生成、环境路径、容器挂载、测试路径和活跃文档的同步迁移。
- 任务专用 Teacher 接口与通用接口的边界整理。
- 对固定末端数量、求解器身份、部署检查规则的有限解耦，具体见第 8 节。
- 分阶段验证、已知限制、迁移映射和最终交付记录。

### 2.2 四端继续保留的所有权

| 目录 | 所有的实现与状态 |
| --- | --- |
| `simulation/mujoco/` | MuJoCo 模型与步进、渲染、执行器映射、Ruckig 调用、任务插件、Teacher、仿真评价与 ROS 服务实现 |
| `hardware/` | 后续真实驱动及硬件适配；当前只有边界说明，不新增假驱动 |
| `runtime/ros2/` | 会话、租约、观测同步、动作流、录制、导出、审计、采集编排及任务/Teacher 的客户端适配 |
| `learning/lerobot/` | LeRobot 数据转换、训练、模型导出、Policy 服务及 ACT 适配 |

四端的 Dockerfile、依赖锁、第三方版本选择、`container.env`、命令分发脚本仍属于对应实现。方案目录可以保存带 MuJoCo 或 ACT 语义的配置数据，不承接物理、训练和控制业务代码。

### 2.3 必须保护

- 工作区已有大量用户修改及未跟踪文件；以开始实施时的工作树内容为基线，不能用 HEAD 代替。
- 不执行自动提交、推送、stash、reset、clean，不恢复用户已删除的文件。
- 不搬移、删除或重写 `var/` 的原始数据、模型、快照、台账和既有实验产物。
- 不修改已完成实验的随机种子、预算、训练/验证划分、质量标签或成功统计。
- 不借迁移修改控制限位、跟随预算、相机、任务判据、训练参数和夹爪准入参数。
- 不覆写旧 prepared 文件来修复路径，不伪造摘要绕过旧产物校验。
- 不修改第三方上游源码、许可证、Panda 模型或固定 Ruckig wheel。
- 只清理确认属于本轮的进程和测试资源；重建前先确认容器和数据挂载是否被其他工作使用。

### 2.4 非目标

- 修复当前 ACT 推理任务失败、提升成功率、重新采满正式 50 条或重新完整训练。
- 实现第二个真实物理后端、真实机器人、触觉输入、力矩控制或灵巧手控制。
- 建立自动启动四端的全局编排服务、任意代码插件平台或公共业务状态机。
- 强制把所有测试移动进四端，或把第三方源码与运行产物塞入 `shared/`。
- 为尚不存在的实现创建大量空目录或占位驱动。

## 3. 已核对的当前基线与风险

以下是 2026-11-11 的源码观察，实现前应再次检查，不得当作运行验证结果。

| 当前事实 | 迁移含义 |
| --- | --- |
| `contracts/` 有 Policy Proto、两个 ROS 接口包、7 份 JSON Schema、Python 契约包 | 要逐文件分类，不能把整个目录原样标记为完全通用 |
| `robot_api_msgs` 有 15 个消息、9 个服务；`sim_api_msgs` 有 1 个消息、5 个服务 | 修改包归属时同步 CMake、依赖声明、生成器、导入、服务端、客户端和测试 |
| Python 公共包未直接导入四端业务实现 | 保留此依赖方向，不为方案绑定增加反向导入 |
| `GetTeacherChunk.srv` 是固定 60×16，且阶段码属于堆叠 | 可作为已有方案专用扩展保留，不能放进通用 Teacher 协议冒充任意任务支持 |
| 执行反馈固定左右两个双指夹爪 | 公共数据容器需要支持可选数量；Panda 接触语义仍由具体实现检查 |
| `FollowerReference` 只接受带 Ruckig 0.15.3 的算法标识 | 区分公共表示版本与生产者实现来源，不能只删除严格校验 |
| `runtime_config.schema.json` 与当前 `RuntimeRecipe`、模板不一致 | 核实调用方后撤下失效重复定义；Runtime 私有配置由 runtime 所有 |
| `dev_workspace.py` 强制验证两个 ROS 包，且有固定 runtime 模块路径 | 按选中的包集构建与校验，通用 SDK 校验不导入具体 runtime |
| `check_dockerfiles.py` 对 simulation 强制 NVIDIA 和固定挂载 | 实现要求由各端声明，公共检查器验证通用约束和声明的一致性 |
| 根 pytest 路径、各端环境变量、prepare 脚本和并行录制存在旧公共路径 | 扫描所有入口；不能只让普通 Python import 通过 |
| campaign 的 `required_paths` 仍有已迁移服务的旧路径 | 这是待核实的存量漂移，要纳入方案路径校验，不能原样复制错误引用 |
| `hardware/` 尚无真实驱动 | 只能验证接口依赖隔离，不能声称已通过真机测试 |

当前根文档与部分局部说明还引用不存在的 A1～A7 文档。只修复活跃导航，不编造历史验收记录。

## 4. 目标目录与分类规则

```text
shared/
├── README.md
├── common/
│   ├── README.md
│   ├── interfaces/
│   │   ├── policy/v1/policy.proto
│   │   ├── ros/robot_api_msgs/
│   │   ├── ros/sim_api_msgs/          # 可选仿真/任务生命周期；不含堆叠 Teacher
│   │   └── schemas/v1/               # 公共文档与新增 binding 格式
│   ├── python/
│   │   ├── contracts/               # 保留 pyproject.toml + src/robot_contracts
│   │   ├── trajectory/              # src/robot_trajectory
│   │   ├── ros_support/             # src/robot_ros_support
│   │   └── run_logging/             # src/robot_run_logging
│   └── tools/                       # SDK、setup、DDS、测试与检查工具
└── scenarios/
    ├── README.md
    ├── _template/
    │   ├── README.md
    │   └── binding.example.yaml
    └── panda_stack/
        ├── README.md
        ├── binding.yaml
        ├── profiles/
        │   ├── robots/              # 保留现有两个档案及标定附件
        │   ├── observations/
        │   ├── actions/
        │   └── control/
        ├── configs/
        │   ├── simulation/          # backend、scene、checks 配方
        │   ├── runtime/             # teacher、policy、campaign 配方
        │   └── learning/            # dataset、training、policy 配方
        └── extensions/
            └── ros/panda_stack_api_msgs/
                └── srv/GetTeacherChunk.srv
```

图中只为现有内容和本轮必要功能设目录。每个 ROS 包仍须有正常的包元数据和构建入口。`extensions/` 仅在存在专用定义时创建；专用接口使用自身包名。

Python 包名保持 `robot_contracts`、`robot_trajectory`、`robot_ros_support`、`robot_run_logging`，不增加旧路径转发包或通配导出，不将它们合并成 `utils`。发行包版本仅在兼容性确有变化时调整并记录。

### 4.1 分类原则

- 决定两个端如何交换数据的格式，可进入公共接口；只被一个实现读取的配置格式由该实现所有。
- 通用结构校验与资源上界可以在 common，实验参数及机器人名称进入方案。
- 公共 ROS 转换可以依赖 ROS；ROS-free 包不得因绑定解析而导入 ROS。
- 默认关节数、相机名、任务阶段、模型类型和具体方案名不得隐藏在公共加载器中。
- 同一份 Profile、控制约束和标定附件只保留一个权威来源，其他配置通过路径引用。
- 复用已有档案时允许显式引用另一方案的权威文件；解析结果记录真实来源，不做隐式继承、目录回退或文件复制。确有多个方案复用后再决定是否提取资源目录，本轮不预建资源注册中心。
- `third_party/`、`tests/`、`.build/`、`var/` 保留；它们是源码存储、验证或产物类别，不需要伪装成永不变化的公共实现。

### 4.2 旧路径到新归属的最低映射

| 旧来源 | 目标归属 |
| --- | --- |
| `contracts/policy/` | `shared/common/interfaces/policy/` |
| `contracts/ros/robot_api_msgs/` | `shared/common/interfaces/ros/robot_api_msgs/` |
| `contracts/ros/sim_api_msgs/` 的通用生命周期与复位接口 | `shared/common/interfaces/ros/sim_api_msgs/`，按可选包处理 |
| `contracts/ros/sim_api_msgs/srv/GetTeacherChunk.srv` | `shared/scenarios/panda_stack/extensions/ros/panda_stack_api_msgs/srv/` |
| `contracts/schemas/` 的公共格式 | `shared/common/interfaces/schemas/` |
| `contracts/schemas/v1/runtime_config.schema.json` | 核实无调用后删除；如需保留准确版本，放 `runtime/ros2/` 私有格式目录并维护真实模型一致性 |
| `contracts/python/` | `shared/common/python/contracts/` |
| `libraries/trajectory/`、`ros_support/`、`run_logging/` | `shared/common/python/` 对应包目录 |
| `infrastructure/scripts/`、`infrastructure/docker/` 的通用工具 | `shared/common/tools/`；保留职责明确的脚本，不建立旧入口转发层 |
| `profiles/` | `shared/scenarios/panda_stack/profiles/` |
| 仿真端当前 Panda backend、scene、checks 配方 | `shared/scenarios/panda_stack/configs/simulation/` |
| runtime 的 teacher、policy、stack campaign 配方 | `shared/scenarios/panda_stack/configs/runtime/` |
| learning 的 Panda/ACT 数据、训练、策略配方 | `shared/scenarios/panda_stack/configs/learning/` |
| 各端 `container.env`、Dockerfile、依赖锁、源码与资产 | 保留在所属实现目录 |
| Fake Policy、仅用于故障注入或回归的配置 | 保留测试归属，不能因被配置引用而成为正式方案默认 |

对基线文件、容器/非容器重复配方、历史夹具逐个登记为迁移、保留、合并或删除，写明调用证据。不得只按文件名批量删除。

## 5. 方案绑定的最小契约

### 5.1 示例结构

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

### 5.2 路径与输入规则

1. 方案内的配置路径以 `binding.yaml` 所在目录为基准，不依赖当前工作目录。公共 Schema 路径从显式 shared root 解析。
2. 仿真和学习配方仍保留各自现有相对路径语义；移动时更新到唯一来源，解析器不得静默改变旧配方解释方式。
3. 仓库内的可跟踪配方不写 `/home/ljk/...` 等主机绝对路径。容器部署位置由部署配置提供。
4. 运行数据与模型可使用 `/data/...` 等部署路径，必须显式区分源配置引用与运行产物引用。模型路径不纳入机器人语义摘要。
5. `--bundle` 可覆盖 `models.bundle`，但必须在有效配置和运行记录中保存覆盖后的真实来源与摘要。禁止伪造占位摘要。
6. `--scenario` 与直接选择同类 `--config`/`--template` 的参数互斥，冲突时报错；seed、run-id、output、模型引用等明确运行参数可覆盖并记录。
7. 对于不适用的端或阶段，字段可以显式为空，并按阶段校验；例如采集不要求模型，推理不要求训练输出存在。
8. 禁止方案自动发现、隐式选中 Panda、YAML 深层继承、任意 shell 命令执行和自动启动服务。模板目录不参与真实方案选择。
9. 从任意引用文件递归收集依赖时，要检测不存在路径、循环引用和路径类别错误；允许明确的跨方案复用，实际读取范围限于声明的源根及产物根。

### 5.3 最小公共 API 与工具

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

## 6. 四端接入与配置归属

### 6.1 消费链路

| 入口 | 从方案读取 | 实际业务所有者 |
| --- | --- | --- |
| simulation prepare/backend | 仿真 recipe、机器人档案、公共与扩展接口需求 | simulation |
| runtime prepare/interactive/policy | runtime recipe、档案、控制约束、Policy binding | runtime |
| record/record_parallel/campaign | teacher recipe、backend recipe、campaign 与清理信息 | runtime 的采集编排 |
| learning convert/train/bundle | dataset/training recipe 与档案引用 | learning |
| learning prepare-policy/policy | service recipe、模型包引用 | learning |

选中一个方案不能只影响命令的表面输出。各端必须用选中的实际路径构造有效配置，并在 prepared manifest 中记录 scenario 身份、binding 摘要和实际输入。

CLI 保留直接配置模式作为正常低层入口，不保留旧目录路径作为隐藏 fallback。只传 `--help` 不要求 SDK、模型或 ROS 服务已运行。

### 6.2 内容与引用的单一来源

- 方案绑定主要保存选择关系，具体数值仍维护在对应 recipe。不要在 binding 中再次复制 16 维限值、相机尺寸和训练参数。
- 纯移动阶段保持 Profile、observation、action 内容及语义摘要不变。路径变化导致的 recipe 文件摘要变化属于正常的新运行身份，必须解释。
- 任务阶段、接触门禁、质量规则和成功判定配置归方案；实现代码留在各端。
- 控制器通用默认值归实现，当前 Panda/实验数值归方案；不得借分类重新调参。
- `policy_binding.json` 继续由真实模型包校验生成，不能由 scenario binding 替代其内容真实性校验。
- 记录原始模型输出、准入目标、融合目标、参考轨迹与实测反馈的分层行为不变。

## 7. 容器、SDK 与部署规则

### 7.1 容器路径

本轮采用三端统一的 `SHARED_ROOT=/shared`，Compose 将仓库 `shared/` 只读绑定到 `/shared`。这是容器内部路径约定，不是宿主目录假设。

- simulation 继续自身目录挂载 `/app:ro`；runtime/learning 可以保留 `/workspace:ro` 工作区挂载。
- 删除迁移后失效的 `/contracts`、`/profiles`、`/deps/*` 等旧公共挂载与引用，不为它们创建软链接。
- 各端 `PROJECT_SOURCE_PATHS` 显式指向所需公共 Python 包源码与本端源码。learning 不加载 ROS 或轨迹执行依赖。
- 共享 setup 脚本只设置调用方声明的环境，不按角色拼接具体业务路径，不启动程序。
- `var/`、`.build/`、SDK 输出和第三方源码的既有数据挂载继续有效。先确认真实路径，不创建或搬移宿主 `/data`。
- 修复移动后的脚本根目录推导；`shared` 根与业务项目根是不同概念，SDK 工具使用显式输入，不能依赖错误的 `parents[n]`。

### 7.2 SDK 与可选扩展

- ROS 生成器支持核心包、选中的可选公共包、选中的方案扩展包。核心 Robot API 不要求安装 sim 或 Panda 扩展。
- 无方案输入的核心构建只构建核心；构建 `panda_stack` 完整功能时由 binding 选择额外包并校验依赖闭包。
- 根据选中的包集校验 SDK 完成标记、生成模块归属与源摘要，不能硬编码所有端都必须存在的包列表。
- completion marker 应明确所选包及其源码来源，防止某次核心构建产物被误当作完整 Panda SDK。
- Policy SDK 仍只从公共 Proto 生成，不依赖 ACT 或 LeRobot。
- 修改 `.msg/.srv/.proto`、包构建元数据或选择的包集必须使旧标记失效；README 变化不应使生成代码被判为过期。
- SDK 产物只写 `.build/` 或显式新输出根；不得修改生成文件来假装已迁移。

### 7.3 部署检查

公共检查器保留源码只读、源码/产物隔离、依赖镜像、容器空闲启动等现有共同约束。GPU、渲染方式、挂载需求和本端命令集合由所属实现的机器可读声明提供。

以最小声明文件实现，不引入部署框架。当前 MuJoCo 声明继续要求 NVIDIA/EGL，learning 声明当前 GPU 需求，runtime 继续是 CPU。检查器能验证一个 CPU 仿真声明，且会拒绝违背当前 MuJoCo GPU 声明的配置。不能通过删除检查来达成“通用”。

普通 runtime 不获得 Docker socket；宿主 `record_parallel.sh` 的容器编排职责仍留 runtime，而不是迁入公共工具。

## 8. 有限的契约边界整理

这一阶段在纯迁移与方案接入通过后执行。每项修改分别列出类型、生产者、消费者、审计、离线文件和 SDK 的受影响位置。

### 8.1 通用与专用 ROS 接口

- 复位、通用任务生命周期与不透明任务指标可保留在可选 `sim_api_msgs` 中；它不成为真机控制的强制依赖。
- 固定 60×16、堆叠阶段码和 IK 诊断的 `GetTeacherChunk` 转入 `panda_stack_api_msgs`，保留现有载荷与语义，不顺便设计任意任务 Teacher 平台。
- 更新 simulation ROS 适配、runtime Teacher 客户端及其测试。通用控制和 common 不导入该专用包。
- runtime 中读取专用扩展的客户端适配归 runtime，可按绑定的能力标识选择；不能把 `if scenario_id == "panda_stack"` 塞入公共解析器。
- Proto 注释中“固定三相机、16 关节、60 点”等配方描述移入方案说明。公共字段仍保持明确单位、顺序和时间语义。

### 8.2 末端反馈的最小结构通用化

只将公共容器从“必须左右各一个、各两个手指”调整为可选、有界、具名的反馈集合，不实现新的末端控制器：

- 顶层夹爪反馈允许为空或有界数组，建议上限 8；存在的每个夹爪含有界手指反馈，建议 1～8。
- 现有 `side` 字段作为兼容字段名保留，但值接受有效的非空末端标识；不能再限定 left/right 或固定顺序。
- 对标识唯一、索引唯一、数组长度、有限数值、证据对应关系继续严格检查。手指索引采用明确的正整数上界，不要求固定为 1、2。
- Panda 的双侧、双指、接触同物体与完成条件仍在 Panda 执行/任务适配中校验，现有两侧反馈顺序保持不变。
- 不将吸盘、力矩或灵巧手语义塞入现有接触证据类型。当前可选类型仍是一种明确的夹爪反馈能力。
- ROS IDL、Python 模型、转换器、生产者、消费者和一致性测试同步修改；无夹爪数据的普通反馈能够通过公共层。

### 8.3 曲线表示与求解器来源

- 公共校验依据曲线表示、采样/归一化版本、系数与限值语义；继续拒绝不支持的表示或无效数据。
- 生成者算法与 Ruckig 版本作为来源信息保存，不能由公共验证器要求所有后端都使用 Ruckig 0.15.3。
- 将 `ruckig_duration_ns` 等实现专用字段从公共语义中分离：选定明确的通用时长字段与来源记录，并给出旧记录读取/迁移规则。不得仅改名后改变原有时长含义。
- Ruckig 仍只在 simulation 调用。公共轨迹算法继续消费公开的曲线数据。
- 公共约定的维数、段数、3 秒 horizon 等上界先保持不变并说明适用范围；不要把协议资源边界误当作需要全部参数化的实验数值。

### 8.4 版本与历史产物

纯移动阶段不更改 wire 字段、字段编号、单位或 Profile 格式版本。第 8 节涉及不兼容变化时，必须明确升级对应协议/能力标识及生产者、消费者的拒绝规则，不静默复用旧主版本，不同时升级未改变的 Policy 协议。

旧 prepared 文件、bundle、MCAP 和实验台账原样保留。历史数据读取优先使用记录中的快照和类型描述；若当前离线读取器不能兼容，必须提供可验证的旧版本读取路径或只写到新目录的显式转换，并在报告说明适用范围。不能为追求兼容恢复旧源码目录或改写原始记录。

开始修改协议前，先选一个实际可读取的历史 episode 做读取基线；修改后验证其身份与结果没有被重解释。没有可用样本或旧类型环境时，报告兼容性验证缺口，不伪造通过。

## 9. 分阶段执行与交付

阶段必须顺序执行。每阶段先验证再进入下一阶段，失败先定位；不把前一阶段的半迁移状态带入协议修改。独立的说明补充可继续，但不得跳过必需验证并宣称完成。

| 阶段 | 工作内容 | 可审查交付 | 阶段通过条件 |
| --- | --- | --- | --- |
| S0 基线清点 | 查看用户改动、活跃进程/容器、旧路径调用者、公共包依赖、现有可运行测试与历史读取样本 | implementation 记录中的基线、完整逐文件分类、风险与验证命令 | 不要求工作树干净；既有失败被记录；每个将移动的源均有唯一目标 |
| S1 纯目录迁移 | 公共 interfaces/python/tools 及现有 profiles 归并；暂不修改协议载荷与物理配置 | 可导入的公共包、可解析的配置路径、同步的挂载与 SDK 路径 | 公共契约与档案字节/语义保持；相关原有测试、SDK 构建与静态检查达到基线；旧活跃路径无遗漏 |
| S2 方案绑定 | 建立 binding、迁移实例 recipes、接通各端与并行采集入口 | 可校验、可解析、被实际消费的 panda_stack 方案 | 从不同 cwd 结果一致；选中路径实际进入准备产物；采集无模型可校验，推理缺模型明确失败 |
| S3 边界整理 | 实施第 8 节；拆分扩展 SDK；把部署实现要求改为显式声明 | 专用 Teacher 包、可选反馈集合、曲线版本与来源分离、按包集验证 SDK | 核心使用无需仿真/Panda 包；新旧不兼容明确拒绝；当前 Panda 行为和约束保留 |
| S4 示例与替换验证 | 完成 _template、改动矩阵、测试替身归属与新增方案验证 | 可照着使用的模板、独立协议替身或明确命名的 learner 专用 fake | 新方案在测试临时目录加入后无需修改 common/注册表；模板不能误启动真实任务 |
| S5 集成与文档收口 | 完成最小集成回归、更新导航/规范/流程、删除旧入口与过渡代码 | 最终差异、测试报告、实施记录与命令 | 第 12 节清单满足；限制如实披露；活跃文档可操作 |

S1 中未解耦的既有类型可以暂时原样迁入 common，但必须标记待 S3 处理；不得以 S1 已搬家为由将整个任务记为完成。S0/S1 单独交付时应明确“仅完成阶段”，不宣称方案切换目标达成。

### 9.1 跨文件迁移清单

至少覆盖：

- 各端 run.sh、prepare 脚本、Python 包搜索路径和测试配置。
- simulation worker、render/viewer、tools 的默认配置引用。
- runtime task client、teacher source、offline audit/export、campaign、record、record_parallel。
- learning dataset、training、bundle、policy config、prepare-policy 和 Fake Policy。
- Compose、container.env、SDK marker、生成器根目录、包安装依赖。
- 根 README、docs 导航/standards、四端及公共目录 README。
- JSON/YAML/Proto/IDL/CMake/package.xml 中的引用，不能只搜索 `.py`。

可使用下面的扫描作为起点；结果须逐项分类，不能机械要求历史文件和说明中没有旧词：

```bash
rg -n 'contracts/|libraries/|profiles/|infrastructure/|/contracts|/profiles|/deps/|/config/' \
  --glob '!var/**' --glob '!.build/**' --glob '!third_party/src/**' .
```

## 10. 示例目录的具体要求

`shared/scenarios/_template/README.md` 至少包含：

1. 方案、任务实现、机器人、后端、模型和 checkpoint 的区别。
2. 文件结构、相对路径基准、模型引用与本地部署配置的区别。
3. 创建方案 → 补齐引用 → 只读校验 → 各端 prepare → 隔离验证的顺序。
4. 一个结构完整的 `binding.example.yaml`；使用明显的待填写值，不悄悄继承 Panda 默认值。
5. 下表及对应操作说明。
6. 缺失文件/能力/模型、摘要不匹配、协议版本不匹配等典型错误的定位方式。
7. 正式数据、test_only、运行退出、录制完整性与任务成功的区别。
8. 引用现有权威配置的例子，说明哪些改动必须产生新身份，哪些只是模型路径选择。

| 用户想换什么 | 主要修改位置 | 一般可保持的内容 | 是否需要新实现/公共协议 |
| --- | --- | --- | --- |
| 同模型换 checkpoint | 模型引用或命令行 bundle 参数 | scenario、机器人档案、接口 | 先验证输入输出与档案摘要；通常不用新协议 |
| 同机器人换已接入学习算法 | learning 选择、数据/服务配方 | Robot API、runtime 控制接口 | 新算法适配在 learning；输入输出兼容则不改公共接口 |
| 同任务换已接入机器人 | profiles、控制、场景与相应配方 | 通用接口与工具 | 驱动/映射由对应端提供，不能仅改 robot_id |
| 换已接入仿真后端 | backend、场景和部署声明 | Robot/Policy 公共接口 | 接口能力满足才可复用；不存在的后端需先实现 |
| 新任务 | task_type、任务/示教/评价配方与必要扩展 | 观测/执行的兼容公共格式 | 新逻辑在对应端，专用指标 Schema 在方案 |
| 仿真切真机 | hardware、设备绑定、可用能力与复位/结束方式 | 兼容的核心接口和模型格式 | 当前仓库不能宣称只改配置即可运行真机 |
| 新观测或动作能力 | 对应端与显式协议版本 | 其他未受影响接口 | 如触觉/力矩，属于能力扩展，需要设计与验证 |

更换 checkpoint 不新建模型名目录；需要保留两组配方时使用同方案内显式命名的配方，不引入深层 override 体系。

## 11. 验证计划与证据等级

### 11.1 纯数据及边界验证

- 严格 binding 模型：正确输入、重复键、未知字段/版本、缺文件、路径解析、跨方案引用、循环引用、阶段条件与参数冲突。
- 换 cwd 不影响解析；模板占位值不能被当成真实配置。
- 档案/动作/观测内容保持的摘要比较；配置路径重写前后的有效参数比较，忽略已声明的来源路径和新运行身份。
- 公共依赖禁止反向导入四端；contracts 不导入轨迹算法，ROS-free 包不间接加载 ROS/MuJoCo/PyTorch。
- 以临时新增方案证明 common 无修改；这是配置扩展性证明，不能称为第二个真实后端测试。
- 独立协议替身直接基于公共数据/生成协议；依赖 LeRobot 内部类型的 fake 明确归为学习端专用测试。
- 执行反馈的零夹爪、单夹爪和当前双夹爪结构测试；Panda 的双指同物体接触规则仍在实现测试覆盖。
- 不同生成者产生同种合法曲线表示时，公共验证器依据表示与数值校验；未知表示及非法系数仍失败。
- SDK 选择核心/带扩展两种包集、源变更失效、README 变更不失效、缺模块与过期安装模块拒绝。
- CPU 仿真部署声明能够校验；当前 MuJoCo 缺 GPU 声明要求仍会被拒绝。

### 11.2 各端环境验证

复用现有相关单元/集成测试，新增测试集中在新边界和实际失败路径，不为每个移动文件写镜像测试。业务验证遵守现有容器执行边界。

| 环境 | 最低验证 | 不能替代的结论 |
| --- | --- | --- |
| 宿主静态环境 | 新路径扫描、文档链接、shell 语法、部署静态检查 | 不等于三端可运行 |
| 核心 ROS SDK 环境 | 仅核心接口生成、导入、转换及控制客户端依赖隔离 | 不等于真机验证 |
| 完整 Panda ROS SDK 环境 | 选中可选包与专用 Teacher 包，服务类型一致 | 不等于任务成功 |
| simulation | prepare、场景加载/渲染、既有控制/任务协议相关集成测试 | 不等于模型任务成功率 |
| runtime | prepare、租约/停止、录制与离线导出/审计相关测试 | 不等于学习数据语义完全不变，仍需对照样本 |
| learning | ROS-free 导入、数据/模型包校验、相关转换与策略测试 | 不等于重新完整训练或收敛 |

### 11.3 最小有界闭环回归

在基线与环境允许时执行以下有界回归，所有输出使用独立的新目录和诊断台账，不占用或重写正式采集及评价台账：

1. 通过新方案入口完成一次 Teacher 采集、收尾、导出与审计，对比基线的任务/录制/质量结果。
2. 两个隔离 worker 各尝试一次，验证不同 ROS domain、命名空间、配置和日志目录；再验证一次中断清理。不得出现跨 worker 串数据或取消其他业务进程。
3. 若有可用真实 bundle，执行一次有时间上界的策略闭环，检查 binding、服务、动作、反馈和收尾链路。物理任务失败可与基线一致，必须单独报告，不能用 Teacher 或真值补救。
4. 对一个基线已能读取的历史 episode 执行只读导出/审计；记录 IDL 变化对旧数据的实际兼容方式。

环境缺失、预算不足或资源被占用时，继续可独立完成的代码和静态验证，明确列出尚未执行的必需验证及复现命令。不能把缺失环境等同于测试通过，也不能把代码和文档已写完等同于全部交付完成。

不运行完整训练、不补采正式 50 条、不重复完整评价集来验证目录迁移。若基线某环节已经失败，记录失败原因和迁移前后差异，禁止靠放宽门禁获得绿色结果。

## 12. 最终交付与验收清单

### 12.1 必交文件和记录

- `shared/` 目标结构及各实际维护目录的简明 README。
- `panda_stack/binding.yaml` 与真实方案说明。
- `_template/README.md` 与 `binding.example.yaml`。
- 方案模型/解析工具、各端入口接入、SDK/部署声明及相关测试。
- 更新后的根 README、`docs/README.md`、`docs/standards.md`、各端使用说明。
- `docs/architecture-tasks/shared-scenarios-implementation.md`：记录基线、文件归属表、阶段状态、最终接口变更、验证命令/结果、历史兼容策略和未完成项。

### 12.2 验收清单

- [ ] 原有用户改动和保护对象保留，没有自动提交、推送或覆盖旧产物。
- [ ] 根公共目录完成归并，四端实现所有权不变。
- [ ] common 生产代码与配置没有 Panda、stack、ACT、MuJoCo 版本的业务选择分支；有界协议或可选能力的通用名称不算业务分支。
- [ ] 新方案通过显式引用选中配置，各端实际消费，公共代码无需注册新方案名。
- [ ] 方案和控制参数只有一个权威来源，模型真实性仍由 bundle 校验。
- [ ] 核心 SDK、可选公共包、方案扩展包可区分构建与校验。
- [ ] 专用 Teacher 不再成为公共控制依赖；反馈数量与曲线来源处理满足第 8 节。
- [ ] 版本不兼容被明确拒绝，历史文件未被改写，兼容性验证有证据或明确缺口。
- [ ] 通用部署规则与具体实现声明分离，没有删除约束来绕过检查。
- [ ] 模板说明换任务、机器人、后端、算法、checkpoint 和新能力分别改哪里。
- [ ] 活跃源码/部署/操作文档无旧入口依赖；历史记录和旧产物中的旧路径按历史语义保留。
- [ ] `contracts/`、`libraries/`、`infrastructure/`、`profiles/` 不再作为运行源码入口；确认全部引用迁移后才删除空目录说明，不保留软链接或转发脚本。
- [ ] 静态、单元、SDK、集成和实际闭环结果分开记录；没有以单种子或 Fake 结果宣称泛化。
- [ ] 所需验证全部通过才标记任务完成；否则明确交付阶段和剩余工作。

## 13. 实施方式

默认顺序为 S0 → S1 → S2 → S3 → S4 → S5。用户可以选择一次交付全部阶段，也可以按配套提示词逐阶段交付；阶段交付不隐含下一阶段已完成。

按本任务书实施时，新架构要求取代旧规范中冲突的目录与路径规定；功能、安全、数据完整性和实验纪律继续有效。更新规范以描述最后的真实实现，不能把本文的未来状态直接写成已完成。

发生例行实现选择时按本文目标决定并记录，不为命名、文件移动、可逆修复反复请求确认。遇到实际无法兼容的旧产物、不可隔离的活动业务、或必须改变本文明确保护的业务语义时，先完成不受影响的工作，指出具体冲突和需要用户决策的内容。
