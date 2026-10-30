# 仓库目录与组织规范

更新日期：2026-10-30。本文描述目录归属和候选扩展结构，与 [接口规范](architecture-and-interfaces.md) 配套。当前阶段 1～2 的实际入口见 [使用指南](stage2-quickstart.md)，本轮维护调整见 [审查记录](maintainability-review.md)。ROS 与 ML 项目独立构建，置于同一仓库。

当前业务实现集中在 components/runtime；mock ROS 后端位于 tests/doubles，纯 Python mock 控制核心位于 libraries/backend_core。尚未创建的组件、assets 和 third_party 按实际需求建立。根 tests/unit 与 tests/integration 是当前测试入口，不要求先迁移到组件内部。

## 1. 顶层分类

| 目录 | 归属内容 | 边界 |
| --- | --- | --- |
| `contracts/` | ROS 接口、Protobuf、JSON Schema、协议兼容规则 | 公共格式与语义定义的唯一来源 |
| `components/` | 四类环境对应的业务代码 | 各组件独立声明依赖，不互相 import 实现 |
| `libraries/` | 有明确用途的共享实现，例如 backend_core | 不作为通用 utils 收纳处；每个库独立声明依赖 |
| `profiles/` | 机器人、观测、动作的版本化语义实例 | 不包含设备 IP、容器地址、硬件序列号或秘密信息 |
| `configs/` | 采集、推理、转换、训练与后端适配的可复用配方 | 引用 profile 和资源 ID，不绑定具体开发电脑 |
| `assets/` | 机器人描述、几何、场景和材质 | 被动资源；驱动、复位和任务逻辑属于 components |
| `deploy/` | Dockerfile、entrypoint、Compose、部署示例与机器绑定 | 负责如何运行代码；不承载采集/控制算法 |
| `tools/` | 构建、契约生成、配置检查和启动辅助命令 | 薄入口，业务规则仍归对应组件 |
| `tests/` | 当前集中管理 unit、integration 和 doubles | 范围由子目录区分；组件独立发布时再评估是否迁移单测 |
| `third_party/` | 外部源码清单、版本锁定、许可与补丁 | 外部 checkout 单独存放，不混入本项目代码 |
| `docs/` | 当前设计、使用说明、决策与历史材料 | 不作为协议源码或安装依赖的来源 |
| `var/` | 数据、权重、标定结果、运行记录和缓存 | 默认不进入 Git，可整体映射到外部磁盘 |
| `.build/` | ROS 构建输出、生成 SDK、wheel、测试结果 | 可删除后重建，不保存唯一的代码或实验成果 |

`components/` 对应职责，`deploy/` 对应部署。一个镜像可以运行多个节点，同一组件也可用不同命令启动多个容器；不按每个容器复制一套源码。

## 2. 目标目录树

以下是候选扩展结构，不代表所有文件已生成，也不是必须创建的目录清单。runtime 当前采用平铺 Python 模块，优先在原包内分离职责；只有规模与实际复用需要时才增加下面列出的子包：

```text
sim_dual_arm/
├── README.md
├── .gitignore
├── .dockerignore                         # 首次实现 Docker 时增加
├── contracts/
│   ├── ros/robot_api_msgs/                # ROS 接口包
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   ├── msg/
│   │   ├── srv/
│   │   └── action/
│   ├── policy/v1/policy.proto            # 已有协议草案的唯一源码
│   ├── python/src/robot_contracts/        # 轻量类型、语义配置校验与哈希
│   └── schemas/v1/                       # RobotProfile / 观测 / 动作 / manifest
├── components/
│   ├── backend_real/
│   │   ├── ros/dual_arm_backend_real/     # 原生 ROS 包；含适配器与 launch
│   │   ├── requirements.in               # 需要额外 pip 依赖时才增加
│   │   ├── locks/
│   │   └── tests/
│   ├── backend_sim/
│   │   ├── ros/dual_arm_backend_sim/      # 公共 API、场景编排、复位与任务逻辑
│   │   ├── locks/
│   │   └── tests/
│   ├── runtime/
│   │   ├── ros/dual_arm_runtime/
│   │   │   ├── package.xml
│   │   │   ├── launch/
│   │   │   └── dual_arm_runtime/
│   │   │       ├── session/
│   │   │       ├── observation/
│   │   │       ├── execution/
│   │   │       ├── recording/
│   │   │       ├── policy_client/
│   │   │       └── input_sources/
│   │   ├── requirements.in
│   │   ├── locks/
│   │   └── tests/
│   └── learning/
│       ├── pyproject.toml
│       ├── uv.lock                       # 采用 uv 时生成
│       ├── src/dual_arm_learning/
│       │   ├── datasets/
│       │   ├── training/
│       │   ├── serving/
│       │   ├── adapters/lerobot/
│       │   └── processing/               # 训练/推理共用处理器
│       └── tests/
├── libraries/
│   ├── backend_core/src/robot_backend_core/ # 无 ROS 的控制状态与插值核心
│   └── ros_support/src/robot_ros_support/   # ROS 侧共用的显式协议转换
├── profiles/
│   ├── robots/<profile_id>/profile.yaml
│   ├── observations/<schema_id>.yaml
│   └── actions/<schema_id>.yaml
├── configs/
│   ├── runtime/{collect,infer,teleop}.yaml
│   ├── learning/{convert,train,serve}/
│   └── backends/{real,sim}/
├── assets/
│   ├── robots/<model_id>/description/    # ROS 描述包，含 URDF/xacro、meshes
│   └── scenes/<scene_id>/                # world.sdf、物体、材质、来源和许可
├── deploy/
│   ├── docker/
│   │   ├── ros-base/Dockerfile           # 三个 ROS 环境的公共基础层
│   │   ├── backend-real/Dockerfile
│   │   ├── backend-sim/Dockerfile
│   │   ├── runtime/Dockerfile
│   │   ├── learning/Dockerfile
│   │   └── entrypoints/
│   ├── compose/{base,sim,real}.yaml
│   ├── examples/sim_collect.yaml        # 已有部署绑定示例
│   └── local/                           # 当前机器的配置，不进入 Git
├── tools/
│   ├── contracts/
│   ├── build/
│   └── run/
├── tests/
│   ├── contracts/
│   ├── backend_conformance/
│   ├── integration/
│   ├── e2e/
│   └── fixtures/                        # 只保留小型、可公开的样例
├── third_party/
│   ├── manifests/
│   ├── patches/
│   └── src/                             # 外部 checkout，不进入 Git
├── docs/
│   ├── README.md
│   ├── repository-layout.md
│   ├── architecture-and-interfaces.md
│   ├── docker-dependency-analysis.md
│   ├── simulation-design.md
│   └── archive/
├── var/                                 # 持久化工作数据，不进入 Git
└── .build/                              # 可重建产物，不进入 Git
```

ROS 包内部再采用其构建类型要求的 CMake 或 Python 文件。目录中的 `ros/` 表示 ROS 项目边界，包名以 package.xml 为准；learning 是独立 Python 项目，不加入 ROS 工作空间。

## 3. 公共代码与依赖方向

- backend-real 和 backend-sim 依赖 `robot_api_msgs`、语义 profile；共同的 lease、上下文校验和事件规则在第二个后端实现时根据实际需求提取。
- 当前 backend_core 仅供 mock 使用，包含 mock 状态和插值；它是纯 Python 包，不包含厂商 SDK、Gazebo 插件或模型框架。不要把整个 mock 执行器当作未来真实控制器。runtime 不依赖其执行实现。
- runtime 只依赖公共 ROS 契约、Policy SDK 和自身应用逻辑；learning 只依赖 Policy SDK、数据 schema 与模型库。
- `contracts/` 包含定义、生成规则与必要的轻量 SDK 包装，不放业务仲裁、伺服控制或模型预处理。

新增共享库需要实际复用或独立测试的理由，不提前建立庞大的 common/utils 层。新增库以清晰用途命名；不得通过增加共享库把 ROS、驱动和 ML 依赖重新混在一起。

## 4. 配置分类：结构、语义、运行配方、机器绑定

| 层次 | 目录 | 例子 |
| --- | --- | --- |
| 结构定义 | `contracts/schemas/v1/` | 一个 RobotProfile 必须有哪些字段、类型和必填项 |
| 语义实例 | `profiles/` | 左右臂关节顺序、rad/m、相机角色、坐标系、动作空间和已验证约束 |
| 功能配方 | `configs/` | 观测同步规则、采集话题、训练参数、模型服务配置、某类驱动/仿真映射 |
| 部署绑定 | `deploy/examples/`、`deploy/local/` | 选用哪些 profile/配方、sim 或 real、IP/端口、namespace、GPU 和挂载位置 |
| 运行快照 | `var/runs/<run_id>/` | 本次实际解析出的配置、版本/digest、日志与指标 |

同一个参数只有一个权威来源。例如关节单位属于 profile；USB 序列号属于真实机器绑定；模型归一化属于模型包/learning processing；模型服务地址属于部署绑定。

当前加载路径为 configs/runtime/mock_collect.yaml → 引用的 profiles → schema 和语义校验 → manifest 中的 configuration 快照。deploy/examples/sim_collect.yaml 没有执行解析器，不能通过修改它改变当前演示。

需要多机器部署时，再实现候选解析顺序：部署绑定选择 profile 与功能配方 → 应用机器级覆盖 → 应用允许的 CLI 覆盖 → schema 校验和语义兼容检查 → 保存 resolved 快照。覆盖只能作用于允许的参数；不能用本地覆盖静默改变 profile 中的关节单位或输入输出语义，语义变化应形成新版本。

profile 保存标定要求或引用，不把每台设备新生成的标定结果堆进 Git。实际标定保存在 `var/artifacts/calibrations/`，按 ID/digest 引用并随原始数据记录。

环境变量主要用于挂载路径、设备访问、地址与部署选择；完整训练超参数和机器人的语义配置使用结构化文件，不全部塞进 `.env`。

## 5. ROS 工作空间、Python 项目与生成代码

每个 ROS 环境明确选择源码发现根，复用仓库中的原始 package 路径，不在几个环境里复制相同的 `src/`：

| 构建目标 | 主要源码来源 |
| --- | --- |
| 当前 mock | colcon：contracts/ros、tests/doubles/ros；pip：contracts/python、libraries/backend_core、libraries/ros_support |
| 当前 runtime | colcon：contracts/ros、components/runtime/ros；pip：contracts/python、libraries/ros_support、生成的 Policy SDK |
| 当前 learning:protocol | 只生成、安装并导入 Policy SDK；没有 learning 业务包 |
| 后续 backend-real / backend-sim | contracts/ros、对应 components 下的 ROS 包、选定描述/驱动包；按需单独安装 Python 库 |
| 后续 learning | components/learning Python 项目，以及单独安装的轻量 Policy SDK |

colcon 支持显式 `--base-paths`，构建/安装输出也可指定到独立目录。当前 Docker 使用 /tmp/colcon-build、/tmp/colcon-log 和 /opt/project/install；增加宿主机开发构建时使用 `.build/ros/<target>/{build,install,log}/`。避免从仓库根默认扫描所有组件、外部示例和 ML 项目。[colcon 源码发现](https://colcon.readthedocs.io/en/released/reference/discovery-arguments.html)、[构建目录参数](https://colcon.readthedocs.io/en/released/reference/verb/build.html)

ROS SDK 从 `.msg/.srv/.action` 构建生成，Policy SDK 从唯一 `.proto` 构建生成。生成文件/wheel 放 `.build/generated/`、`.build/wheels/` 或镜像构建阶段，不手工编辑和提交到各组件。镜像构建直接使用契约源码，不依赖开发电脑残留的 `.build/`。

learning 的 Python 依赖由自身 pyproject.toml 声明，采用 uv 时维护自己的 uv.lock；ROS 包的系统依赖由 package.xml/rosdep 管理。ROS 环境额外需要的 pip 依赖放在所属组件的 requirements.in，并生成带平台信息的锁文件。根目录不建立把 ROS 与 ML 混合求解的全仓库锁文件。

## 6. Docker 与 Compose 的归属

Dockerfile 统一放在 `deploy/docker/`，构建上下文使用仓库根，按需 COPY 公共契约与当前组件。先安装依赖，再复制频繁修改的业务代码；开发源码挂载与可发布镜像分开处理。

entrypoint 只负责 source 环境、路径准备和 exec 进程；episode、复位、控制权等逻辑放组件代码。tools 中的启动脚本负责容器组合与产物导出，不能替代 Runtime API 实现控制流程。当前主入口调用 Docker CLI，不要求 Compose。

当前 compose/mock.yaml 是可选定义，尚未在本机验证。未来确有多种部署组合时，可采用基础文件加后端 overlay；不同时选择两个后端，离线训练不要求启动 ROS。避免为了目标目录同时维护两套功能不同的启动配置。

`deploy/local/` 保存当前机器的真实设备参数、路径和非公开配置。`.env.example` 或公开部署示例只写占位值；私有配置不写进镜像，也不放在 profiles 中。

## 7. 数据、模型、运行记录与缓存

```text
var/
├── data/
│   ├── raw/<real|simulation>/<robot_id>/<episode_id>/
│   │   ├── recording/                   # rosbag2/MCAP
│   │   └── manifest.json
│   └── datasets/<dataset_id>/<version>/
├── artifacts/
│   ├── models/<model_id>/<bundle_digest>/
│   └── calibrations/<robot_id>/<calibration_id>/
├── runs/<run_id>/
│   ├── resolved.yaml
│   ├── versions.json
│   ├── logs/
│   ├── metrics/
│   └── checkpoints/                     # 训练过程状态，导出后形成完整模型包
└── cache/                               # Hub、包下载、仿真资源等可重建缓存
```

数据、模型包与标定结果是需要保留的产物；缓存与 `.build/` 可重建。二者不能因为都不进 Git 就混在同一类清理目录中。`var/` 可由配置映射到外部存储，数据迁移不要求修改代码。

容器内使用稳定路径，例如 `/data/raw`、`/data/datasets`、`/artifacts/models`、`/artifacts/calibrations`、`/runs`；宿主机目录由 deploy 绑定。原始数据/数据集可只读挂载给 learning，各输出和缓存单独挂载。

run_id 组织一次启动/实验，episode_id 组织一段交互记录；二者在 manifest 中关联。数据与模型的版本不靠文件夹名字猜测，manifest 保存 profile/schema、代码/依赖、转换器、输入产物和配置 digest。临时写入、完整校验与提交规则沿用接口规范。

## 8. 资源、第三方、测试和文档

机器人描述与几何放 `assets/robots/`，sim 与 real 共用同一几何来源；仿真插件、驱动配置和场景编排仍归组件/配方。大型下载资源通过来源、版本和哈希管理，必要时采用独立存储或 LFS，避免把缓存当作项目资源提交。

third_party 提交外部来源/固定版本、许可和有需要的补丁；checkout 放被忽略的 src。框架使用发布版依赖时无需复制一份源码；项目自有适配代码始终放对应组件。

当前测试集中在根目录：tests/unit 验证纯 Python 模块；tests/integration 验证原生 MCAP 并提供 ROS 后端探针；tests/doubles 保存 mock 后端。后续按实际需要增加契约/后端一致性/e2e 测试，不要求先创建空目录。fixtures 只放小型公开或合成数据，大记录进入 var。

docs/README.md 是文档索引，当前职责、接口、依赖与目录规范相互链接。旧方案集中说明为历史材料；现有 docs/contracts 的旧路径只保留迁移提示，不继续维护另一份协议。

命名约定：镜像/Compose 服务使用 `backend-real`、`backend-sim`；代码目录和 Python/ROS 名使用下划线，如 `backend_real`、`dual_arm_runtime`。协议主版本出现在路径/命名空间（v1），可变地址和设备编号不编码进源码目录。

## 9. 常见修改应落在哪里

| 修改 | 主要位置 |
| --- | --- |
| 更换机械臂厂商 | backend_real 适配器、相应 profile/描述资源/后端配方与本地绑定 |
| 更换仿真器 | backend_sim 实现、资源与仿真配方；保持 Robot API，重新验证后端一致性 |
| 增加相机 | backend 接入、profile/观测配置与相应标定；模型输入变化时更新 schema 和模型包 |
| 更换学习框架或策略 | learning adapters/processing、学习配方与依赖；超出既有动作契约时显式升级接口 |
| 更换电脑或远程推理主机 | deploy 绑定及必要的 GPU/依赖锁文件 |
| 修改接口字段 | contracts 源定义、生成 SDK、兼容检查与受影响调用方 |
| 调整采集/训练超参数 | configs；本次有效值保存到 runs/resolved 快照 |

阶段 1～2 已加入实际需要的契约、schema、纯 Python 库、ROS 节点、构建镜像、启动入口与测试；mock profile 使用 JSON，运行配方使用 YAML。runtime 当前按 configuration、observation、node、recording、audit 模块划分，规模扩大后再拆相应子包。其余目录按后续功能逐步创建，不把目标目录树等同于已实现范围。
