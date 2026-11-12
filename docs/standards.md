# 项目规范

本规范定义 `sim_dual_arm` 仓库的目录组织、架构边界、接口契约、代码质量、配置管理、日志记录、容器环境与验收准则。操作流程见[根 README](../README.md)，当前进度见[项目状态](project-status.md)。

---

## 1. 文档体系与职责边界

```text
README.md                    项目概览、环境构建与完整闭环操作指南
docs/
├── README.md                文档索引与阅读导航
├── project-status.md        总体目标、技术约束、当前状态与产物定位
├── standards.md             工程规范（本文件）
├── features/                功能说明（一个功能一篇，描述当前实现）
├── experiments/             重要实验记录与故障排查经验（YYYY-MM-DD-<topic>.md）
├── history/                 阶段演进摘要与 DSH 关键结论索引
└── architecture-tasks/      架构改进任务书与执行验收记录
<各维护目录>/README.md        该目录的职责、核心入口、模块列表与依赖边界
```

- **唯一数据源（SSOT）**：代码、契约（Schema/IDL/Proto）与配置文件决定真实系统行为，文档仅作说明与索引，不作为运行时配置加载来源。
- **不维护冗余流水**：根 `README.md` 面向使用者保持简洁可操作；当前阶段状态唯一维护在 `project-status.md`；历史决策浓缩于 `history/README.md`。
- **局部目录说明**：项目自维护目录均提供精炼的 `README.md`，说明职责、入口与边界；`.git`、`.build`、`var`、缓存目录与 `third_party/` 上游源码目录不额外添加项目文档。

---

## 2. 仓库目录结构与数据归类

仓库按运行端与具体实现分层，隔离仿真引擎、运行中间件与机器学习框架：

| 目录 | 职责内容 | 依赖与隔离边界 |
| --- | --- | --- |
| `simulation/<engine>/` | 物理仿真实现（`src/`、`configs/`、`docker/`、`scripts/`、`assets/`） | 当前为 `mujoco/`；不预建其他引擎空目录，不导入 `runtime` 或 `learning` |
| `hardware/` | 真机统一 Robot API 边界说明与后续驱动 | 当前仅保留边界说明；不构建假驱动或占位空模块 |
| `runtime/<middleware>/` | 运行端实现（`ros/`、`configs/`、`docker/`、`scripts/`） | 当前为 `ros2/`；不导入仿真物理实现，不执行模型训练 |
| `learning/<framework>/` | 学习与推理实现（`src/`、`configs/`、`docker/`、`scripts/`） | 当前为 `lerobot/`；严格保持 ROS-free，不导入 ROS 或 `runtime` |
| `shared/common/tools/` | 公共容器配置（`docker/`）与构建/SDK/检查脚本（`scripts/`） | 仅放跨端基础设施，不包含业务逻辑、场景参数或训练算法 |
| `shared/common/python/` | 具有明确调用方的跨端共享库（`shared/common/python/contracts/`、`ros_support/`、`run_logging/`、`trajectory/`） | 不持有运行生命周期，不建立泛化的杂项 `utils` |
| `shared/common/python/trajectory/` | 共享轨迹求值、跟随规划数学和数值验证 | 依赖 `robot_contracts`，保持 ROS-free；契约不得反向依赖本库 |
| `shared/common/interfaces/` | 公共接口约束：Protobuf、ROS 2 IDL、JSON Schema v1 与轻量 Python 契约包 | 跨端数据结构与语义校验的唯一定义源，不依赖 ROS、MuJoCo 或 PyTorch；不写具体机器人数值 |
| `shared/scenarios/<id>/` | 方案绑定：`binding.yaml` 选择档案、四端配方与方案专用接口 | 只做选择，不持有实现；当前实例为 `panda_stack`，`_template` 说明替换，`panda_stack_diag` 仅用于诊断 |
| `shared/scenarios/panda_stack/profiles/` | 机器人语义档案、观测/动作绑定与共享控制配置（`control/`） | 当前 Panda 实例的权威数值来源；严禁写入主机路径、IP、账号或设备序列号 |
| `tests/` | 单元测试（`unit/`）、集成测试（`integration/`）、测试替身（`fakes/`）与夹具（`fixtures/`） | 测试替身与合成数据均标记 `test_only`，严禁冒充正式实验数据 |
| `third_party/` | 固定的外部上游源码（如 `lerobot-act-v0.6.1`） | 保持上游源码与许可证原样；Panda 资产直接置于 `simulation/mujoco/assets/` |
| `var/` | 运行期数据、数据集、模型包、日志与台账（不纳入 Git） | 按 `run-id`、`episode-id` 与实验目录严格隔离，禁止覆盖 |
| `.build/` | 生成的 SDK、ROS 工作空间、构建缓存与只读权重缓存（不纳入 Git） | 清理前须核对挂载与复现依赖 |

公共接口约束（`shared/common/interfaces/` 与 `shared/common/python/`）描述所有机器人、后端与算法都必须满足的协议；当前 Panda/MuJoCo/ACT 实例的关节顺序、相机、限位与训练数值属于 `shared/scenarios/panda_stack/`，不得写回公共协议。四端只实现自己的能力，方案通过 `binding.yaml` 选择已有实现，不在公共生产包中按方案名分支。

---

## 3. 架构解耦与状态所有权

1. **后端物理所有权（`simulation`）**：独占 MuJoCo `MjModel`/`MjData`、物理步进、Ruckig 规划、EGL 渲染与接触检测。控制、渲染与状态查询严格遵守线程所有权，跨线程通过只读快照或有界邮箱通信。任务逻辑通过通用 `SimTaskPlugin` 接口（如 `tasks/stack/`）解耦，核心评估器不直接依赖 ROS 消息。
2. **运行端会话管理（`runtime`）**：负责多路相机与关节状态同步、控制租约（Lease）管理、流式动作块接续、MCAP 录制与离线审计。通过 ROS 2 `robot_api_msgs` 与后端通信；仿真复位与通用任务生命周期使用可选的 `sim_api_msgs`，方案专用 Teacher 类型由 binding 选择的 `interfaces/extension_ros` 提供并按能力标识延迟加载，通过 gRPC `PolicyService` 与策略服务通信。
3. **无 ROS 学习端（`learning`）**：负责将运行端导出的合格数据转换为 LeRobot 数据集、执行 ACT 训练、导出 safetensors 模型包并提供 gRPC 策略推理服务。推理流水线（会话锁、RGB 解码、夹爪准入、绝对时间融合）收口于 `ChunkedPolicyPipeline`。
4. **示教与评估隔离**：程序示教（Teacher）与物体真值仅限数据采集阶段使用。正式 ACT 策略评估严禁读取物体真值、调用 Teacher/IK 补救或用脚本替换模型输出。

---

运行端命令与离线处理位于 `dual_arm_runtime/offline`；`offline/mcap_reader` 解码消息，`offline/audit` 仅接收纯数据。`offline/episode_finalization` 保持在线收尾编排。仿真按 `config/control/motion/follower/kinematics/physics/render/runtime/ros/tasks/tools` 分包，只有 ROS 适配层、任务转换器与 viewer 可加载 ROS。

## 4. 接口、动作语义、时钟与反馈

- **关节与动作语义**：以 `shared/scenarios/panda_stack/profiles/` 为准。当前双 Panda 为 16 维位置控制：左臂 7 关节（rad）、左夹爪总开度（m）、右臂 7 关节（rad）、右夹爪总开度（m）。
- **相机语义**：固定为 `global`、`left_wrist`、`right_wrist` 三路 320×240 RGB。标定摘要（Semantic Hash）与 Profile、数据集、模型包严格绑定。
- **观测不可变性**：`ObservationBuffer` 分配观测 ID 后立即冻结关节状态与三路图像引用，迟到帧不得篡改已发布的观测内容。
- **双时钟分离**：RPC 超时、租约续期与资源监督使用单调墙钟（Wall Clock）；轨迹规划、动作块对齐、时间融合与任务计时使用仿真时间（Simulation Clock）。
- **控制生命周期与幂等性**：`SubmitFollower` 的接收确认（ACK）不等于执行完成。按 `run_id`、`episode_id`、`command_id` 与 `chunk_index` 追踪 `STARTED`、实时反馈与终态（`COMPLETED` / `ABORTED` / `PREEMPTED` / `REJECTED`）。已冻结的终态不被迟到消息覆盖。
- **多层动作留痕**：ACT 原始预测（`raw_act`）、准入目标（`admitted`）、时间融合传输目标（`execution_targets`）、分段三次样条参考（`FollowerReference`）与实测反馈（`ExecutionFeedback`）分别独立记录，禁止用期望参考值冒充实测反馈。

---

## 5. 代码质量与模块设计

- **代码风格与静态检查**：遵循 Python 3.12 标准与根目录 `pyproject.toml` 中的 Ruff 规则（行宽 100，启用 `E`/`F`/`I`/`B`/`UP`）。字段名与变量名显式标明物理单位（如 `_rad`、`_m`、`_ns`、`_s`）。
- **模块与函数规模**：按单一职责与状态所有权拆分模块，显式组合优于继承（禁止使用 `*Mixin` 拆分核心控制器）。单个 `.py` 文件不超过 720 行，核心业务函数不超过 85 行。
- **单层声明式数据模型**：公开的配置与契约 DTO 直接继承 Pydantic v2 `StrictContractModel`（`extra="forbid"`、`frozen=True`、`strict=True`），禁止使用私有 Pydantic 模型加 `@dataclass` 双层套壳或冗长的手写类型检查。
- **异常与清理保证**：异常信息须包含组件来源、请求标识与稳定原因码。资源清理（停止运动、释放租约、刷盘关闭日志/MCAP）逐步独立尝试并聚合错误，不因前序清理失败而跳过后续清理，不裸捕异常或静默返回默认值。

---

## 6. 配置管理与单一数据源（SSOT）

- **Profile 格式**：Robot Profile 仅支持 `schema_version=2`，唯一 Schema 为 `shared/common/interfaces/schemas/v1/robot_profile.schema.json`；使用 1～8 路具名 `cameras`，每路必须提供标定摘要。不保留 v1 加载转换。Schema 集合目录版本不代表所有契约内部格式版本。
- **职责分层**：JSON Schema（`shared/common/interfaces/schemas/v1/`）定义结构约束，Profile（`shared/scenarios/panda_stack/profiles/`）定义机器人物理与控制参数，业务 YAML 定义运行配方，`container.env` 定义容器环境路径。
- **控制参数集中维护**：跨 `simulation` 与 `runtime` 共享的控制限值（`panda_control_v1.yaml`）、跟随窗口与偏差预算（`panda_follower_v1.yaml`）以及流式超时（`panda_stream_v1.yaml`）唯一维护在 `shared/scenarios/panda_stack/profiles/control/` 中，不在各端配置中内联复制。
- **运行时动态摘要绑定**：源 YAML 配置中不手写静态文件 SHA-256。准备脚本（`prepare` / `prepare-policy`）在生成运行配置时自动计算模型包、Profile 与 Schema 的真实摘要并写入 `policy_binding.json` 与 `run_manifest.json`。
- **禁止原地覆盖**：所有 `prepare` 输出目录、运行目录与日志目录必须全新创建，已存在时立即报错退出。

---

## 7. 日志、录制与审计

- **结构化运行日志**：统一使用 `robot_run_logging`，记录 `run_id`、`component`、序列号、墙钟时间及上下文 ID。采用有界队列与 JSONL 轮转，日志写入失败或溢出必须显式报告。
- **有界 MCAP 录制**：单写线程配合条数与字节双重有界队列，慢盘不阻塞实时控制回路；大尺寸图像与动作数组按消息或独立产物存储，Manifest 记录 1 MiB 分块 SHA-256 摘要。
- **多维结果独立判定**：进程退出码、运行时状态（`runtime`）、录制完整性（`recording`）、图像质量门禁（`quality`）与物理任务成功（`task`）分别独立报告。物理堆叠成功不代表图像质量合格，录制完整不代表任务成功。

---

## 8. 容器化环境与执行边界

1. **三端独立镜像与统一编排**：
   - 每个实现目录（`simulation/mujoco`、`runtime/ros2`、`learning/lerobot`）恰好维护一份 `docker/Dockerfile`、`docker/docker-compose.yml`、`configs/container.env` 与 `scripts/run.sh`。不维护根目录 Compose 包装或宿主机启动脚本。
   - 三份 Compose 共享项目名 `sim-dual-arm`。`simulation` 接入 `ros` 网络并启用 NVIDIA GPU（EGL 渲染）；`runtime` 接入 `ros` 与 `learning` 网络（纯 CPU）；`learning` 仅接入 `learning` 网络并启用 NVIDIA GPU（无 ROS 挂载与环境变量）。
   - 停止单个服务使用 `docker compose stop <service>` 与 `docker compose rm -f <service>`，不对单端执行 `down --remove-orphans`。
2. **纯粹的依赖镜像**：
   - `Dockerfile` 仅安装固定版本的系统与 Python 依赖（如仿真端通过单次 `pip install --require-hashes -r /opt/dependencies/requirements.lock` 安装含固定 Ruckig wheel 在内的依赖），不 `COPY` 业务源码、配置或启动脚本，不内嵌多行脚本或控制流。
   - 镜像统一设置 `ENTRYPOINT []` 与 `CMD ["sleep", "infinity"]`。容器启动后保持空闲，由用户通过 `docker compose exec <service> bash` 进入容器手动执行命令；运行端示教录制使用 `runtime/ros2/scripts/record.sh --count N`，其余业务使用 `scripts/run.sh <command>`。
   - 并行录制的宿主机入口 `runtime/ros2/scripts/record_parallel.sh` 仅负责编排独立 simulation 容器；预算分配、录制和导出继续在 runtime 容器内执行。不得向 runtime 挂载 Docker socket 或安装物理仿真依赖。
3. **显式挂载与 SDK 校验**：
   - 三端统一将仓库 `shared/` 只读挂载至 `/shared:ro`；`simulation` 仅将自身目录只读挂载至 `/app:ro`，并按需只读挂载 `/opt/ros-dev`。
   - `runtime` 与 `learning` 将仓库根只读挂载至 `/workspace:ro`，将 `var/` 与 `.build/` 可写挂载至 `/data` 和 `/build`。
   - ROS SDK（`/opt/ros-dev`）与 Policy SDK（`/opt/policy-sdk`）由 `runtime` 容器内的 `run.sh build-ros-sdk` / `build-policy-sdk` 显式构建；各端业务命令启动前自动校验 SDK 完整性与源码摘要。

---

## 9. 实验纪律与测试验收

- **测试入口**：容器内统一通过 `scripts/run.sh test`（底层调用 `shared/common/tools/scripts/test.sh`，固定 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`）执行测试，宿主机直接调用业务脚本会明确报错退出（退出码 2）。
- **Dockerfile 与 Compose 边界检查**：修改容器配置后必须运行 `python3 shared/common/tools/scripts/check_dockerfiles.py` 验证合规性。
- **预算与台账不可篡改**：正式采集台账（`attempts.jsonl`）与训练预算台账（`training-budget.jsonl`）仅追加写入，预检、超时与失败尝试均计入分母，严禁删除失败记录或清空台账重计。
- **同场景回归与泛化评估分离**：针对特定失败种子的修复回归使用独立实验名与台账，不得并入独立新种子的泛化成功率统计。
