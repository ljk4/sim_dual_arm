# `sim_dual_arm` 仓库规则（RULES）

本文件是仓库的最高实施准则。所有架构、代码、配置、脚本、产物与文档改动必须遵守以下规则。

---

## 1. 核心工程原则

### 1.1 个人自用、零历史包袱
- **MUST**：协议、配置、接口或目录演进时，直接修改当前实现并同步更新所有调用方与测试。
- **MUST NOT**：保留旧版兼容分支（`legacy_*`）、迁移框架、废弃字段别名、re-export shims 或旧产物回退逻辑。
- **`build/` 与 `var/`**：视为可随时重建的运行/构建产物，不做跨版本兼容。

### 1.2 禁止过度设计与过度防御（Anti-HERO）
- **No Useless Hashing（禁止无用哈希/留痕）**：不为“证明跑过”生成无人消费的清单或摘要文件。仅当哈希/校验能跳过更昂贵的重复计算，或用于跨进程契约握手（如 Profile / Bundle 一致性）时才允许存在。
- **No Unnecessary Indirection（禁止多余中间层）**：
  - 禁止离线生成 `/data/prepared/` 等中间配置目录；三端统一在启动时通过 `--scenario <id>` 或 `--config <path>` 在内存中解析配方，并支持 `--set dot.path=value` 覆盖。
  - 禁止在同一目录树内复制同一份配置/JSON（`shutil.copy2`）；每份元数据只保留唯一权威文件。
  - 禁止编写纯透传包装类、空壳基类、未使用的插件注册表或 feature flag。
- **No Fake Edge-Case Scaffolding（禁止假想敌兜底）**：单人本地使用，不写多租户权限、防篡改锁或不可达冷门异常兜底；同一调用链内已通过 Pydantic 校验的数据，禁止重复手写 `isinstance` 二次检查。
- **SSOT（单一可信源）**：同一参数禁止在多份配置中重复维护。

### 1.3 架构隔离与端内对称化
- **通用端 100% 方案无关**：`shared/common/`、`runtime/ros2/`、`learning/lerobot/` 的业务代码与默认配置中，**严禁硬编码 `panda_stack`、`marvin_handover` 等方案/机器人/任务名**，也严禁隐式回退到某一默认方案。
- **方案配置集中存放**：具体方案只存在于 `shared/scenarios/<scenario-id>/`（`binding.yaml`、`profiles/`、`configs/`、`extensions/`）。
- **端内插件平级对称（Symmetry）**：在 `simulation/mujoco/` 等包含具体机器人/任务的端内，通用机制（`control/`、`motion/`、`follower/`、`render/`、`ros/`、`runtime/`、`kinematics/common.py`）必须与具体插件彻底分离；不同机器人/任务必须平级对称存放（如 `physics/panda/` 与 `physics/marvin/`、`tasks/stack/` 与 `tasks/handover/`），严禁把先接入的机器人写在顶层公用目录。

---

## 2. 容器环境与脚本规范

### 2.1 容器入口 `env.sh`
- 仓库根目录 `env.sh` 是管理所有 Docker 环境的唯一宿主机入口（`up`、`enter`、`stop`、`rm`、`ps`、`viewer`）。
- **新增容器环境时**：必须同步在 `env.sh` 的 `resolve_env` 与帮助信息中注册环境名、Compose 路径与服务名。
- **单端启停边界**：所有端共享 Compose 项目名 `sim-dual-arm`。清理单端必须使用 `stop` + `rm`（或 `./env.sh stop <env>` / `./env.sh rm <env>`），**严禁对单端执行 `docker compose down --remove-orphans`**。

### 2.2 用户脚本设计与输出规范
- **单脚本单领域**：每个用户脚本（`scripts/*.sh`）只负责单一明确领域的任务，不把无关子命令混在一起。
- **运行后展示配置与有效参数**：业务脚本启动时必须清晰打印实际使用的配置文件路径（如 Binding、配方路径、输出目录）与展开后的有效参数。
- **长时任务用非刷屏进度条**：采集、转换、训练、批量推理等长时任务在 TTY 下必须使用原地刷新的非刷屏进度条；非 TTY 下仅输出关键阶段与最终摘要。
- **高信噪比、易于理解**：屏蔽底层冗余过程事件（明细写入日志文件），终端只输出对操作与结果判断有直接价值、易于人类阅读的信息。

---

## 3. 目录与产物隔离规范

### 3.1 `var/`（容器 `/data`）四分法（强制隔离）
所有运行期产物必须按 `<scenario-id>` 归入 `var/<scenario-id>/`（容器内 `/data/<scenario-id>/`），并严格分为以下四个平级目录，**绝对禁止混放或私自在 `var/` 根下新建目录**：

| 目录 | 允许存放 | 严禁混入 |
| :--- | :--- | :--- |
| `datasets/` | 经筛选转换后供训练/验证的数据集（LeRobot 分片、manifest、采样掩码、`formal-select.json`） | 原始 MCAP、运行日志、试采/诊断中间文件、模型权重 |
| `models/` | 训练权重/checkpoints、训练预算台账、导出可部署模型包（`bundles/`） | 原始录制数据、训练数据集、运行日志、评估图 |
| `logs/` | 三端运行事件日志、采集台账（`ledger.jsonl`）以及**运行时录制内容**（`raw/` MCAP、`manifest.json`、`task_result.json`、随录帧/标签） | 正式 LeRobot 数据集、模型权重/Bundle、离线诊断报告 |
| `diagnostics/` | 环境探针、场景/运动学检查、离线评估报告、集成/冒烟测试临时产物 | 正式采集录制、正式训练数据集、正式模型包 |

> 无对应 `<scenario-id>` 的系统级临时检查统一放入 `var/_system/diagnostics/`。
> **新产物准入铁律**：新增功能若产生新的文件/目录类型，**必须先向用户说明其性质与消费者，经确认归属后再实施**。

### 3.2 `build/`（容器 `/build`）精简规范
`build/` 仅存放运行时直接加载的生成 SDK，禁止存放评估报告、缓存或临时编译目录：
- `build/ros-dev/`：仅保留 `install/` 与 `contracts.json`；`colcon` 的 `build/`、`log/`、`COLCON_HOME` 必须在 `/tmp` 执行并在结束后清理。
- `build/generated/policy-sdk/`：仅保留 `dual_arm_policy/` 与 `source.sha256`。

---

## 4. Agent 交付与行为准则
- **先判断再改动，不写无用校验**：能直接判断的逻辑不堆砌冗余检查；只修范围内与受支持用法可达的问题。
- **直接呈现最终正确状态**：代码、注释与文档中不保留试错痕迹、废弃方案说明或免责套话；如有未验证项或已知限制，在回复末尾集中说明。
- **遇到阻塞或架构边界变更**：涉及新增产物类型、跨端接口变更或无法验证时，先说明清楚并向用户确认，再继续推进。
