# `sim_dual_arm` Agent 指导与工程规范（AGENTS.md）

本文件是指导 Agent 在本仓库开展**开发排查**与**规范收敛**的最高准则。常规功能开发、跨模块排错与架构调整采用**“开发分支快速突破 → 规范分支收敛清理 → 合并主分支并删除临时分支”**的双阶段 Git 闭环流程（轻量改动适用 `§1` 快速通道；`§0` 全局红线在任何情况下均强制遵守）。

---

## 0. 全局红线（开发与规范两阶段均强制遵守）

1. **破坏性改动与破例须先请示**：
   - 在排查、开发或规范收敛过程中，若需要做出**破坏性改动**——包括修改跨端契约（Protobuf / ROS 2 IDL / JSON Schema / 配置结构）、改变动作窗口或数据分布导致既有数据集/模型失效、改变核心物理或评价判定语义——或认为某项偏离本规范的做法确有工程意义时，**必须先向用户说明原因、影响范围与预期收益，经用户确认并获得许可后方可实施修改**。
2. **`third_party/` 离线受保护资产边界**：
   - `third_party/`（固定版本 LeRobot 源码 `third_party/src/lerobot-act-v0.6.1/` 与共享离线权重 `third_party/weights/resnet18-f37072fd.pth`）属于长期受保护的宿主机离线资产，容器以只读方式挂载为 `/third_party` 且不会代为恢复。
   - **严禁**删除、覆盖或污染 `third_party/` 下的源码与权重，**严禁**在训练或推理代码中添加自动联网下载兜底逻辑。
3. **Git 写操作与暂存红线**：
   - **严禁** Agent 未经用户明确指令私自执行 `git commit`、`git merge`、`git branch -D`、`git push`、`git reset --hard`、`git clean -fd` 等提交、推送、合并、删分支或破坏性清理工作区操作。
   - 经用户指令执行提交时，必须显式指定文件列表（`git add <files>`），**严禁**盲目使用 `git add -A` 或 `git add .`，避免将临时探针、调试脚本或未跟踪产物混入提交。

---

## 1. 双阶段 Git 闭环工作流（开发 vs. 规范）

Agent 必须严格区分当前处于**开发阶段**还是**规范阶段**，严禁把两阶段的关注点混在一起导致效率低下或规范失守：

- **阶段判定锚点（以当前 Git 分支前缀为唯一真相源）**：
  - 当前处于 `dev/*` 或 `fix/*` 分支 → 自动执行**阶段一（开发阶段）**规则；
  - 当前处于 `norm/*` 或 `clean/*` 分支 → 自动执行**阶段二（规范阶段）**规则；
  - 当前处于 `main` 分支 → 接到常规开发或排错任务时，必须先从 `main` 切出 `dev/<topic>` 或 `fix/<topic>` 分支再动手改代码；若符合下方轻量改动豁免条件，或用户明确指示无须开新分支，则按豁免或用户指令执行。
- **轻量改动豁免（快速通道）**：
  - 纯文档修订（如 `README.md`、`docs/`、`AGENTS.md`）、单文件参数微调（如 YAML 超参调整）、拼写或注释修正等低风险小改动，无需走双分支闭环，可直接单分支或按用户指令直接完成（仍须严守 `§0` 全局红线）。

```text
[最近提交 (main)]
       │
       ▼  1. 新建开发分支 (如 dev/<topic> 或 fix/<topic>)
┌──────────────────────────────────────────────────────────┐
│ 阶段一：开发阶段（修 Bug / 排查问题 / 添加新功能）         │
│  • 最高指标：以最快速度定位根因、跑通功能、完成任务目标     │
│  • 允许动作：可大改大修、快速试错、注入探针，不在形式规范上 │
│    浪费时间（但仍须严守 §0 全局红线）                      │
│  • 方法准则：必须遵循“自底向上、逐环排除”的高效排查/开发步骤 │
│    （参考 docs/experiments/closed-loop-triage.md）          │
└──────────────────────────────────────────────────────────┘
       │
       ▼  2. 汇报结果并请示确认 → 经用户下达指令后在开发分支显式 git add/commit
       │
       ▼  3. 从已保存的开发分支新建规范分支 (如 norm/<topic> 或 clean/<topic>)
┌──────────────────────────────────────────────────────────┐
│ 阶段二：规范阶段（去除冗余 / 架构对齐 / 契约与目录收敛）     │
│  • 核心目标：在不破坏已验证功能的前提下，对照本文 §2～§4     │
│    彻底去除和修正开发阶段遗留的不符合规范之处               │
│  • 必做清理：删除临时探针/备份/无用校验、消除硬编码与多余    │
│    中间层、归正 var/ 与 build/ 产物、按 §3.4～§3.5 验收门禁 │
└──────────────────────────────────────────────────────────┘
       │
       ▼  4. 跑通 §3.5 合入前审查门禁并请示确认 → commit → squash 合并回 main
       │     → 删除开发分支与规范分支（一次闭环修正结束）
```

### 1.1 阶段一：开发阶段（完成任务为最高指标）
1. **分支起点**：开始修 Bug、排查问题或添加新功能时，**先从最近提交新建独立的开发分支**（如 `git checkout -b dev/<topic>` 或 `fix/<topic>`）。
2. **结果导向，允许大改大修**：
   - 开发阶段以**实际解决问题、跑通闭环、达成任务指标**为唯一最高优先级。
   - 允许跨模块大改大修、直接修改核心链路、编写临时诊断脚本或快速实验，不在代码形式规范、抽象约束、目录洁癖或防御性校验上浪费时间，**但仍必须严守 `§0` 全局红线**；临时探针脚本统一放在 `/tmp/` 或 `var/<robot-id>/<task-id>/debug/`（系统级放 `var/_system/debug/`），严禁散落在受版本控制的源码目录中。
3. **遵循快捷高效的排查与开发步骤**（详见 [`docs/experiments/closed-loop-triage.md`](docs/experiments/closed-loop-triage.md)）：
   - **自底向上、从易到难逐环排除**：面对长链路问题（如“采集 → 训练 → 推理 → 仿真”），严禁在末端盲目调参或在错误假设下设计复杂协议；先用最小离线脚本验证底层数据与物理回放，再核对模型离线输入输出，随后隔离推理执行侧窗口与规划，最后对齐仿真物理接触与评价器边界。
   - **最小探针、证据先行**：用几十行聚焦探针直接打印关键物理量/张量/拒绝原因，一锤定音定位真实根因，排除干扰假说后再动手修改。
4. **阶段一收尾节点**：功能跑通或问题修复并完成实测验收后，向用户汇报结果并请示确认；**仅在用户确认完成并明确下达指令后，按指定文件范围在开发分支执行 `git add <files>` 与 `git commit`（禁止私自 commit）**。

### 1.2 阶段二：规范阶段（严格收敛与清理）
1. **分支起点**：从开发阶段已提交保存的分支上，**新建规范分支**（如 `git checkout -b norm/<topic>` 或 `clean/<topic>`）。
2. **全面去芜存菁与规范对齐**：对照本文 `§2～§4` 逐项审查并修正开发分支中的临时改动：
   - 清理开发排查期遗留的临时脚本、调试打印、冗余防御分支、废弃配置字段与未消费产物；
   - 将硬编码参数、方案耦合逻辑、不对称插件结构重构至符合架构隔离与 SSOT 要求；
   - **必须按 `§3.4` 的改动-验证触发清单执行回归测试与最小闭环实测验收**，严禁仅凭语法检查或口头宣称逻辑等价，确保规范化重整未改变已验证通过的任务效果；合入 `main` 前，还须按 `§3.5` 对项目现有全部任务跑通四步全链路审查（符合 `§1` 轻量改动豁免条件者除外）。
3. **阶段二收尾与闭环结束**：向用户展示规范化改动摘要及 `§3.4`、`§3.5` 实测验收结果，**请示用户确认规范达标**；仅在用户确认并下达指令后，按显式文件范围执行 `git commit`，将规范分支以 **`git merge --squash`（或单次干净 commit）方式合并回 `main`**（避免把开发分支的杂乱试错提交带入主分支），确认合入无误后再删除本次的开发分支与规范分支，完成一次完整的闭环修正。

---

## 2. 核心工程规范（规范阶段强制达标）

### 2.1 个人自用、零历史包袱
- **MUST**：协议、配置、接口或目录演进时，直接修改当前实现并同步更新所有调用方与测试。
- **MUST NOT**：保留旧版兼容分支（`legacy_*`）、迁移框架、废弃字段别名、re-export shims 或旧产物回退逻辑。
- **`build/` 与 `var/`**：视为可随时重建的运行/构建产物，不做跨版本兼容。
- **面向用户的文档无历史包袱**：面向用户的说明文档（`README.md`、`docs/`）只直接说明当前应如何配置与使用，不保留历史演进、废弃对比或“不要/无需做某事”等冗余提示；本文件（`AGENTS.md`）作为 Agent 行为约束规范，允许使用明确的禁止条款。

### 2.2 禁止过度设计与过度防御（Anti-HERO）
- **No Useless Hashing（禁止无用哈希/留痕）**：不为“证明跑过”生成无人消费的清单或摘要文件。仅当哈希/校验能跳过更昂贵的重复计算，或用于跨进程契约握手（如 Profile / 模型身份一致性）时才允许存在。
- **No Unnecessary Indirection（禁止多余中间层）**：
  - 禁止离线生成 `/data/prepared/` 等中间配置目录。三端在启动时通过 `--robot <robot-id> --task <task-id>` 或 `--config <path>` 在内存中解析配方。常用运行参数使用专用命令行选项，其余参数在配方中维护。
  - 禁止在同一目录树内复制同一份配置/JSON（`shutil.copy2`）；每份元数据只保留唯一权威文件。
  - 禁止编写纯透传包装类、空壳基类、未使用的插件注册表或 feature flag。
- **No Fake Edge-Case Scaffolding（禁止假想敌兜底）**：单人本地使用，不写多租户权限、防篡改锁或不可达冷门异常兜底；同一调用链内已通过 Pydantic 校验的数据，禁止重复手写 `isinstance` 二次检查。
- **SSOT（单一可信源）**：同一参数禁止在多份配置中重复维护。

### 2.3 架构隔离与端内对称化
- **通用端 100% 方案无关**：`shared/common/`、`runtime/ros2/`、`learning/lerobot/` 的业务代码与默认配置中，**严禁硬编码 `marvin_sim/stack`、`marvin_sim/handover` 等方案/机器人/任务名**，也严禁隐式回退到某一默认方案。
- **方案按两级目录集中存放**：具体方案只存在于 `shared/scenarios/<robot-id>/<task-id>/`（如 `shared/scenarios/marvin_sim/stack/`、`shared/scenarios/marvin_sim/handover/`），内含 `binding.yaml`、`profiles/`、`configs/`、`extensions/`；必须以机器人 id 与任务 id 分两级目录组织，禁止使用单层 `<scenario-id>` 扁平命名。
- **三端统一选择入口**：所有端的 CLI 一律通过 `--robot <robot-id> --task <task-id>` 选择方案（与直接 `--config <recipe>` 互斥）；运行期产物路径也按同一对 `<robot-id>/<task-id>` 展开，确保配置与产物一一对应。
- **端内插件平级对称（Symmetry）**：在 `simulation/mujoco/` 等包含具体机器人/任务的端内，通用机制必须统一收敛到 `robot_backend_sim/common/`（含 `control/`、`motion/`、`follower/`、`render/`、`ros/`、`runtime/`、`kinematics/`、`physics/`、`config/`、`tasks/` 中方案无关的部分），与具体插件彻底分离；具体机器人/任务按场景 id 平级对称存放为 `robot_backend_sim/<robot-id>/<task-id>/`（如 `marvin_sim/stack/`、`marvin_sim/handover/`）。机器人级、任务无关的代码（robot assembly、robot kinematics、robot io、robot_config）直接放在 `<robot-id>/` 下；任务级代码（environment、evaluator、teacher、gate、task config、scene 编排）放在 `<robot-id>/<task-id>/` 下。调度注册表（`common/config/scene_loader.py`、`common/physics/robot_adapters.py`、`common/tasks/registry.py`）只按 assembly/task 名解析场景目录，新增机器人/任务只需新建 `<robot-id>/<task-id>/` 目录树并放入插件文件，严禁把先接入的机器人写在顶层公用目录，也严禁在 `common/` 内硬编码任何机器人/任务名。

### 2.4 改动/新增代码归属判断准则
- **第一级判断：与端无关（`shared/`） vs 与四端有关（`simulation` / `runtime` / `learning` / `hardware`）**
  - **与具体运行端无关**（跨端接口契约、通用数学/轨迹工具、方案绑定与配方）：统一归属 `shared/`。
    - 与机器人和任务均无关 → 归属 `shared/common/`（严禁出现任何机器人名、任务名或场景特定阈值/魔数）；
    - 与机器人或任务有关 → 归属 `shared/scenarios/<robot-id>/`（跨任务共享的机器人级契约档案与扩展置于 `profiles/`、`extensions/`）及 `shared/scenarios/<robot-id>/<task-id>/`（任务级档案置于 `profiles/{robot,control}/`，任务级配方置于 `configs/`）。
  - **与四端之一有关**（属于 `simulation/mujoco/`、`runtime/ros2/`、`learning/lerobot/`、`hardware/` 某一端的具体实现）：先确定归属哪一端，再按第二级判断端内归属。
- **第二级判断：端内机器人/任务无关（`common/`） vs 机器人专属（`<robot-id>/`） vs 任务专属（`<robot-id>/<task-id>/`）**
  - **与机器人和任务均无关**（通用控制、运动规划、跟随器、渲染、ROS 通信、训练与推理管线等通用机制） → 归属该端的 `common/`（或完全方案无关的端主包，严禁硬编码 `<robot-id>` 或 `<task-id>`）；
  - **与特定机器人绑定、与具体任务无关**（如机器人装配 `assembly`、运动学模型与协同轴组拓扑、关节 I/O 映射、`robot_config`、真机特定型号驱动） → 归属该端下的 `<robot-id>/`；
  - **与特定机器人及特定任务绑定**（如任务场景 `scene`、任务环境 `environment`、程序示教 `teacher`、阶段门控 `gate`、成功评价器 `evaluator`、任务专属配置与诊断） → 归属该端下的 `<robot-id>/<task-id>/`。
- 涉及破坏性改动或破例需求时，统一遵照 `§0` 全局红线先请示后实施。

---

## 3. 容器环境、脚本与验收门禁规范

### 3.1 容器入口 `env.sh`
- 仓库根目录 [`env.sh`](env.sh) 是管理所有 Docker 环境的唯一宿主机入口，支持子命令：`<env>`、`up`、`build`、`exec`、`stop`、`rm`、`viewer`、`ps`。
- **容器变更生效方式**：修改 Docker 挂载或环境变量配置后须执行 `./env.sh up <env>` 使配置生效；修改 Dockerfile 或依赖后须执行 `./env.sh build <env>` 显式重建镜像；在已运行的容器内执行脚本或测试使用 `./env.sh exec <env> [cmd...]`。
- **新增容器环境时**：必须同步在 `env.sh` 的 `resolve_env` 与帮助信息中注册环境名、Compose 路径与服务名。
- **单端启停边界**：所有端共享 Compose 项目名 `sim-dual-arm`。清理单端必须使用 `stop` + `rm`（或 `./env.sh stop <env>` / `./env.sh rm <env>`），**严禁对单端执行 `docker compose down --remove-orphans`**。

### 3.2 宿主机与三端容器执行及挂载边界
修改源码只能在宿主机工作区进行（实时映射进容器）；运行业务脚本与测试必须在对应环境或容器内执行，严禁在容器内尝试写入只读挂载路径：

| 执行环境 | 只读挂载（禁止容器内写入） | 可写挂载 | 负责执行的脚本与入口 |
| :--- | :--- | :--- | :--- |
| **宿主机（仓库根目录）** | — | 全部工作区源码、`build/`、`var/` | `./env.sh`、`./record_parallel.sh`、`./inference_parallel.sh`、`./env.sh viewer ...` |
| **`simulation` 容器** | `/app`（`simulation/mujoco/`）、`/shared`（`shared/`）、`/opt/ros-dev`（`build/ros-dev/`） | `/data`（`var/`） | `scripts/teacher_backend.sh`、`scripts/inference_backend.sh`、`scripts/backend.sh`、`scripts/test.sh` |
| **`runtime` 容器** | `/app`（`runtime/ros2/`）、`/shared`（`shared/`） | `/build`（`build/`）、`/data`（`var/`） | `scripts/build_sdk.sh`、`scripts/record.sh`、`scripts/client_interactive.sh`、`scripts/client_policy.sh`、`scripts/test.sh` |
| **`learning` 容器** | `/app`（`learning/lerobot/`）、`/shared`（`shared/`）、`/third_party`（`third_party/`）、`/opt/policy-sdk`（`build/generated/policy-sdk/`） | `/data`（`var/`） | `scripts/convert.sh`、`scripts/train.sh`、`scripts/server_policy.sh`、`scripts/test.sh` |

> 容器内测试（`tests/`）统一按 [`tests/README.md`](tests/README.md) 方式将工作区源码打包送入容器 `/tmp` 临时目录，并调用对应端的 `/app/scripts/test.sh` 执行。

### 3.3 用户脚本设计与输出规范
本节规范同时适用于**宿主机根目录脚本**（`env.sh`、`record_parallel.sh`、`inference_parallel.sh`）以及**三端容器内的 `scripts/*.sh`**：
- **单脚本单领域**：每个用户脚本只负责单一明确领域的任务，不把无关子命令混在一起。
- **运行后展示配置与有效参数**：业务脚本启动时必须清晰打印实际使用的配置文件路径（如 Binding、配方路径、输出目录）与展开后的有效参数。
- **长时任务用非刷屏进度条**：采集、转换、训练、批量推理等长时任务在 TTY 下必须使用原地刷新的非刷屏进度条；非 TTY 下仅输出关键阶段与最终摘要。
- **高信噪比、易于理解**：屏蔽底层冗余过程事件（明细写入日志文件），终端只输出对操作与结果判断有直接价值、易于人类阅读的信息。

### 3.4 改动-验证触发清单（阶段二硬门禁）
阶段二收敛完成后，必须对照下表触发对应实测验收（严禁仅凭语法检查代替运行验证）：

| 改动范围 | 必跑验证命令与通过判据 |
| :--- | :--- |
| **`shared/common/` 下 Protobuf 或 ROS 2 IDL** | 在 `runtime` 容器重跑 `./env.sh exec runtime scripts/build_sdk.sh`，并通过对应 `pytest` 用例 |
| **采集、录制或导出链路**（`runtime` 采集 / Teacher / 导出逻辑） | 启动对应 `scripts/teacher_backend.sh`，在 `runtime` 容器跑通一次试采：`scripts/record.sh --robot <robot-id> --task <task-id> --purpose pilot --count 1 --max-consecutive-failures 1`（退出码为 0 且 `qualified` 为 1） |
| **`simulation/mujoco/`**（仿真后端、场景、运动学或评价器） | 按 [`tests/README.md`](tests/README.md) 在 `simulation` 容器执行对应 `scripts/test.sh`（如 `tests/integration/test_sim_scene.py`）；涉及闭环语义时补跑一次 `pilot` 试采或单局推理 |
| **`learning/lerobot/`**（转换、训练或 Policy 服务） | 按 [`tests/README.md`](tests/README.md) 在 `learning` 容器执行对应 `scripts/test.sh`；涉及推理服务或协议时，启动 `scripts/server_policy.sh` 与 `scripts/inference_backend.sh`，并在 `runtime` 容器跑通 `scripts/client_policy.sh`（或宿主机 `./inference_parallel.sh --robot <robot-id> --task <task-id> --count 1 --workers 1`，可按需传 `--seed-start`）至少一局 |
| **`runtime/ros2/`**（控制、轨迹或流式客户端） | 按 [`tests/README.md`](tests/README.md) 在 `runtime` 容器执行对应 `scripts/test.sh`，并按影响面补跑一次 `pilot` 试采或单局 `scripts/client_policy.sh` |
| **Docker Compose 挂载/环境或 Dockerfile 依赖** | 执行 `./env.sh up <env>` 或 `./env.sh build <env>`，并确认对应容器内入口脚本可正常运行 |

> **验收防污染与进程回收**：Agent 自测严禁使用 `--purpose formal` 污染正式台账；验收期间启动的临时后端或 Policy 服务在测试结束后必须立即停止清理，避免残留进程占用 `50051` 端口、ROS 2 话题或显存。

### 3.5 合并主分支前全任务闭环审查门禁（`norm/*` → `main` 必检）
除符合 `§1` 轻量改动豁免（纯文档、单文件参数微调等低风险小改）外，任何改动从 `norm/*` 合并回 `main` 前，**必须对项目中全部现有任务**（当前为 `marvin_sim/stack` 与 `marvin_sim/handover`；后续新增任务同步纳入）依次跑通以下 4 步全链路实测审查，并将结果汇报用户：

- **执行原则（优先模拟真实用户入口）**：每一步尽量直接调用各端 `README.md` 与 [`tests/README.md`](tests/README.md) 文档化的标准脚本入口（如 `simulation/mujoco/scripts/teacher_backend.sh`、`simulation/mujoco/scripts/inference_backend.sh`、`./env.sh exec runtime scripts/*.sh`、`./env.sh exec learning scripts/*.sh`）；若个别步骤因小样本转换/回放需求不易直接复用生产默认配方，允许在 `var/<robot-id>/<task-id>/debug/` 下生成临时 `--config` 配方或使用 [`tests/fakes/policy/run_fake.py`](tests/fakes/policy/run_fake.py)，不必为了凑参数而强行修改生产脚本或正式配方。
1. **试采（Pilot 采集）**：启动对应任务的 `teacher_backend.sh --robot <robot-id> --task <task-id>`（等待 `ready.json` 就绪），在 `runtime` 容器执行 `scripts/record.sh --robot <robot-id> --task <task-id> --purpose pilot --count 2 --max-consecutive-failures 2`（至少 2 局以满足 train/validation 划分），确认退出码为 `0` 且 `qualified` 为 `2`，并记录合格录制目录、对应 `seed` 与 `export/labels.json` 路径。
2. **试训（数据转换 + Smoke 训练）**：基于第 1 步生成的 pilot 录制，在 `learning` 容器依次执行 `scripts/convert.sh` 与 `scripts/train.sh --mode smoke`（小样本试采可使用 `var/<robot-id>/<task-id>/debug/` 下的临时 `--config` 配方并配合 `--max-steps`），确认退出码为 `0` 且输出可用于推理加载的模型产物（含 `model.safetensors`）。
3. **试推（真实模型闭环推理，不要求任务成功）**：基于第 2 步产出的 smoke 模型，启动 `learning` 端 `scripts/server_policy.sh`（指向 smoke 模型目录）与 `simulation` 端 `scripts/inference_backend.sh --robot <robot-id> --task <task-id>`，在 `runtime` 容器运行 `scripts/client_policy.sh` 执行单局闭环推理，确认三端 gRPC / ROS 2 控制与观测流转无报错、正常退出并完成录制（**不要求**少步数 smoke 模型完成任务成功）。
4. **采集数据推理源成功判定与完整推理链路验证**：使用第 1 步真实 pilot 录制导出的 `export/labels.json` 作为推理源（通过 [`tests/fakes/policy/run_fake.py`](tests/fakes/policy/run_fake.py) 启动真实 gRPC `PolicyService` 回放录制标签），以与该 pilot 录制一致的 `--seed` 启动 `scripts/inference_backend.sh`，在 `runtime` 容器通过 `scripts/client_policy.sh --config <debug_replay_yaml> --reset-seed <pilot_seed>` 运行单局闭环测试（临时配置允许测试策略并对齐首帧时序），确认最终评价返回 `TASK_SUCCEEDED`（`action_source` 为 `policy`、`evaluated` 为 `true`、`conditions_passed` 为 `true` 且 `recording.complete` 为 `true`）。
- **审查后清理边界**：四步全流程严禁使用 `--purpose formal` 污染正式台账；完成后必须立即停止全部临时后台进程并清理本次审查产生的临时 pilot 录制、debug 配方、smoke 模型、日志与试采台账，待确认工作区与 `var/` 洁净后再请示用户合并 `main`。

---

## 4. 目录与产物隔离规范

### 4.1 `var/`（容器 `/data`）四分法（强制隔离）
所有按方案运行的产物必须归入 `var/<robot-id>/<task-id>/`（容器内 `/data/<robot-id>/<task-id>/`，如 `var/marvin_sim/stack/`、`var/marvin_sim/handover/`），任务目录下严格分为**四个一级平级目录**（`datasets/`、`models/`、`logs/`、`debug/`；其中 `datasets/` 下再分 `raw/` 与 `lerobot/`），**绝对禁止混放或私自在 `var/` 根下新建未授权目录**：

| 目录 | 允许存放 | 严禁混入 |
| :--- | :--- | :--- |
| `datasets/raw/` | 合格的原始录制数据（`raw/` 内 `manifest.json`、`task_result.json`、`recording/*.mcap`，以及 `export/` 解包帧/标签）与 `formal-select.json` | 不合格尝试、运行日志、模型权重 |
| `datasets/lerobot/` | 经转换后供训练/验证消费的 LeRobot 数据集（`train/`、`validation/`、`processors/`、`manifest.json`） | 原始 MCAP、模型权重、运行日志 |
| `models/` | **仅**以 `<模型名>_<YYYYMMDD_HHMMSS>/` 命名的模型文件夹（如 `act_20261117_143022/`），目录内**仅允许包含推理直接加载所需的最小产物集合**（当前 ACT 为 `model.safetensors` 网络权重与归一化统计量，以及仅作备忘不消费的 `training_args.yaml`） | 中间 checkpoint、预算台账、互斥锁、`runs/` 与 `bundles/` 中间层、原始录制数据、训练数据集、运行日志 |
| `logs/` | 三端运行事件日志、后端日志（`backend/<run-id>/`）、采集台账与尝试日志（`collection/`）、转换日志（`convert/`）、训练过程指标（`training/`）、策略服务日志（`policy-service/`）、单局推理录制（`recordings/`）及批量推理逐局记录（`batch-policy/<batch-id>/`） | 正式 LeRobot 数据集、模型权重、离线诊断报告 |
| `debug/` | 环境探针、场景/运动学检查、离线评估报告（如 `batch-policy/<batch-id>/report.json`）、集成/冒烟测试临时产物 | 正式采集录制、正式训练数据集、正式模型包 |

> **合法保留段边界**：直接配置模式（无 `<robot-id>/<task-id>` 对）统一使用 `var/_direct/`；与具体方案无关的系统级调试产物统一放入 `var/_system/debug/`。严禁在 `var/` 根下新建其他目录。
> **新产物准入铁律**：规范阶段保留的新功能若产生新的正式文件/目录类型，**必须先向用户说明其性质与消费者，经确认归属后再定型**。

### 4.2 `build/`（容器 `/build`）精简规范
`build/` 仅存放运行时直接加载的生成 SDK，禁止存放评估报告、缓存或临时编译目录：
- `build/ros-dev/`：仅保留 `install/` 与 `contracts.json`；`colcon` 的 `build/`、`log/`、`COLCON_HOME` 必须在 `/tmp` 执行并在结束后清理。
- `build/generated/policy-sdk/`：仅保留 `dual_arm_policy/` 与 `source.sha256`。

---

## 5. Agent 交付与沟通准则
- **先判断再改动，直接给结论**：定位问题或实施修改时，以实测数据与最小复现为依据；向用户汇报时先给结论与验收结果，再附关键根因与改动明细。
- **规范交付不留试错痕迹**：在阶段二（规范阶段）收尾时，代码、注释与文档中不保留试错痕迹、废弃方案说明或免责套话；如有未验证项或后续优化方向，仅在向用户的回复末尾集中说明。
- **严格遵守节点确认**：开发完成后的 Git 保存、规范达标后的 Git 保存与合并删分支，均须按 `§0` 与 `§1` 流程经用户确认并下达指令后执行。

### 5.1 子代理分工与验收边界（Subagent Delegation）
- **MUST**：面对大范围代码检索、多文件读取、局部文档同步或具备明确变更清单的机械替换任务时，主代理应优先委派高性价比轻量模型或只读探索子代理并行完成，以最小化主上下文占用与额度消耗。
- **验收边界**：子代理负责只读检索与明确清单的机械修改；**跨端契约变更、跨目录路径/命名重命名（含 YAML 字符串路径与脚本参数隐式引用）及最终一致性校验必须由主代理复核**。
