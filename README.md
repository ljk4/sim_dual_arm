# sim_dual_arm

面向双臂机器人的仿真数据采集、ACT 策略训练与流式控制闭环系统。当前提供两个可直接运行的方案：

- [`panda_stack`](shared/scenarios/panda_stack/README.md)：MuJoCo 双 Panda 堆叠；
- [`marvin_handover`](shared/scenarios/marvin_handover/README.md)：Marvin 抓取—空中交接—入箱。

两个方案共用同一套通用实现：仿真端负责物理步进、三路相机渲染与程序示教（Teacher），运行端负责观测同步、流式动作块调度与 MCAP 录制，学习端负责数据集转换、ACT 训练、Bundle 导出与 gRPC Policy 服务。

## 架构概览

仓库按职能划分为三个容器化运行端、真机边界与公共/方案层：

- **`simulation/mujoco/`**：MuJoCo 物理仿真后端，负责机器人动力学、三路 EGL 相机渲染、Ruckig 有界跟随与接触判定，通过 ROS 2 接口对外提供服务。
- **`runtime/ros2/`**：ROS 2 运行端，负责多路观测同步、控制租约与流式动作块调度、MCAP 录制、离线审计、导出与采集调度。
- **`learning/lerobot/`**：ROS-free 学习与策略服务端，负责数据集转换、ACT 混合精度训练、模型包（Bundle）导出与 gRPC Policy 推理服务。
- **`hardware/`**：真机 Robot API 边界说明；当前没有可验证的真实驱动。
- **`shared/common/`**：公共层，只放跨端接口约束（Policy Protobuf、ROS 2 IDL、JSON Schema）、共享 Python 包与通用工具，不选择具体任务、机器人或算法。
- **`shared/scenarios/<id>/`**：方案绑定层，用 `binding.yaml` 选择档案、配方与方案专用接口；[`_template`](shared/scenarios/_template/README.md) 说明新增方案的方法。

**三端统一的一步式工作流**：任何端启动时只传 `--scenario <id>`，由该方案 `binding.yaml` 在内存中选择并展开配方；不再生成 `/data/prepared/` 中间目录，也不再需要在三端分别执行 `prepare`。所有入口支持重复 `--set dot.path=value` 覆盖配方中已存在的键，用于临时调参。直接配置模式仍可用：把 `--scenario <id> [--role ...] [--bundle ...]` 换成对应端的 `--config <recipe>`（学习端策略服务也接受 `--config`）。

```text
simulation backend ──ROS 2──▶ runtime ──gRPC──▶ learning policy service
        ▲                        │
        └──── 观测 / 执行反馈 ─────┘
```

---

## 1. 环境准备

所有宿主机命令默认在仓库根目录下执行。

```bash
git clone https://github.com/ljk4/sim_dual_arm.git
cd sim_dual_arm

# 首次启动容器前，以当前宿主机用户创建输出根目录（使用 ./env.sh 启动时会自动创建）
mkdir -p build var
```

仓库根目录提供 [`env.sh`](env.sh) 统一管理 `runtime`、`simulation`、`learning` 三个 Docker 环境：

```bash
./env.sh <env>                # 构建、后台启动并进入容器 bash（up -d --build + exec）
./env.sh up <env>             # 仅构建并后台启动容器
./env.sh exec <env> [cmd...]  # 进入已运行的容器执行命令（默认 bash）
./env.sh stop <env>           # 停止指定环境容器
./env.sh rm <env>             # 停止并删除指定环境容器
./env.sh viewer [args...]     # 显式指定方案或配置，启动 MuJoCo 实时可视化
./env.sh ps                   # 查看当前项目所有容器状态
```

`build/` 仅存放运行时加载的生成 SDK，`var/` 存放运行日志、录制、数据集、模型与台账。两者整体由 Git 忽略，不提交空目录或 `.gitkeep`；各构建和运行命令按需创建子目录。首次启动前由宿主机用户创建这两个根目录，避免 Docker 自动创建后归 root 所有；UID/GID 应与 Compose 中的运行用户一致。

三端容器统一将各自的实现目录只读挂载为 `/app`（`working_dir: /app`），公共目录 `shared/` 只读挂载为 `/shared`，运行输出 `var/` 挂载为 `/data`；runtime/learning 额外将 `build/` 挂载为 `/build`，simulation 仅只读挂载所需的 `build/ros-dev` 至 `/opt/ros-dev`。更新挂载配置后，重新执行 `./env.sh up <env>` 使配置生效。

所有运行期产物按方案归入 `var/<scenario-id>/`（容器 `/data/<scenario-id>/`），任务目录内严格分为四个平级目录：

```text
var/<scenario-id>/
├── datasets/       # LeRobot 数据集、manifest、filter 掩码、formal-select.json
├── models/         # 训练 runs/checkpoints、训练预算台账、bundles/<name>
├── logs/           # 三端事件日志、采集台账、录制内容、record-pools
└── diagnostics/    # 场景/运动学/交接检查、离线评估与测试产物
```

直接配置模式没有方案 id，统一使用保留段 `var/_direct/`；与方案无关的环境探针写入 `var/_system/diagnostics/environment/`。

仓库不附带已生成的 SDK、采集数据、训练权重或模型包。下文的输出路径用于生成和使用本机产物；历史实验记录不代表当前机器已有这些文件。

学习端另外使用固定版本的 LeRobot 源码，首次使用须按[第三方源码说明](third_party/README.md)准备；容器镜像不会代为恢复宿主机已清理的源码目录。

---

## 2. 数据采集流程

按顺序完成：**单路试采一次（pilot，可观察可视化）→ 停止手动后端与 viewer → 宿主机并行补足 50 条合格正式数据 → 自动生成训练/验证清单 → 第 3 节转换与训练**。

### 2.1 启动 Runtime 容器并构建 SDK

构建、后台启动并进入 `runtime` 容器：

```bash
./env.sh runtime
```

在 `runtime` 容器内一条命令生成 ROS 2 接口工作空间与 Policy Protobuf/gRPC SDK：

```bash
scripts/build_sdk.sh
```

### 2.2 启动 Simulation 容器与试采示教后端

在宿主机新终端中构建并进入 `simulation` 容器：

```bash
./env.sh simulation
```

在 `simulation` 容器内直接以方案启动示教后端（显式指定方案；默认 `--seed 0 --run-id teacher-backend`，启动时会打印所用配置文件路径与完整生效参数）：

```bash
scripts/teacher_backend.sh --scenario panda_stack
```

`teacher_backend.sh` 会在内存中读取 `binding.yaml`、展开配方路径、打印配置路径与详细参数，并写入本次实际生效的快照 `/data/panda_stack/logs/backend/teacher-backend/backend.yaml`；运行日志位于 `/data/panda_stack/logs/backend/teacher-backend/log`。`--run-id teacher-backend` 同时是 runtime 端 `record.sh` 的默认后端日志位置，单路试采无需再指定 `--backend-log`。

需要切换方案或临时调参时可追加参数，例如降低仿真速度：

```bash
scripts/teacher_backend.sh --scenario panda_stack --seed 0 --run-id teacher-backend --set run.realtime_speed=0.4
```

### 2.3 试采时开启实时 MuJoCo 可视化（可选）

`viewer` 独立订阅后端发布的只读物理状态快照，不参与物理步进与控制。它同样直接接受 `--scenario`（或直接 `--config`）。在宿主机桌面环境下（若报 X11 权限错误，先在宿主机执行 `xhost +SI:localuser:$(id -un)`）：

```bash
# 显式选择方案和角色；policy 角色使用 --role policy
./env.sh viewer --scenario panda_stack --role teacher
```

### 2.4 单路试采一次，确认完整链路

回到 `runtime` 容器终端，使用刚启动的手动 teacher backend 试采一次。试采指定 `pilot`，不占用正式 50 条的名额（`scripts/record.sh` 会在目录不存在时自动创建、已有台账时默认增量追加，并在启动后自动打印所用配置文件路径、详细参数与当前 campaign 状态）：

```bash
# 只试一次：合格即结束，失败也立即结束
scripts/record.sh \
  --scenario panda_stack --purpose pilot --count 1 --max-consecutive-failures 1
```

确认试采命令退出码为 0，末尾 `recording.finished` 的 `reason` 为 `target_reached`、`qualified` 为 1；本局应同时满足任务成功、录制完整、导出质量合格。若失败，先检查日志并修复，再重新执行本节的试采命令。

`--count` 统计本次新增合格条数；`--max-consecutive-failures 1` 让试采最多尝试一次。每次试采继续记录在原台账并占用 pilot 预算，种子自动递增；pilot 数据不会进入后面的正式训练/验证清单。

### 2.5 停止试采后端和 viewer

试采合格后，在 viewer 终端（若已开启）和手动 teacher backend 终端分别按 `Ctrl-C`，释放计算资源。保留 runtime 容器运行；simulation 容器可保持空闲，后续第 3.4 节仍会使用它启动推理后端。

### 2.6 宿主机并行采集，补足 50 条合格正式数据

在**宿主机新终端的项目根目录**执行：

```bash
./record_parallel.sh \
  --scenario panda_stack --purpose formal --target-total 50 \
  --workers auto --max-consecutive-failures 3
```

`--target-total 50` 表示正式台账累计达到 50 条合格数据：已有正式数据时只补足差额；已经达到目标则直接成功退出。若中断或连续失败提前结束，修复原因后再次执行相同命令即可续采，不需要手动计算剩余条数或清空台账。该参数不能与表示“新增条数”的 `--count` 同时指定。

入口自动创建独立的 MuJoCo backend，复用 runtime 容器并行录制与导出。每路使用独立的 ROS domain、命名空间和日志目录，共用原 campaign 的种子和预算。自动档按 CPU、可用内存和显存选择至多 4 路；如需固定并发，将 `--workers auto` 改为 `--workers 4`。

并行 worker 禁用 viewer 状态发布、预览 PNG 和 GIF，保留训练 RGB、标签及质量门禁。连续失败按完成顺序计数，合格一次清零；连续 3 次不合格会取消在途任务并提前退出，取消项仍占预算。预算/种子不足也会停止。按 `Ctrl-C` 可中断本批次；命令结束时自动清理本批次后台实例。配置、backend 日志和性能汇总保存到 `var/panda_stack/logs/record-pools/pool-*/`。

### 2.7 查看数据集与训练/验证清单状态

正式台账累计达标并正常结束时（`reason=target_reached`），采集入口会自动原子写出 `var/panda_stack/datasets/formal-select.json`，无需手动重定向。回到 **runtime 容器终端**即可查看当前数据集、磁盘占用与训练/验证清单划分状态（显式指定方案）：

```bash
scripts/record.sh status --scenario panda_stack
```

确认清单状态显示为 `SELECTED`（训练 45 条 / 验证 5 条，自动排除 `pilot` 试采）后，即可进入第 3 节的转换与训练。

---

## 3. 训练与策略推理流程

### 3.1 启动 Learning 容器

在宿主机新终端中构建并进入 `learning` 容器：

```bash
./env.sh learning
```

### 3.2 转换数据集与训练 ACT 模型

在 `learning` 容器内将正式清单中的 50 条合格 episode 转换为 LeRobot 数据集（45 条训练 / 5 条验证，输出至 `/data/panda_stack/datasets/panda_stack_learning_v1`）。转换配方直接读取 `/data/panda_stack/datasets/formal-select.json`，无需移动原始数据：

```bash
scripts/convert.sh --scenario panda_stack
```

转换时终端只显示配置、日志路径和原地刷新的进度，包括已完成条数、百分比、用时和预计剩余时间；编码器等详细输出写入日志。训练使用同样的显示方式，额外显示 step、loss 和验证状态。重定向输出时仅保留阶段摘要，避免生成大量进度行。可用 `--log-file /data/panda_stack/logs/convert/<文件名>.log` 指定详细日志；需要原始输出和 JSON 结果时添加 `--no-progress`。

训练使用共享官方 ResNet18 权重文件 `third_party/weights/resnet18-f37072fd.pth`（两方案共用，长期保留，不属于 `build/` 生成产物）；也可在 learning 容器内用 `RESNET18_WEIGHTS` 指定现有权重文件。训练按离线方式运行，缺少权重时会报错，不会自动下载。

启动完整模式训练（默认 5000 步，启用 PyAV 视频解码、张量批处理、多进程预取与 CUDA bfloat16 混合精度）：

```bash
scripts/train.sh --scenario panda_stack --mode full
```

训练完成后，记录终端输出的 `run_dir` 并导出可部署的 safetensors 模型包（Bundle）：

检查点保存在 `models/runs/<run-id>/`；训练事件、指标和训练终态摘要保存在同方案的 `logs/training/<run-id>/`。

```bash
TRAIN_RUN="/data/panda_stack/models/runs/<实际生成的-run-目录名>"
scripts/bundle.sh \
  --scenario panda_stack --mode full \
  --run "$TRAIN_RUN" \
  --output /data/panda_stack/models/bundles/<verified-bundle>
```

### 3.3 直接启动 Policy 推理服务

在 `learning` 容器内用 Bundle 直接启动 gRPC Policy 服务；`--scenario` 读取方案绑定的 `policy_service` 配方与机器人档案，`--bundle` 覆盖 `models.bundle`：

```bash
scripts/policy.sh \
  --scenario panda_stack \
  --bundle /data/panda_stack/models/bundles/<verified-bundle>
```

服务默认监听 `50051`，日志目录为 `/data/panda_stack/logs/policy-service`（可用 `--log-directory` 或环境变量 `ACT_LOG_DIRECTORY` 修改）。保持该终端运行。若本机尚未生成 Policy SDK，先在 runtime 容器执行 `scripts/build_sdk.sh`。

### 3.4 直接启动推理仿真后端

回到 `simulation` 容器，用同一 Bundle 以 policy 角色启动后端：

```bash
scripts/policy_backend.sh \
  --scenario panda_stack \
  --bundle /data/panda_stack/models/bundles/<verified-bundle> \
  --seed 20000 --run-id policy-backend
```

运行快照与日志分别位于 `/data/panda_stack/logs/backend/policy-backend/backend.yaml` 与 `/data/panda_stack/logs/backend/policy-backend/log`。后端与第 3.3 节的 Policy 服务、第 3.5 节的 runtime 需同时在线。

### 3.5 直接运行推理或交互闭环

回到 `runtime` 容器，直接以方案启动一局交互式闭环（自动生成或使用指定 `run-id`，输入 `y` 复位环境并执行一局，结束后保持 IDLE 直至 `Ctrl-C` 退出）：

```bash
scripts/interactive.sh \
  --scenario panda_stack \
  --bundle /data/panda_stack/models/bundles/<verified-bundle> \
  --seed 20000 --run-id policy-interactive
```

非交互式单局执行使用 `scripts/policy.sh`，录制默认落到 `/data/<scenario-id>/logs/recordings/<run-id>`；需要另选位置时再显式传 `--output`：

```bash
scripts/policy.sh \
  --scenario panda_stack \
  --bundle /data/panda_stack/models/bundles/<verified-bundle> \
  --reset-seed 20000 --run-id policy-run
```

后续测试不同随机种子时，直接传入新的种子即可，无需重新生成任何配置：

```bash
scripts/interactive.sh \
  --scenario panda_stack \
  --bundle /data/panda_stack/models/bundles/<verified-bundle> \
  --seed 20001 --run-id policy-interactive-2
```

如需无人工干预批量仿真推理测试（仅支持 MuJoCo 仿真后端且禁止可视化，执行前需保持第 3.3 节 `learning` 容器的 `scripts/policy.sh` 与第 3.4 节 `simulation` 容器的 `scripts/policy_backend.sh` 同时在线；启动时打印配置与参数，结束后输出成功率与报告路径）：

```bash
scripts/batch_policy.sh \
  --scenario panda_stack \
  --bundle /data/panda_stack/models/bundles/<verified-bundle> \
  --count 10 --seed-start 20000
```

本轮每个方案累计 120 次推理，每批最多 10 次；保留失败、拒绝与取消记录，并按模型和有效配置分组审计。验收要求每个方案至少一次真实任务成功。

### 3.6 用 `--set` 调参与采集产物

所有 `--scenario` 入口都支持重复 `--set dot.path=value`，只覆盖配方中已存在的点分路径；值按 YAML 解析，未知路径或类型错误会在启动前被严格校验拒绝。常见用法：

```bash
# simulation：降低仿真速度
scripts/teacher_backend.sh --scenario panda_stack --run-id teacher-slow --set run.realtime_speed=0.4

# runtime：放宽观测最大年龄（该键由观测档案派生，也可覆盖）
scripts/interactive.sh --scenario panda_stack \
  --bundle /data/panda_stack/models/bundles/<verified-bundle> \
  --run-id policy-fast --set observation.max_age_ns=200000000

# learning：只运行 smoke 预算并指定轮次数
scripts/train.sh --scenario panda_stack \
  --mode smoke --set modes.smoke.steps=64

# 采集：为每次尝试追加运行时覆盖
scripts/record.sh --scenario panda_stack --purpose pilot --count 1 \
  --set observation.max_age_ns=250000000
```

直接配置模式同样支持 `--set`：把 `--scenario ...` 换成 `--config <recipe>` 即可，二者互斥。

正式采集的每次尝试在 `var/<scenario-id>/logs/campaign/` 下拍平保存，不再有 `stack/<run_id>` 层级，也不再复制重复 JSON；正式筛选清单另存到 `datasets/`：

```text
var/panda_stack/
├── datasets/
│   └── formal-select.json         # 目标达标时自动原子写出，供 convert 消费
├── models/                        # 训练 runs/checkpoints、预算台账、bundles/
├── logs/
│   ├── backend/<run-id>/          # 仿真后端快照、run.json 与事件日志
│   ├── runs/<run-id>/             # runtime 事件日志
│   ├── recordings/<run-id>/       # 单局/交互录制
│   ├── record-pools/pool-*/       # 并行采集批次配置、worker 日志与 summary
│   └── campaign/
│       ├── attempts.jsonl         # 追加式台账：预算、种子、终态
│       └── attempts/
│           └── <purpose>-<seed>-<run_id>/  # attempt 目录（direct 模式另有 runtime.yaml）
│               ├── raw/                    # 录制最终目录
│               │   ├── manifest.json
│               │   ├── task_result.json
│               │   └── recording/          # rosbag2 MCAP
│               ├── logs/<run_id>/          # 运行端事件日志
│               └── export/
│                   ├── labels.json         # 逐帧标签与来源元数据
│                   ├── raw_predictions.json
│                   ├── quality.json        # 质量门禁结果（唯一一份）
│                   ├── rgb/<frame>/<camera>.png
│                   ├── preview/<phase>_<camera>.png   # 可选；并行采集时跳过
│                   └── video/<camera>.gif             # 可选；并行采集时跳过
└── diagnostics/                   # 场景/运动学/交接检查、离线评估与测试产物
```

---

## 4. 使用 marvin_handover

`marvin_handover` 与 `panda_stack` 使用完全相同的入口，训练默认同样为 50 条正式数据（45 训练 / 5 验证）与 full 5000 步，共享同一份 ResNet18 权重；只需替换 `--scenario`：

```bash
# simulation：示教后端
scripts/teacher_backend.sh --scenario marvin_handover --seed 0 --run-id teacher-backend

# runtime：试采
scripts/record.sh --scenario marvin_handover \
  --purpose pilot --count 1 --max-consecutive-failures 1

# 宿主机项目根目录：并行补足 50 条正式数据
./record_parallel.sh --scenario marvin_handover \
  --purpose formal --target-total 50 --workers auto

# learning：正式转换与 full 训练
scripts/convert.sh --scenario marvin_handover
scripts/train.sh --scenario marvin_handover --mode full

# 推理：learning 策略服务、simulation 后端、runtime 交互闭环
scripts/policy.sh --scenario marvin_handover \
  --bundle /data/marvin_handover/models/bundles/<bundle>
scripts/policy_backend.sh --scenario marvin_handover \
  --bundle /data/marvin_handover/models/bundles/<bundle> \
  --seed 5100 --run-id policy-backend
scripts/interactive.sh --scenario marvin_handover \
  --bundle /data/marvin_handover/models/bundles/<bundle> \
  --seed 5100 --run-id policy-interactive

# runtime：批量推理（需保持上方 policy.sh 与 policy_backend.sh 在线），每批最多 10 次，累计 120 次，验收至少一次真实成功
scripts/batch_policy.sh --scenario marvin_handover \
  --bundle /data/marvin_handover/models/bundles/<bundle> \
  --count 10 --seed-start 5100
```

该方案的 `binding.yaml` 中 `models.bundle` 为空；推理时通过 `--bundle` 显式提供已验证 Bundle，档案与配方无需改动。方案说明见 [`shared/scenarios/marvin_handover/README.md`](shared/scenarios/marvin_handover/README.md)。

---

目录导航：[文档索引](docs/README.md) · [公共层与方案绑定](shared/README.md) · [实施规则](RULES.md)
