# sim_dual_arm

面向双臂机器人的仿真与真机采集、策略训练及流式控制闭环系统。系统按 **机器人本体（Robot）× 执行后端（Backend）× 运行编排（Runtime）× 学习范式（Learning）** 四维正交组织，当前提供两个完整闭环方案：

- [`marvin/mujoco/stack`](shared/scenarios/marvin/mujoco/stack/README.md)：Marvin × MuJoCo 双臂桌面单臂堆叠；
- [`marvin/mujoco/handover`](shared/scenarios/marvin/mujoco/handover/README.md)：Marvin × MuJoCo 抓取—空中交接—入箱。

两个方案共用同一套通用实现与 Marvin 公共本体资产（`shared/robots/marvin/`）：MuJoCo 后端负责物理步进、三路相机渲染与程序示教（Teacher），运行端负责观测同步、流式动作块调度与 MCAP 录制，学习端负责数据集转换、ACT 训练与 gRPC Policy 服务。

Stack 与 Handover 均按各自配方的固定物块位姿复位。采集和评估的种子用于区分运行。

## 架构概览

仓库按职能划分为运行端与可扩展执行/学习端、公共与本体资产层及三层方案绑定层：

- **`simulation/mujoco/`**：MuJoCo 物理仿真后端（容器名 `mujoco`），负责机器人动力学、三路 EGL 相机渲染、已提交轨迹的安全校验与执行、接触判定和程序示教。
- **`runtime/ros2/`**：ROS 2 运行中枢（容器名 `runtime`），负责观测同步、控制租约、Policy 和 Teacher 的轨迹生成与提交、MCAP 录制与离线审计。
- **`learning/lerobot/`**：ROS-free 学习与策略服务端（容器名 `lerobot`），负责数据集转换、ACT 混合精度训练、模型（`model.safetensors` + `training_args.yaml`）写出与 gRPC Policy 推理服务。
- **`hardware/tianji/`**：天机双臂、90C 夹爪与相机驱动、设备配置与独立 Docker 环境（容器名 `tianji`）；真机 ROS 2 接入与 Sim2Real 路线详见 [架构扩展与演进指南](docs/features/架构扩展与演进指南.md)。
- **`shared/common/`**：公共层，存放跨端接口契约（Policy Protobuf、ROS 2 IDL、JSON Schema）、共享 Python 包与通用工具（含宿主机并行编排），与具体机器人、后端、任务或算法无关。
- **`shared/robots/<robot-id>/`**：机器人本体公共资产（URDF、STL 网格、跨后端共享档案与机器人级 ROS 扩展，如 `shared/robots/marvin/`）。
- **`shared/scenarios/<robot-id>/<backend-id>/<task-id>/`**：三层方案绑定层，用 `binding.yaml` 选择档案、配方与扩展接口（如 `shared/scenarios/marvin/mujoco/stack/`、`shared/scenarios/marvin/mujoco/handover/`）；[`_template`](shared/scenarios/_template/README.md) 说明新增方案的方法。

使用 `--robot <robot-id> --backend <backend-id> --task <task-id>` 时，对应方案的 `binding.yaml` 在内存中选择并展开配方。条数、并发数和仿真倍速等运行参数使用专用选项，其余参数在对应配方中修改。直接配置模式使用各命令帮助列出的配置路径参数。

```text
mujoco (or hardware) backend ──ROS 2──▶ runtime ──gRPC──▶ lerobot policy service
             ▲                              │
             └──────── 观测 / 执行反馈 ───────┘
```

各端职责分工、通信契约与未来扩展设计见 [跨端架构分工与解耦审查](docs/features/跨端架构分工与解耦审查.md) 与 [架构扩展与演进指南](docs/features/架构扩展与演进指南.md)。

---

## 1. 环境准备

所有宿主机命令默认在仓库根目录下执行。

```bash
git clone https://github.com/ljk4/sim_dual_arm.git
cd sim_dual_arm

# 首次启动容器前，以当前宿主机用户创建输出根目录（使用 ./env.sh 启动时也会自动创建）
mkdir -p output/build/ros-dev output/build/generated/policy-sdk
```

仓库根目录通过 [`env.sh`](env.sh) 统一管理 `runtime`、`mujoco`、`lerobot`、`tianji` 四个 Docker 环境；`env.sh` 完整子命令以及全部宿主机与容器内脚本的参数说明和示例见 [脚本入口与使用指南](docs/features/脚本入口与使用指南.md)。

所有运行与构建产物统一存放在 `output/`（不纳入 Git）：
- `output/build/`：存放 `runtime` 构建的生成 SDK（`ros-dev/` 与 `generated/policy-sdk/`）。
- `output/<robot-id>/<backend-id>/<task-id>/`（容器内 `/data/<robot-id>/<backend-id>/<task-id>/`）：按三层坐标隔离的运行产物，每个方案目录严格分为四个平级目录（`datasets/`、`models/`、`logs/`、`debug/`）。

容器统一将各自的实现目录只读挂载为 `/app`（`working_dir: /app`），公共目录 `shared/` 只读挂载为 `/shared`，运行输出 `output/` 挂载为 `/data`；`runtime` 将 `output/build/` 读写挂载为 `/build`，`mujoco` 与 `tianji` 只读挂载 `output/build/ros-dev` 至 `/opt/ros-dev` 与 `output/build` 至 `/data/build`，`lerobot` 只读挂载 `output/build/generated/policy-sdk` 至 `/opt/policy-sdk`。更新挂载配置后，重新执行 `./env.sh up <env>` 使配置生效。

天机硬件环境使用 `./env.sh build tianji`、`./env.sh up tianji`，默认读取 `hardware/tianji/configs/devices.yaml` 中已迁入的设备配置。目录和使用方法见 [天机硬件后端](hardware/tianji/README.md)。

```text
output/
├── build/
│   ├── ros-dev/                             # 编译生成的 ROS 2 接口工作空间
│   └── generated/policy-sdk/                # 编译生成的 Policy gRPC/Protobuf Python SDK
└── <robot-id>/<backend-id>/<task-id>/       # 如 output/marvin/mujoco/stack/
    ├── datasets/
    │   ├── raw/
    │   │   ├── formal-select.json           # 正式达标时发布的有序合格局清单
    │   │   └── <purpose>-<seed>-<run_id>/   # 每次合格采集尝试
    │   │       ├── raw/                     # manifest.json、task_result.json、recording/（MCAP）
    │   │       └── export/                  # labels.json、raw_predictions.json、quality.json、rgb/<frame>/<camera>.png
    │   └── lerobot/                         # 转换后的 train/、validation/、processors/、manifest.json
    ├── models/
    │   └── act_<YYYYMMDD_HHMMSS>/
    │       ├── model.safetensors            # 网络权重与归一化统计量，推理直接加载
    │       └── training_args.yaml           # 训练参数备忘，仅记录不消费
    ├── logs/
    │   ├── backend/<run-id>/                # 后端就绪标记（ready.json）、快照（backend.yaml）、status.json 与日志
    │   ├── collection/
    │   │   ├── attempts.jsonl               # 追加式采集台账：预算、种子、终态
    │   │   ├── attempts/<attempt>/          # 每次采集尝试的运行日志
    │   │   └── record-pools/pool_*/         # 并行采集批次配置、worker 日志与汇总
    │   ├── convert/                         # 数据集转换详细日志
    │   ├── training/act_<YYYYMMDD_HHMMSS>/  # 训练事件、指标与终态摘要
    │   ├── policy-service/                  # 策略服务日志与预测记录
    │   ├── recordings/<run-id>/             # 交互/单局推理录制
    │   ├── runs/                            # 运行端会话日志
    │   └── batch-policy/<batch-id>/         # 并行批量推理逐局记录与日志
    └── debug/
        └── batch-policy/<batch-id>/report.json  # 并行批量推理评估报告
```

直接配置模式没有 `<robot-id>/<backend-id>/<task-id>` 三元组，统一使用保留段 `output/_direct/`；与方案无关的系统级调试产物写入 `output/_system/debug/`（`build` 为保留 `robot-id`）。

学习端另外使用固定版本的 LeRobot 源码与离线权重，首次使用须按[第三方源码说明](third_party/README.md)准备；容器镜像不会代为恢复宿主机已清理的源码目录。

也可以只用 `uv` 在云服务器上搭建学习端，并让本机的运行端/仿真端通过 SSH 隧道连接云端策略服务，完整步骤见 [`learning/lerobot/README.md`](learning/lerobot/README.md)。

---

## 2. 数据采集流程

按顺序完成：**单路试采一次（pilot，可观察可视化）→ 停止手动后端与 viewer → 宿主机并行补足 50 条合格正式数据 → 自动生成有序合格清单 → 第 3 节转换与训练**。

### 2.1 启动 Runtime 容器并构建 SDK

首次使用先显式构建镜像，再启动并进入 `runtime` 容器：

```bash
./env.sh build runtime
./env.sh runtime
```

在 `runtime` 容器内一条命令生成 ROS 2 接口工作空间与 Policy Protobuf/gRPC SDK：

```bash
scripts/build_sdk.sh
```

### 2.2 启动 MuJoCo 容器与试采示教后端

在宿主机新终端中构建并进入 `mujoco` 容器：

```bash
./env.sh build mujoco
./env.sh mujoco
```

在 `mujoco` 容器内直接以方案启动示教后端（显式指定机器人/后端/任务；`--seed` 未传时随机，`--run-id` 默认 `teacher-backend`，启动时会打印所用配置文件路径与完整生效参数）：

```bash
scripts/teacher_backend.sh --robot marvin --backend mujoco --task stack
```

降低仿真速度时使用 `--realtime-speed`。其他仿真参数在方案的 Teacher 后端配方中修改。

```bash
scripts/teacher_backend.sh --robot marvin --backend mujoco --task stack --seed 0 --run-id teacher-backend --realtime-speed 0.4
```

### 2.3 试采时开启实时 MuJoCo 可视化（可选）

`viewer` 独立订阅后端发布的只读物理状态快照，不参与物理步进与控制。在宿主机桌面环境下（若报 X11 权限错误，先在宿主机执行 `xhost +SI:localuser:$(id -un)`）：

```bash
./env.sh viewer --robot marvin --backend mujoco --task stack
```

### 2.4 单路试采一次，确认完整链路

回到 `runtime` 容器终端，使用刚启动的手动 teacher backend 试采一次。试采指定 `pilot`，不占正式采集名额：

```bash
# 只试一次：合格即结束，失败也立即结束
scripts/record.sh \
  --robot marvin --backend mujoco --task stack --purpose pilot --count 1 --max-consecutive-failures 1
```

确认试采命令退出码为 0，末尾 `采集结束：目标达成 | 合格 1/1`；本局应同时满足任务成功、录制完整、导出质量合格。

### 2.5 停止试采后端和 viewer

试采合格后，在 viewer 终端（若已开启）和手动 teacher backend 终端分别按 `Ctrl-C`，释放计算资源。保留 `runtime` 容器运行；`mujoco` 容器可保持空闲，后续第 3.4 节仍会使用它启动推理后端。

### 2.6 宿主机并行采集，补足 50 条合格正式数据

在**宿主机新终端的项目根目录**执行：

```bash
./record_parallel.sh \
  --robot marvin --backend mujoco --task stack --purpose formal --target-total 50 \
  --workers auto --max-consecutive-failures 3
```

`--target-total 50` 表示正式台账累计达到 50 条合格数据：已有正式数据时只补足差额；已经达到目标则直接成功退出。配置、backend 日志和性能汇总保存到 `output/marvin/mujoco/stack/logs/collection/record-pools/pool_*/`。

### 2.7 查看采集与正式清单状态

在 **runtime 容器终端**查看合格条数、磁盘占用与正式清单状态：

```bash
scripts/record.sh status --robot marvin --backend mujoco --task stack
```

确认清单状态显示为 `SELECTED` 后，即可进入第 3 节的转换与训练。

---

## 3. 训练与策略推理流程

### 3.1 启动 LeRobot 容器

在宿主机新终端中启动并进入 `lerobot` 容器：

```bash
./env.sh lerobot
```

### 3.2 转换数据集与训练 ACT 模型

在 `lerobot` 容器内读取 `/data/marvin/mujoco/stack/datasets/raw/formal-select.json`。转换器按 `act_dataset.yaml` 的 `validation_percent` 随机抽取整局作为验证集（默认 10%，50 条合格局划分为 45 条训练和 5 条验证），划分结果写入 `/data/marvin/mujoco/stack/datasets/lerobot/manifest.json`。

```bash
scripts/convert.sh --robot marvin --backend mujoco --task stack
```

训练使用共享官方 ResNet18 权重文件 `third_party/weights/resnet18-f37072fd.pth`（按离线方式运行）；也可在容器内用 `RESNET18_WEIGHTS` 指定现有权重文件。

启动完整模式训练（默认 5000 步，启用 PyAV 视频解码、张量批处理、多进程预取与 CUDA bfloat16 混合精度）：

```bash
scripts/train.sh --robot marvin --backend mujoco --task stack --mode full
```

训练直接写出可部署模型目录 `output/marvin/mujoco/stack/models/act_<YYYYMMDD_HHMMSS>/`（包含 `model.safetensors` 与 `training_args.yaml`），训练事件、指标和终态摘要保存在 `output/marvin/mujoco/stack/logs/training/act_<YYYYMMDD_HHMMSS>/`。

### 3.3 直接启动 Policy 推理服务

在 `lerobot` 容器内启动 gRPC Policy 服务；省略 `--model` 时自动选择 `/data/marvin/mujoco/stack/models/` 下最新的 `act_*` 模型目录，也可显式指定模型目录或 `model.safetensors`：

```bash
scripts/server_policy.sh --robot marvin --backend mujoco --task stack
# 或显式指定模型
scripts/server_policy.sh --robot marvin --backend mujoco --task stack \
  --model /data/marvin/mujoco/stack/models/act_<YYYYMMDD_HHMMSS>
```

服务默认监听 `50051`，日志目录为 `/data/marvin/mujoco/stack/logs/policy-service`。保持该终端运行。

### 3.4 直接启动推理仿真后端

回到 `mujoco` 容器，以 policy 角色启动后端（自动生成 `inference_backend_<robot>_<backend>_<task>_<YYYYMMDD_HHMMSS>` 作为 `run_id`，`--seed` 未传时随机）：

```bash
scripts/inference_backend.sh \
  --robot marvin --backend mujoco --task stack
```

就绪标记、运行快照与日志分别位于 `/data/marvin/mujoco/stack/logs/backend/inference_backend_marvin_mujoco_stack_<YYYYMMDD_HHMMSS>/ready.json`、`backend.yaml` 与 `log`。后端与第 3.3 节的 Policy 服务、第 3.5 节的 runtime 需同时在线。

### 3.5 直接运行推理或交互闭环

回到 `runtime` 容器，启动一局交互式闭环（输入 `y` 复位环境并执行一局，结束后保持 `IDLE` 直至 `Ctrl-C` 退出）：

```bash
scripts/client_interactive.sh \
  --robot marvin --backend mujoco --task stack \
  --seed 6000
```

非交互式单局执行使用 `scripts/client_policy.sh`，录制默认落到 `/data/marvin/mujoco/stack/logs/recordings/<run-id>`：

```bash
scripts/client_policy.sh \
  --robot marvin --backend mujoco --task stack \
  --reset-seed 6000 --run-id policy-run
```

如需无人工干预的**并行**批量仿真推理测试，在**本机项目根目录**执行（需保持第 3.3 节 `lerobot` 容器的 `scripts/server_policy.sh` 在线）：

```bash
./inference_parallel.sh \
  --robot marvin --backend mujoco --task stack \
  --count 10 --seed-start 6000 --workers auto
```

如需将录制的单局或批量推理 MCAP 合成为多视角可视化 MP4（默认保存到同目录下的 `<目录名>.mp4`），可在宿主机项目根目录运行：

```bash
./recording_to_video.sh output/marvin/mujoco/stack/logs/recordings/policy-run
```

---

## 4. 使用 marvin/mujoco/handover

`marvin/mujoco/handover` 与 `marvin/mujoco/stack` 使用完全相同的入口，只需替换 `--task`：

```bash
# mujoco：示教后端
scripts/teacher_backend.sh --robot marvin --backend mujoco --task handover --seed 0 --run-id teacher-backend

# 宿主机（可选）：MuJoCo 实时可视化
./env.sh viewer --robot marvin --backend mujoco --task handover

# runtime：试采
scripts/record.sh --robot marvin --backend mujoco --task handover \
  --purpose pilot --count 1 --max-consecutive-failures 1

# 宿主机项目根目录：并行补足 50 条正式数据
./record_parallel.sh --robot marvin --backend mujoco --task handover \
  --purpose formal --target-total 50 --workers auto

# lerobot：正式转换与 full 训练
scripts/convert.sh --robot marvin --backend mujoco --task handover
scripts/train.sh --robot marvin --backend mujoco --task handover --mode full

# 推理：lerobot 策略服务、mujoco 后端、runtime 交互闭环
scripts/server_policy.sh --robot marvin --backend mujoco --task handover
scripts/inference_backend.sh --robot marvin --backend mujoco --task handover --seed 5100
scripts/client_interactive.sh --robot marvin --backend mujoco --task handover --seed 5100

# 本机项目根目录：并行批量推理
./inference_parallel.sh --robot marvin --backend mujoco --task handover \
  --count 10 --seed-start 5100 --workers auto
```

方案详细说明见 [`shared/scenarios/marvin/mujoco/handover/README.md`](shared/scenarios/marvin/mujoco/handover/README.md)。

---

## 5. 路线图与待办事项（Real2Sim & RL）

系统后续按“**先完成天机硬件 Real2Sim 仿真对齐，再落地独立强化学习端 `learning/rl/`**”推进，完整方案与共识详见 [天机 Real2Sim 标定与实施方案](docs/features/天机Real2Sim标定与实施方案.md)、[强化学习架构与 Real2Sim 协同设计](docs/features/强化学习架构与Real2Sim协同设计.md) 与 [架构扩展与演进指南](docs/features/架构扩展与演进指南.md)。

### 阶段一：天机真机 Real2Sim 仿真重构与 RL 接口预留
- [ ] 核对实验室末端/台架 CAD 与指垫实测尺寸，按零历史包袱原则原地升级 `shared/robots/marvin/` 与 MuJoCo 装配（碰撞解析几何与可选 STL 视觉网格解耦，参见 [天机 Real2Sim 标定与实施方案](docs/features/天机Real2Sim标定与实施方案.md)）
- [ ] 执行天机真机标定与 Sys-ID：控制器 `forward_kinematics` 对账 + 桌面 ChArUco/AprilTag 多相机手眼联合标定 + TCP 四角点触校验 + 悬空小幅激励动态辨识
- [ ] 全仓统一 3 路相机契约为 `cam_high` / `cam_left_wrist` / `cam_right_wrist`（真机 `cv2.remap` 去畸变居中下采样至 `320×240` 对齐 MuJoCo `fovy`），统一 90C `[0.0, 0.090] m` 导轨行程契约、软指垫接触物理与语义末端句柄（`tcp_site_ids`、`finger_pad_body_ids`、`pad_aperture_m`）
- [ ] 在 100% 零 `rclpy` 依赖的纯 Python 物理与伺服步进层复刻天机动态约束（延迟缓冲、APEX `1.0°` 限幅、90C `0.3 mm` 死区），迁移适配 `stack` 与 `handover` 双基准任务并内建三层可开关域随机化
- [ ] 上提共享纯 Python 轨迹制动核，打通 `hardware/tianji` ROS 2 后端（`robot_api_msgs`）、跨仿真/真机通用的单手柄微分 IK 遥操（预留工作站“边拖边录 + 脚踏板”程序导入位）与四项 Real2Sim 定量验收门禁

### 阶段二：强化学习（`learning/rl`）三位一体接入与无损数据集升级
- [ ] 抽离公共 `shared/common/python/policy_serving/` gRPC 服务层，升级 `binding.yaml` 支持多学习范式槽位（`learning.act`、`learning.rl` 等）与 `models/<范式前缀>_<时间戳>/` 产物约定
- [ ] 升级 LeRobot Parquet 数据集写出器：支持内嵌无损 PNG 字节列，并补齐双臂 EE 6D 位姿、`task` 文本与 `reward` / `done` 列（兼顾 ACT、RL 与外部自定义 VLA 无损转换）
- [ ] 搭建基于 `TorchRL` + `TensorDict` 的 `learning/rl/` 与 Colab 50+ GB 全内存并发环境（`docs/colab/scripts/setup_colab_rl.sh`），率先落地特权状态 **RL-as-Teacher** 自动化示教与无损采数
- [ ] 落地与 ACT 平级的**非对称动作块视觉 RL 策略**，实现 Colab 真实物理成功率在线选模并通过统一 `PolicyService` 完成本机三端闭环验收
- [ ] 落地 **ACT 宏步残差后训练器（Chunk-Residual SAC）** 与 Colab 一键训练/同步工作流（`train_rl_colab.ipynb`）

---

目录导航：[文档索引](docs/README.md) · [脚本入口与使用指南](docs/features/脚本入口与使用指南.md) · [跨端架构分工与解耦审查](docs/features/跨端架构分工与解耦审查.md) · [架构扩展与演进指南](docs/features/架构扩展与演进指南.md) · [天机 Real2Sim 标定与实施方案](docs/features/天机Real2Sim标定与实施方案.md) · [强化学习架构与 Real2Sim 协同设计](docs/features/强化学习架构与Real2Sim协同设计.md) · [重要实践与经验](docs/experiments/README.md) · [公共层与方案绑定](shared/README.md) · [Agent 指导与规范](AGENTS.md)
