# sim_dual_arm

面向双臂机器人的仿真数据采集、ACT 策略训练与流式控制闭环系统。当前基于 MuJoCo 双 Panda 机械臂堆叠任务，实现从程序示教（Teacher）、LeRobot ACT 训练到有界跟随控制（Ruckig `SubmitFollower`）的完整闭环。

## 架构概览

仓库按职能划分为三个容器化运行端、真机边界与公共/方案层：

- **`simulation/mujoco/`**：MuJoCo 物理仿真后端，负责双 Panda 动力学步进、三路 EGL 相机渲染、Ruckig 轨迹规划、接触判定与程序示教，通过 ROS 2 接口对外提供服务。
- **`runtime/ros2/`**：ROS 2 运行端，负责多路观测同步组装、控制租约与流式动作块调度、MCAP 录制、离线审计与评估。
- **`learning/lerobot/`**：ROS-free 学习与策略服务端，负责数据集转换、ACT 混合精度训练、模型包（Bundle）导出与 gRPC Policy 推理服务。
- **`hardware/`**：真机 Robot API 边界说明；当前没有可验证的真实驱动。
- **`shared/common/`**：公共层，只放跨端接口约束（Policy Protobuf、ROS 2 IDL、JSON Schema）、共享 Python 包与通用工具，不选择具体任务、机器人或算法。
- **`shared/scenarios/<id>/`**：方案绑定层，用 `binding.yaml` 选择档案、配方与方案专用接口。当前实例为 [`panda_stack`](shared/scenarios/panda_stack/README.md)（MuJoCo + ROS 2 + LeRobot ACT）；[`_template`](shared/scenarios/_template/README.md) 说明替换步骤。Marvin 接入目标见[任务文档](docs/marvin-handover/README.md)。

四端（simulation/runtime/learning/hardware）持有实际实现、依赖、容器和入口脚本；公共协议与当前 Panda/ACT/MuJoCo 实例数值分开维护，方案选择只发生在 `prepare`/`build-ros-sdk` 等准备入口。

---

## 1. 环境准备

所有宿主机命令默认在仓库根目录下执行。

```bash
git clone https://github.com/ljk4/sim_dual_arm.git
cd sim_dual_arm

# 首次启动容器前，以当前宿主机用户创建输出根目录
mkdir -p build var
```

`build/` 存放生成的 SDK、构建结果和缓存，`var/` 存放运行日志、录制、数据集、模型与台账。两者整体由 Git 忽略，不提交空目录或 `.gitkeep`；各构建和运行命令按需创建子目录。首次启动前由宿主机用户创建这两个根目录，避免 Docker 自动创建后归 root 所有；UID/GID 应与 Compose 中的运行用户一致。

runtime/learning 容器内 `/build` 与 `/workspace/build` 指向宿主机同一个 `build/`；`/data` 与 `/workspace/var` 指向同一个 `var/`。前者供容器工具使用，后者供项目相对路径使用。simulation 仅挂载所需的 SDK 与 `/data`。更新挂载配置后，重新执行对应服务的 `docker compose up -d` 使配置生效。

仓库不附带已生成的 SDK、采集数据、训练权重或模型包。下文的输出路径用于生成和使用本机产物；历史实验记录不代表当前机器已有这些文件。

学习端另外使用固定版本的 LeRobot 源码，首次使用须按[第三方源码说明](third_party/README.md)准备；容器镜像不会代为恢复宿主机已清理的源码目录。

---

## 2. 数据采集流程

按顺序完成：**单路试采一次（pilot，可观察可视化）→ 停止手动后端与 viewer → 宿主机并行补足 50 条合格正式数据 → 筛选 45 条训练 / 5 条验证 → 第 3 节转换与训练**。

### 2.1 启动 Runtime 容器并构建 SDK

构建并后台启动 `runtime` 容器：

```bash
docker compose -f runtime/ros2/docker/docker-compose.yml build
docker compose -f runtime/ros2/docker/docker-compose.yml up -d
docker compose -f runtime/ros2/docker/docker-compose.yml exec runtime bash
```

在 `runtime` 容器内生成 ROS 2 接口工作空间与 Policy Protobuf/gRPC SDK：

```bash
cd /workspace
runtime/ros2/scripts/run.sh build-ros-sdk --scenario panda_stack --shared-root /shared
runtime/ros2/scripts/run.sh build-policy-sdk
```

### 2.2 启动 Simulation 容器与试采示教后端

在宿主机新终端中构建并进入 `simulation` 容器：

```bash
docker compose -f simulation/mujoco/docker/docker-compose.yml build
docker compose -f simulation/mujoco/docker/docker-compose.yml up -d
docker compose -f simulation/mujoco/docker/docker-compose.yml exec simulation bash
```

在 `simulation` 容器内生成示教后端配置（每个输出目录仅生成一次，拒绝覆盖）并前台启动后端：

```bash
cd /app
scripts/run.sh prepare \
  --scenario panda_stack --role teacher \
  --seed 0 \
  --run-id teacher-backend \
  --output /data/prepared/teacher-backend

scripts/run.sh backend \
  --config /data/prepared/teacher-backend/backend.yaml \
  --run-id teacher-backend
```

### 2.3 试采时开启实时 MuJoCo 可视化（可选）

`viewer` 独立订阅后端发布的只读物理状态快照，不参与物理步进与控制。在宿主机桌面环境下（若报 X11 权限错误，先在宿主机执行 `xhost +SI:localuser:$(id -un)`）：

```bash
cd simulation/mujoco/docker
docker compose run --rm \
  -e DISPLAY="$DISPLAY" \
  -e XAUTHORITY=/tmp/.Xauthority \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${XAUTHORITY:-/dev/null}:/tmp/.Xauthority:ro" \
  simulation scripts/run.sh viewer \
  --config /data/prepared/teacher-backend/backend.yaml
```

### 2.4 单路试采一次，确认完整链路

回到 `runtime` 容器终端，使用刚启动的手动 teacher backend 试采一次。试采指定 `pilot`，不占用正式 50 条的名额：

```bash
mkdir -p /data/campaigns/panda_stack_v1
runtime/ros2/scripts/run.sh campaign status

# 只试一次：合格即结束，失败也立即结束
runtime/ros2/scripts/record.sh \
  --scenario panda_stack --purpose pilot --count 1 --max-consecutive-failures 1
```

确认试采命令退出码为 0，末尾 `recording.finished` 的 `reason` 为 `target_reached`、`qualified` 为 1；本局应同时满足任务成功、录制完整、导出质量合格。若失败，先检查日志并修复，再重新执行本节的试采命令。

`--count` 统计本次新增合格条数；`--max-consecutive-failures 1` 让试采最多尝试一次。每次试采继续记录在原台账并占用 pilot 预算，种子自动递增；pilot 数据不会进入后面的正式训练/验证清单。

### 2.5 停止试采后端和 viewer

试采合格后，在 viewer 终端（若已开启）和手动 teacher backend 终端分别按 `Ctrl-C`，释放计算资源。保留 runtime 容器运行；simulation 容器可保持空闲，后续第 3.4 节仍会使用它启动推理后端。

### 2.6 宿主机并行采集，补足 50 条合格正式数据

在**宿主机新终端的项目根目录**执行：

```bash
runtime/ros2/scripts/record_parallel.sh \
  --scenario panda_stack --purpose formal --target-total 50 \
  --workers auto --max-consecutive-failures 3
```

`--target-total 50` 表示正式台账累计达到 50 条合格数据：已有正式数据时只补足差额；已经达到目标则直接成功退出。若中断或连续失败提前结束，修复原因后再次执行相同命令即可续采，不需要手动计算剩余条数或清空台账。该参数不能与表示“新增条数”的 `--count` 同时指定。

入口自动创建独立的 MuJoCo backend，复用 runtime 容器并行录制与导出。每路使用独立的 ROS domain、命名空间和日志目录，共用原 campaign 的种子和预算。自动档按 CPU、可用内存和显存选择至多 4 路；如需固定并发，将 `--workers auto` 改为 `--workers 4`。

并行 worker 禁用 viewer 状态发布、预览 PNG 和 GIF，保留训练 RGB、标签及质量门禁。连续失败按完成顺序计数，合格一次清零；连续 3 次不合格会取消在途任务并提前退出，取消项仍占预算。预算/种子不足也会停止。按 `Ctrl-C` 可中断本批次；命令结束时自动清理本批次后台实例。配置、backend 日志和性能汇总保存到 `var/record-pools/pool-*/`。

### 2.7 筛选正式数据，生成训练/验证清单

并行采集结束后，回到 **runtime 容器终端**检查正式合格数并生成清单：

```bash
cd /workspace
runtime/ros2/scripts/run.sh campaign status

# 仅筛选成功后替换正式清单，避免数量不足时覆盖已有文件
runtime/ros2/scripts/run.sh campaign select \
  > /data/campaigns/panda_stack_v1/formal-select.json.tmp &&
mv /data/campaigns/panda_stack_v1/formal-select.json.tmp \
  /data/campaigns/panda_stack_v1/formal-select.json
```

确认筛选成功且清单中的 `status` 为 `SELECTED`、`qualified` 为 50，再进入第 3 节。清单只选择合格 formal 数据，自动划分为 45 条训练 / 5 条验证，排除 pilot 试采。

---

## 3. 训练与策略推理流程

### 3.1 启动 Learning 容器

在宿主机新终端中构建并进入 `learning` 容器：

```bash
docker compose -f learning/lerobot/docker/docker-compose.yml build
docker compose -f learning/lerobot/docker/docker-compose.yml up -d
docker compose -f learning/lerobot/docker/docker-compose.yml exec learning bash
```

### 3.2 转换数据集与训练 ACT 模型

完成第 2.7 节后，在 `learning` 容器内将正式清单中的 50 条合格 episode 转换为 LeRobot 数据集（45 条训练 / 5 条验证，输出至 `/data/learning/panda_stack_learning_v1`）。转换配置直接读取 `/data/campaigns/panda_stack_v1/formal-select.json`，无需移动原始数据：

```bash
cd /workspace
learning/lerobot/scripts/run.sh convert --scenario panda_stack
```

转换时终端只显示配置、日志路径和原地刷新的进度，包括已完成条数、百分比、用时和预计剩余时间；编码器等详细输出写入日志。训练使用同样的显示方式，额外显示 step、loss 和验证状态。重定向输出时仅保留阶段摘要，避免生成大量进度行。可用 `--log-file /data/logs/<文件名>.log` 指定详细日志；需要原始输出和 JSON 结果时添加 `--no-progress`。

训练前将官方 ResNet18 权重文件 `resnet18-f37072fd.pth` 放到宿主机 `build/cache/torch/hub/checkpoints/`；也可在 learning 容器内用 `RESNET18_WEIGHTS` 指定现有权重文件。训练按离线方式运行，缺少权重时会报错，不会自动下载。

启动完整模式训练（默认启用 PyAV 视频解码、张量批处理、多进程预取与 CUDA bfloat16 混合精度）：

```bash
learning/lerobot/scripts/run.sh train --scenario panda_stack --mode full
```

训练完成后，记录终端输出的 `run_dir` 并导出可部署的 safetensors 模型包（Bundle）：

```bash
TRAIN_RUN="/data/learning/runs/<实际生成的-run-目录名>"
learning/lerobot/scripts/run.sh bundle \
  --scenario panda_stack --mode full \
  --run "$TRAIN_RUN" \
  --output /data/learning/bundles/stack-act
```

### 3.3 生成 Policy 服务配置与绑定清单

在 `learning` 容器内按方案绑定校验模型包，生成 `policy.yaml`、`policy_binding.json`、`act_contract.json` 与 `run_manifest.json`：

```bash
learning/lerobot/scripts/run.sh prepare-policy \
  --scenario panda_stack --shared-root /shared \
  --bundle /data/learning/bundles/stack-act \
  --run-id infer-5k-server \
  --output /data/prepared/infer-5k-server/policy
```

直接配置模式继续可用：把 `--scenario panda_stack --shared-root /shared` 换成 `--template /workspace/shared/scenarios/panda_stack/configs/learning/policy_temporal.yaml`，两者互斥。

### 3.4 准备并启动推理仿真后端

回到 `simulation` 容器，使用分块 ACT 推理模板生成仿真后端配置并启动：

```bash
cd /app
scripts/run.sh prepare \
  --scenario panda_stack --role policy \
  --bundle /data/learning/bundles/stack-act \
  --seed 20001 \
  --run-id infer-5k-server \
  --output /data/prepared/infer-5k-server/simulation

scripts/run.sh backend \
  --config /data/prepared/infer-5k-server/simulation/backend.yaml \
  --run-id infer-5k-server
```

### 3.5 启动 Policy 推理服务

回到 `learning` 容器，使用全新的日志目录前台启动 gRPC Policy 服务：

```bash
POLICY_LOG="/data/logs/infer-5k-server-policy-$(date -u +%Y%m%dT%H%M%SZ)"
learning/lerobot/scripts/run.sh policy \
  --config /data/prepared/infer-5k-server/policy/policy.yaml \
  --run-id infer-5k-server \
  --log-directory "$POLICY_LOG"
```

### 3.6 准备并交互式运行 Runtime

回到 `runtime` 容器，将 `policy_binding.json` 与运行模板合成为 `runtime.yaml`（`--run-id` 必须与 `prepare-policy` 一致）：

```bash
runtime/ros2/scripts/run.sh prepare \
  --scenario panda_stack --role policy \
  --policy-binding /data/prepared/infer-5k-server/policy/policy_binding.json \
  --bundle /data/learning/bundles/stack-act \
  --seed 20001 \
  --run-id infer-5k-server \
  --output /data/prepared/infer-5k-server/runtime
```

启动交互式闭环评估（自动分配唯一 `run-id` 与 `episode-id`，输入 `y` 复位环境并执行一局，结束后保持 IDLE 直至按 `Ctrl-C` 退出）：

```bash
runtime/ros2/scripts/run.sh interactive \
  --config /data/prepared/infer-5k-server/runtime/runtime.yaml \
  --seed 20001
```

后续测试不同随机种子时，直接传入新的 `--seed` 即可，无需重新执行 `prepare`：

```bash
runtime/ros2/scripts/run.sh interactive \
  --config /data/prepared/infer-5k-server/runtime/runtime.yaml \
  --seed 20002
```

---
