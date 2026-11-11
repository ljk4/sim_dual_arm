# sim_dual_arm

面向双臂机器人的仿真数据采集、ACT 策略训练与流式控制闭环系统。当前基于 MuJoCo 双 Panda 机械臂堆叠任务，实现从程序示教（Teacher）、LeRobot ACT 训练到有界跟随控制（Ruckig `SubmitFollower`）的完整闭环。

## 架构概览

仓库按职能划分为三个容器化运行端与公共契约层：

- **`simulation/mujoco/`**：MuJoCo 物理仿真后端，负责双 Panda 动力学步进、三路 EGL 相机渲染、Ruckig 轨迹规划、接触判定与程序示教，通过 ROS 2 接口对外提供服务。
- **`runtime/ros2/`**：ROS 2 运行端，负责多路观测同步组装、控制租约与流式动作块调度、MCAP 录制、离线审计与评估。
- **`learning/lerobot/`**：ROS-free 学习与策略服务端，负责数据集转换、ACT 混合精度训练、模型包（Bundle）导出与 gRPC Policy 推理服务。
- **`contracts/` & `profiles/` & `libraries/`**：跨端共享的 Protobuf/ROS 2 IDL、JSON Schema、Pydantic 契约模型、机器人配置档案与基础库。

---

## 1. 环境准备

所有宿主机命令默认在仓库根目录下执行。

```bash
git clone https://github.com/ljk4/sim_dual_arm.git
cd sim_dual_arm
```

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
runtime/ros2/scripts/run.sh build-ros-sdk
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
  --template /app/configs/backend/backend_stack_teacher.yaml \
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
  --purpose pilot --count 1 --max-consecutive-failures 1
```

确认试采命令退出码为 0，末尾 `recording.finished` 的 `reason` 为 `target_reached`、`qualified` 为 1；本局应同时满足任务成功、录制完整、导出质量合格。若失败，先检查日志并修复，再重新执行本节的试采命令。

`--count` 统计本次新增合格条数；`--max-consecutive-failures 1` 让试采最多尝试一次。每次试采继续记录在原台账并占用 pilot 预算，种子自动递增；pilot 数据不会进入后面的正式训练/验证清单。

### 2.5 停止试采后端和 viewer

试采合格后，在 viewer 终端（若已开启）和手动 teacher backend 终端分别按 `Ctrl-C`，释放计算资源。保留 runtime 容器运行；simulation 容器可保持空闲，后续第 3.4 节仍会使用它启动推理后端。

### 2.6 宿主机并行采集，补足 50 条合格正式数据

在**宿主机新终端的项目根目录**执行：

```bash
runtime/ros2/scripts/record_parallel.sh \
  --purpose formal --target-total 50 \
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
learning/lerobot/scripts/run.sh convert
```

转换时终端只显示配置、日志路径和原地刷新的进度，包括已完成条数、百分比、用时和预计剩余时间；编码器等详细输出写入日志。训练使用同样的显示方式，额外显示 step、loss 和验证状态。重定向输出时仅保留阶段摘要，避免生成大量进度行。可用 `--log-file /data/logs/<文件名>.log` 指定详细日志；需要原始输出和 JSON 结果时添加 `--no-progress`。

启动完整模式训练（默认启用 PyAV 视频解码、张量批处理、多进程预取与 CUDA bfloat16 混合精度）：

```bash
learning/lerobot/scripts/run.sh train --mode full
```

训练完成后，记录终端输出的 `run_dir` 并导出可部署的 safetensors 模型包（Bundle）：

```bash
TRAIN_RUN="/data/learning/runs/<实际生成的-run-目录名>"
learning/lerobot/scripts/run.sh bundle \
  --mode full \
  --run "$TRAIN_RUN" \
  --output /data/learning/bundles/stack-act
```

### 3.3 生成 Policy 服务配置与绑定清单

在 `learning` 容器内校验模型包，生成 `policy.yaml`、`policy_binding.json`、`act_contract.json` 与 `run_manifest.json`：

```bash
learning/lerobot/scripts/run.sh prepare-policy \
  --template /workspace/learning/lerobot/configs/policy_temporal.yaml \
  --bundle /data/learning/bundles/stack-act \
  --run-id infer-5k-server \
  --output /data/prepared/infer-5k-server/policy
```

### 3.4 准备并启动推理仿真后端

回到 `simulation` 容器，使用分块 ACT 推理模板生成仿真后端配置并启动：

```bash
cd /app
scripts/run.sh prepare \
  --template /app/configs/backend/backend_stack_act_chunked.yaml \
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
  --template /workspace/runtime/ros2/configs/policy_rollout.yaml \
  --policy-binding /data/prepared/infer-5k-server/policy/policy_binding.json \
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

## 4. 配置管理与联动规则

`/data/prepared/...` 下的生成文件是带哈希校验的运行快照，**切勿直接手动修改**。需要调整参数时，请修改仓库中的源配置文件，并使用新的 `--run-id` 或输出目录重新执行准备命令。

### 4.1 配置依赖链

```text
模型包 (/data/learning/bundles/<bundle>)
  + learning/lerobot/configs/policy_temporal.yaml
  ──[learning: prepare-policy]──> /data/prepared/<run-id>/policy/{policy.yaml, policy_binding.json}

policy_binding.json
  + runtime/ros2/configs/policy_rollout.yaml
  ──[runtime: prepare]──────────> /data/prepared/<run-id>/runtime/runtime.yaml

仿真后端模板 (simulation/mujoco/configs/backend/*)
  + simulation/mujoco/configs/scene/*
  + profiles/control/*
  ──[simulation: prepare]───────> /data/prepared/<run-id>/simulation/backend.yaml
```

### 4.2 源配置索引与更新操作

| 配置项 | 源文件路径 | 修改后的更新步骤 |
| --- | --- | --- |
| 示教后端模板 | `simulation/mujoco/configs/backend/backend_stack_teacher.yaml` | 重新执行 simulation `prepare`，重启 `backend` |
| 推理后端模板 | `simulation/mujoco/configs/backend/backend_stack_act_chunked.yaml` | 重新执行 simulation `prepare`，重启 `backend` 与 `viewer` |
| 物理场景、物体、相机 | `simulation/mujoco/configs/scene/panda_stack_learning.yaml` | 重新执行 simulation `prepare`，重启 `backend` 与 `viewer`；新旧场景数据不可混训 |
| 堆叠成功判定与示教参数 | `simulation/mujoco/configs/scene/panda_stack_task.yaml` | 重新执行 simulation `prepare`，重启 `backend` |
| 物理控制限值与静止/接触容差 | `profiles/control/panda_control_v1.yaml` | 同时重新执行 simulation `prepare` 与 runtime `prepare`，重启 `backend` 与 `runtime` |
| 流式动作块超时与卸载过渡 | `profiles/control/panda_stream_v1.yaml` | 重新执行 simulation `prepare`，重启 `backend` |
| 跟随窗口 `target_count` 与偏差预算 | `profiles/control/panda_follower_v1.yaml` | 同时重新执行 simulation `prepare` 与 runtime `prepare`，重启 `backend` 与 `runtime` |
| 策略运行端模板 | `runtime/ros2/configs/policy_rollout.yaml` | 重新执行 runtime `prepare`，重新运行 `interactive` |
| 示教运行端模板 | `runtime/ros2/configs/teacher_collect.yaml` | 下一次 `record.sh` 自动生成新配置，无需手动 `prepare` |
| Policy 服务与时间融合模板 | `learning/lerobot/configs/policy_temporal.yaml` | 重新执行 `prepare-policy` 与 runtime `prepare`，重启 `policy` 服务 |
| 数据集转换规则 | `learning/lerobot/configs/act_dataset.container.yaml` | 使用新输出目录重新执行 `convert` |
| 训练超参数与 DataLoader/AMP | `learning/lerobot/configs/act_training.container.yaml` | 重新执行 `train` → `bundle` → `prepare-policy` → runtime `prepare` |
| ROS 2 消息或服务定义 | `contracts/ros/robot_api_msgs/` | 重新执行 `build-ros-sdk`，重启 `simulation` 与 `runtime` |
| Policy Protobuf 协议 | `contracts/policy/v1/policy.proto` | 重新执行 `build-policy-sdk`，重启 `learning` 与 `runtime` |
| 机器人 Profile 与观测/动作 Schema | `profiles/`、`contracts/schemas/v1/` | 重建 SDK，重新校验数据集、模型包与三端运行配置 |

---

## 5. 常见问题排查

- **`policy.yaml: No such file or directory`**：尚未执行 `prepare-policy` 或路径有误。检查 `/data/prepared/<run-id>/policy/` 是否包含 `policy.yaml` 与 `policy_binding.json`。
- **`FileExistsError` / `REQUEST_ID_CONFLICT`**：准备目录、日志目录或录制目录禁止覆盖复用。每次重启 Policy 服务请使用新的 `--log-directory`，每次重新准备配置请使用新的 `--run-id`，单局策略测试优先使用 `interactive` 自动生成唯一身份。
- **`Failed to add port to server: ... 0.0.0.0:50051`**：旧的 Policy 服务进程未退出。在 `learning` 容器内执行 `ps -eo pid,cmd | grep '[d]ual_arm_learning.policy_service_cli'` 查找 PID 并通过 `kill -TERM <PID>` 终止，确认 `ss -ltnp | grep 50051` 无输出后使用新日志目录重启。
- **`viewer` 无法打开窗口**：确认已通过 `docker compose run` 显式挂载 `/tmp/.X11-unix` 与 `.Xauthority` 并传递 `DISPLAY` 环境变量，且宿主机已执行 `xhost +SI:localuser:$(id -un)`。无图形界面的服务器可正常运行 `backend`、采集与推理。

更多文档请参阅 [文档索引](docs/README.md)、[项目状态](docs/project-status.md) 与 [项目规范](docs/standards.md)。

架构目录与 A1～A7 迁移说明见 [架构执行记录](docs/architecture-tasks/implementation-2026-11-10.md)。ROS 接口调整后请在空闲时重建 SDK，并重建使用新源码挂载与环境变量的 simulation/runtime 容器。
