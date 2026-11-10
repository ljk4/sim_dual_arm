# sim_dual_arm

面向双臂机器人的仿真数据采集、ACT 训练与策略执行项目。当前使用 MuJoCo 双 Panda
堆叠任务，验证从程序示教到 ACT 策略闭环的完整流程。

完整闭环包含两部分：

1. 数据采集
2. 训练与推理

以下所有宿主机命令都从 `sim_dual_arm` 仓库根目录执行。

## 1. 获取代码

含义：把仓库克隆到本地。

```bash
git clone https://github.com/ljk4/sim_dual_arm.git
```

含义：进入仓库根目录；后续命令默认都在这里执行。

```bash
cd sim_dual_arm
```

含义：确认当前位于仓库根目录。

```bash
pwd
```

## 2. 数据采集

### 2.1 构建并启动 runtime 容器

含义：构建 runtime 镜像。

```bash
docker compose -f runtime/ros2/docker/docker-compose.yml build
```

含义：后台启动 runtime 容器。

```bash
docker compose -f runtime/ros2/docker/docker-compose.yml up -d
```

含义：进入 runtime 容器，后续 runtime/SDK 命令在这个终端执行。

```bash
docker compose -f runtime/ros2/docker/docker-compose.yml exec runtime bash
```

### 2.2 生成运行端 SDK

含义：生成仿真端需要的 ROS SDK。

```bash
cd /workspace
runtime/ros2/scripts/run.sh build-ros-sdk
```

含义：生成学习端 Policy SDK，供 runtime 校验 Policy 服务配置。

```bash
runtime/ros2/scripts/run.sh build-policy-sdk
```

### 2.3 构建并启动 simulation 容器

含义：在宿主机新终端进入仓库根目录。

```bash
cd sim_dual_arm
```

含义：构建 simulation 镜像。

```bash
docker compose -f simulation/mujoco/docker/docker-compose.yml build
```

含义：后台启动 simulation 容器。

```bash
docker compose -f simulation/mujoco/docker/docker-compose.yml up -d
```

含义：进入 simulation 容器，后续仿真命令在这个终端执行。

```bash
docker compose -f simulation/mujoco/docker/docker-compose.yml exec simulation bash
```

### 2.4 准备并启动程序示教后端

含义：进入 simulation 容器的工作目录。

```bash
cd /app
```

含义：从教师模板生成 teacher backend 配置；输出目录只能新建一次。

```bash
scripts/run.sh prepare --template /app/configs/backend/backend_stack_teacher.yaml --seed 0 --run-id teacher-backend --output /data/prepared/teacher-backend
```

含义：前台启动教师后端，保持运行。

```bash
scripts/run.sh backend --config /data/prepared/teacher-backend/backend.yaml --run-id teacher-backend
```

### 2.5 打开实时 MuJoCo 可视化（可选）

含义：在宿主机新终端进入 simulation 容器。

```bash
docker compose -f simulation/mujoco/docker/docker-compose.yml exec simulation bash
```

含义：启动 viewer；它只读取后端完整物理快照，不参与控制。

```bash
cd /app
scripts/run.sh viewer --config /data/prepared/teacher-backend/backend.yaml
```

viewer 使用的是仿真端 X11 显示。正常图形桌面环境下 Compose 会自动读取宿主
`DISPLAY` 和 `XAUTHORITY`。若报 X11 权限错误，在宿主机执行：

```bash
xhost +SI:localuser:$(id -un)
```

### 2.6 开始采集

含义：回到 runtime 容器终端，创建采集台账目录。

```bash
mkdir -p /data/campaigns/panda_stack_v1
```

含义：查看当前采集预算和成功数量。

```bash
runtime/ros2/scripts/run.sh campaign status
```

含义：运行一次正式的教师采集。每次执行只尝试一局，失败也计入台账。

```bash
runtime/ros2/scripts/run.sh campaign run --purpose formal --mode teacher -- python3 /workspace/runtime/ros2/scripts/collect_attempt.py
```

含义：重复运行上一行，直到台账中累计 50 条合格正式 episode。

```bash
runtime/ros2/scripts/run.sh campaign status
```

含义：从已完成的合格尝试中选出 45 条训练、5 条验证，并生成
`formal-select.json`。

```bash
runtime/ros2/scripts/run.sh campaign select > /data/campaigns/panda_stack_v1/formal-select.json
```

采集完成后，在教师后端终端按 `Ctrl-C` 停止 backend。

## 3. 训练与推理

### 3.1 构建并启动 learning 容器

含义：在宿主机新终端进入仓库根目录。

```bash
cd sim_dual_arm
```

含义：构建 learning 镜像。

```bash
docker compose -f learning/lerobot/docker/docker-compose.yml build
```

含义：后台启动 learning 容器。

```bash
docker compose -f learning/lerobot/docker/docker-compose.yml up -d
```

含义：进入 learning 容器，后续训练和 Policy 命令在这个终端执行。

```bash
docker compose -f learning/lerobot/docker/docker-compose.yml exec learning bash
```

### 3.2 转换数据

含义：进入 learning 容器的工作目录。

```bash
cd /workspace
```

含义：把采集导出的合格 episode 转成 45/5 的 LeRobot 训练和验证数据集。

```bash
learning/lerobot/scripts/run.sh convert
```

转换会再次校验 manifest、时间对齐、动作标签、图像、质量和哈希。转换输出位于：

```text
/data/learning/panda_stack_learning_v1
```

### 3.3 训练 ACT

含义：运行 full 模式训练，终端最后会输出包含 `run_dir` 的 JSON。

```bash
learning/lerobot/scripts/run.sh train --mode full
```

训练默认用 PyAV 解码视频、DataLoader 默认 `default_collate` 直接产出批张量，
并启用 `num_workers`、`pin_memory`、`prefetch_factor` 和 CUDA AMP（bfloat16）。
这些参数在 `learning/lerobot/configs/act_training*.yaml` 的 `training` 段显式配置。

从输出中记录 `run_dir`，例如：

```text
/data/learning/runs/full-20261105T061827Z-1227723
```

含义：把上一步输出的 `run_dir` 写入当前 shell 变量。

```bash
TRAIN_RUN=/data/learning/runs/full-20261105T061827Z-1227723
```

### 3.4 导出模型包

含义：把训练 run 导出为可部署的 ACT bundle。

```bash
learning/lerobot/scripts/run.sh bundle --mode full --run "$TRAIN_RUN" --output /data/learning/bundles/stack-act
```

### 3.5 生成 Policy 服务配置

含义：校验 bundle，并生成 `policy.yaml`、`policy_binding.json` 和 `act_contract.json`。

```bash
learning/lerobot/scripts/run.sh prepare-policy --template /workspace/learning/lerobot/configs/policy_temporal.yaml --bundle /data/learning/bundles/stack-act --run-id infer-5k-server --output /data/prepared/infer-5k-server/policy
```

该命令每套 bundle 只执行一次。输出目录已存在时不要重复覆盖；需要新配置时使用新的
`--run-id`。

### 3.6 准备并启动推理后端

含义：回到 simulation 容器，从推理模板生成 backend 配置。

```bash
scripts/run.sh prepare --template /app/configs/backend/backend_stack_act_chunked.yaml --seed 20001 --run-id infer-5k-server --output /data/prepared/infer-5k-server/simulation
```

含义：前台启动无示教任务后端，保持运行。

```bash
scripts/run.sh backend --config /data/prepared/infer-5k-server/simulation/backend.yaml --run-id infer-5k-server
```

### 3.7 打开推理实时可视化（可选）

含义：在宿主机新终端进入 simulation 容器并启动 viewer。

```bash
docker compose -f simulation/mujoco/docker/docker-compose.yml exec simulation bash
```

```bash
cd /app
scripts/run.sh viewer --config /data/prepared/infer-5k-server/simulation/backend.yaml
```

### 3.8 启动 Policy 推理服务

含义：回到 learning 容器。Policy 服务的日志目录必须不存在，每次重启使用新目录。

```bash
POLICY_LOG="/data/logs/infer-5k-server-policy-$(date +%Y%m%dT%H%M%SZ)"
```

含义：确认本次日志目录尚不存在。

```bash
test ! -e "$POLICY_LOG"
```

含义：前台启动 Policy 服务，保持运行。

```bash
learning/lerobot/scripts/run.sh policy --config /data/prepared/infer-5k-server/policy/policy.yaml --run-id infer-5k-server --log-directory "$POLICY_LOG"
```

含义：在另一个 learning 容器终端确认服务已就绪。

```bash
grep -R '"event":"policy.ready"' /data/logs/infer-5k-server-policy-* 2>/dev/null | tail -5
```

### 3.9 准备 runtime 运行配置

含义：回到 runtime 容器，确认 Policy binding 已生成。

```bash
test -f /data/prepared/infer-5k-server/policy/policy_binding.json
```

含义：把 Policy binding 和 runtime 模板组合成 runtime 配置。

```bash
runtime/ros2/scripts/run.sh prepare --template /workspace/runtime/ros2/configs/policy_rollout.yaml --policy-binding /data/prepared/infer-5k-server/policy/policy_binding.json --seed 20001 --run-id infer-5k-server --output /data/prepared/infer-5k-server/runtime
```

该命令只执行一次；后续任务复用生成的
`/data/prepared/infer-5k-server/runtime/runtime.yaml`。

### 3.10 交互式运行 ACT

含义：启动一次 runtime，输入 `y` 后复原环境并执行一局。

```bash
runtime/ros2/scripts/run.sh interactive --config /data/prepared/infer-5k-server/runtime/runtime.yaml --seed 20001
```

终端会显示：

```text
输入 y 复原环境并开始本次任务，其他键取消:
```

任务结束后进程保持 IDLE，并打印 episode 结果和录像目录。按 `Ctrl-C` 结束本次
runtime。

含义：下一次任务使用新的 seed，重新执行 interactive。

```bash
runtime/ros2/scripts/run.sh interactive --config /data/prepared/infer-5k-server/runtime/runtime.yaml --seed 20002
```

仿真端复位包括机器人、夹爪、物体和任务状态；真机后端只需要复位机器人到 home，
不复位外部环境。

不要复用同一个手工 `run-id` 或 `episode-id`。`interactive` 会自动生成唯一身份，
避免 `FileExistsError`、`REQUEST_ID_CONFLICT` 和录制目录冲突。

## 4. 配置位置与修改后的操作

不要直接编辑 `/data/prepared/...` 下生成的 `backend.yaml`、`runtime.yaml` 或
`policy.yaml`。它们是某次运行的身份快照，后续校验和审计都以生成时内容为准。
需要修改时，改仓库中的源模板，再重新生成新的运行配置。

### 4.1 配置依赖关系

```text
learning bundle
  + learning/lerobot/configs/policy_temporal.yaml
  -> learning prepare-policy
  -> /data/prepared/<run-id>/policy/policy.yaml
  -> /data/prepared/<run-id>/policy/policy_binding.json

policy_binding.json
  + runtime/ros2/configs/policy_rollout.yaml
  -> runtime prepare
  -> /data/prepared/<run-id>/runtime/runtime.yaml

simulation backend 模板
  + simulation/mujoco/configs/scene/*
  + profiles/control/*
  -> simulation prepare
  -> /data/prepared/<run-id>/simulation/backend.yaml
```

### 4.2 源配置所在位置

| 要修改的内容 | 源配置文件 | 改完后需要做什么 |
| --- | --- | --- |
| 数据采集后端模板 | `simulation/mujoco/configs/backend/backend_stack_teacher.yaml` | 重新执行 simulation `prepare`，然后用新配置重启 backend |
| 推理后端模板 | `simulation/mujoco/configs/backend/backend_stack_act_chunked.yaml` | 重新执行 simulation `prepare`，然后重启推理 backend 和 viewer |
| 场景、物体、相机 | `simulation/mujoco/configs/scene/panda_stack_learning.yaml` | 重新执行 simulation `prepare`，重启 backend 和 viewer；旧数据不要与新场景混训 |
| 任务成功阈值 | `simulation/mujoco/configs/scene/panda_stack_task.yaml` | 重新执行 simulation `prepare`，重启 backend；旧任务结果仍按生成时的 digest 保留 |
| 物理控制限值、容差 | `profiles/control/panda_control_v1.yaml` | 同时重新执行 simulation `prepare` 和 runtime `prepare`，重启 backend 与 runtime |
| chunk stream 超时 | `profiles/control/panda_stream_v1.yaml` | 重新执行 simulation `prepare`，重启 backend |
| follower 窗口和偏差预算 | `profiles/control/panda_follower_v1.yaml` | 同时重新执行 simulation `prepare` 和 runtime `prepare`，重启 backend 与 runtime |
| runtime 策略模板 | `runtime/ros2/configs/policy_rollout.yaml` | 重新执行 runtime `prepare`，重启 runtime |
| 数采 runtime 模板 | `runtime/ros2/configs/teacher_collect.yaml` | 下一次 `campaign run` 会重新生成该 attempt 的 runtime 配置，无需手工 prepare |
| Policy 服务模板 | `learning/lerobot/configs/policy_temporal.yaml` | 重新执行 learning `prepare-policy`，再用新 binding 重新执行 runtime `prepare`，重启 Policy 服务和 runtime |
| 训练参数 | `learning/lerobot/configs/act_training.container.yaml` | 下一次 `train` 使用新参数；训练后重新 bundle、prepare-policy、runtime prepare |
| 数据转换、划分、质量规则 | `learning/lerobot/configs/act_dataset.container.yaml` | 重新执行 `convert`；已有数据集目录不会覆盖，需使用新的输出目录或 run-id |
| 模型 bundle | `/data/learning/bundles/<bundle>` | 重新执行 learning `prepare-policy`，再重新执行 runtime `prepare`，重启 Policy 服务和 runtime |
| ROS 消息或服务定义 | `contracts/ros/robot_api_msgs/` | 重建 ROS SDK 和 Policy SDK，重启三端；动作/观测语义变化时还需要重新采集和训练 |
| 机器人 profile、观测/动作 schema | `profiles/` | 重建 SDK，重新校验模型、训练数据和 runtime；通常需要重新训练 |

### 4.3 几个容易遗漏的联动

**修改 follower 的 `target_count`**

唯一数据源是 `profiles/control/panda_follower_v1.yaml` 的 `target_count`；runtime
配置通过 `follower_config` 直接引用该文件，不再内联重复定义。修改后重新执行
simulation 和 runtime 两个 `prepare`，并重启 backend 和 runtime。

**修改 `panda_control_v1.yaml`**

这个文件同时被 backend 和 runtime 引用。只重新生成 backend 配置是不够的，必须同时
重新生成 runtime 配置。通常还需要重启三端中的 backend 和 runtime。

**更换 Policy bundle**

只需重新执行 `learning prepare-policy` 和 runtime `prepare`，然后重启 Policy 服务和
runtime。backend、场景和 viewer 不需要因为换模型而重启。

**修改 seed**

推理时可以直接通过 interactive 覆盖：

```bash
runtime/ros2/scripts/run.sh interactive --config /data/prepared/infer-5k-server/runtime/runtime.yaml --seed 20002
```

这种情况不需要重新执行 runtime `prepare`。如果修改的是 backend 模板里的默认 seed，
则重新执行 simulation `prepare`。

### 4.4 通用规则

1. 先生成到新的输出目录，不要覆盖旧配置。
2. 生成目录已存在时，使用新的 `--run-id` 或新的 `--output`。
3. `learning prepare-policy` 和 runtime `prepare` 的 `--run-id` 必须一致。
4. 修改配置后，先重启对应后端/服务，再运行 `interactive`。
5. viewer 读取的是启动时传入的 `backend.yaml`；backend 换配置后，viewer 也要用新配置重启。
6. 旧 `/data/raw`、`/data/prepared` 和 `/data/logs` 保留用于审计，不要为了重跑而覆盖。

## 5. 常见问题

### 5.1 `policy.yaml: No such file or directory`

说明缺少 `prepare-policy` 产物。重新执行：

```bash
learning/lerobot/scripts/run.sh prepare-policy --template /workspace/learning/lerobot/configs/policy_temporal.yaml --bundle /data/learning/bundles/stack-act --run-id infer-5k-server --output /data/prepared/infer-5k-server/policy
```

如果输出目录已经存在，先检查其中的 `policy.yaml` 和 `policy_binding.json`，不要覆盖。

### 5.2 `FileExistsError: /data/logs/<name>`

Policy 服务日志目录只能创建一次。使用新的时间戳目录：

```bash
POLICY_LOG="/data/logs/infer-5k-server-policy-$(date +%Y%m%dT%H%M%SZ)"
```

```bash
learning/lerobot/scripts/run.sh policy --config /data/prepared/infer-5k-server/policy/policy.yaml --run-id infer-5k-server --log-directory "$POLICY_LOG"
```

### 5.3 `Failed to add port to server: ... 0.0.0.0:50051`

端口 `50051` 已被占用，通常是旧的 Policy 服务没有退出。

含义：查找残留服务进程。

```bash
ps -eo pid,cmd | grep '[d]ual_arm_learning.policy_service_cli'
```

含义：停止查到的旧进程，把 `<PID>` 替换成实际进程号。

```bash
kill -TERM <PID>
```

含义：确认端口已经释放；没有输出表示空闲。

```bash
ss -ltnp | grep 50051
```

然后使用新的 `--log-directory` 重新启动 Policy 服务。

### 5.4 viewer 报 X11 权限错误

含义：授权当前宿主用户访问 X11。

```bash
xhost +SI:localuser:$(id -un)
```

无图形环境的宿主仍可运行 backend、采集和推理，只是不能显示 viewer。

### 5.5 `FileExistsError`、`REQUEST_ID_CONFLICT` 或录制目录冲突

不要复用旧的 `run-id`、`episode-id` 或输出目录。推理时直接使用 `interactive`
自动生成唯一身份；失败记录和录像保留，不覆盖。
