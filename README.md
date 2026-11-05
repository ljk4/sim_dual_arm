# sim_dual_arm

面向双臂机器人的仿真数据采集、ACT 训练与策略执行项目。以 MuJoCo 双 Panda
堆叠任务作为首个场景，验证从程序示教到学习策略闭环的完整过程，并为后续
切换真机保留统一的机器人接口。

目前已接通采集、训练、推理和反馈链路，并完成一次真实堆叠回归；跨场景可靠性、
真机驱动和双臂协作仍待验证。最新阶段统一维护在[总目标与当前阶段](docs/project-status.md)。

## 系统组成

```mermaid
flowchart LR
    B[MuJoCo 后端 / 后续真机后端] <-->|ROS 2 Robot API| R[runtime：观测、控制与录制]
    R <-->|gRPC Policy API| P[ACT 推理服务]
    R --> D[原始记录与合格数据集]
    D --> T[ACT 训练]
    T --> M[模型包]
    M --> P
```

| 部分 | 责任 | 主要位置 |
| --- | --- | --- |
| simulation | MuJoCo 物理步进、传感器、执行参考、实测反馈、任务判定和程序示教 | `simulation/mujoco/` |
| runtime | 固定身份的观测、控制会话、动作块 ACK/接续、录制、审计与导出 | `runtime/ros2/` |
| learning | 数据转换、ACT 训练、模型包和 ROS-free Policy 服务 | `learning/lerobot/` |
| hardware | 真机统一 Robot API 边界；驱动尚未实现 | `hardware/` |
| contracts | 公共 ROS/Policy 接口、数据格式和语义校验 | `contracts/` |

真机接入时替换机器人后端，应用仍使用同一 Robot API。当前物理执行使用 MuJoCo +
Panda；CPU mock 只验证软件链路，mock 记录不作为正式训练数据。真机驱动、相机和
硬件在环尚未实现，边界见 [hardware/README.md](hardware/README.md)，不提供假驱动或
空框架。学习侧当前固定为 LeRobot ACT；替换其他学习框架时保持 Policy API、三路
RGB + 16 维实测输入和 60×16 位置动作块语义，不改运行端对具体模型的依赖。

## 环境与目录

当前 ROS 侧以 Ubuntu 24.04 / ROS 2 Jazzy / Python 3.12 为基线；学习侧直接继承
官方 `huggingface/lerobot-gpu:latest`。ROS 与机器学习环境分开，
镜像只装依赖，源码、配置、SDK、资产、数据和模型通过挂载提供。

| 目录 | 内容 |
| --- | --- |
| `simulation/`、`hardware/`、`runtime/`、`learning/` | 仿真、真机边界、运行和学习实现；各自持有源码、配置、`docker/` 与 `scripts/` |
| `libraries/` | 有明确调用方的共享实现 |
| `contracts/`、`profiles/` | 接口定义、机器人关节/相机/动作语义 |
| `infrastructure/` | 公共 Docker 文件、容器内构建/SDK 工具和公开示例 |
| `tests/` | 单元、集成测试和测试替身 |
| `docs/` | 目标、规范、功能说明、重要实践与阶段历史 |
| `third_party/` | 固定的外部来源；下载源码在忽略的 `src/` 下 |
| `var/` | 数据、实验、模型、台账与运行日志，不进 Git |
| `.build/` | 构建、SDK、开发环境与隔离工作树，不进 Git |

维护前阅读[项目规范](docs/standards.md)。各目录 README 说明本地职责和入口。

## 一次完整闭环

完整流程只包含两部分：**数据采集** 与 **训练/推理**。三个服务分别长期运行：
仿真端负责环境和 Robot API，学习端负责训练与 Policy 服务，运行端负责观测、控制、
录制和重复运行。下面的终端 A/B/C/D 可分别保持打开。

### 一、数据采集

#### 1. 准备运行端 SDK（终端 A）

~~~bash
cd /path/to/sim_dual_arm/runtime/ros2/docker
docker compose build
docker compose up -d
docker compose exec runtime bash
runtime/ros2/scripts/run.sh build-ros-sdk
runtime/ros2/scripts/run.sh build-policy-sdk
~~~

#### 2. 启动程序示教后端（终端 B）

~~~bash
cd /path/to/sim_dual_arm/simulation/mujoco/docker
docker compose build
docker compose up -d
docker compose exec simulation bash
scripts/run.sh prepare --template /app/configs/backend/backend_stack_teacher.yaml --seed 0 --run-id teacher-backend --output /data/prepared/teacher-backend
scripts/run.sh backend --config /data/prepared/teacher-backend/backend.yaml --run-id teacher-backend
~~~

#### 3. 打开实时 MuJoCo 可视化（终端 D，可选）

仿真 Compose 会自动读取宿主图形会话中的 `DISPLAY` 和 `XAUTHORITY`，正常桌面环境
不需要手动执行 `export` 或 `echo`。如果 viewer 报 X11 权限错误，在宿主执行一次
`xhost +SI:localuser:$(id -un)` 后重试。

~~~bash
cd /path/to/sim_dual_arm/simulation/mujoco/docker
docker compose exec simulation bash

# 与终端 B 使用同一个 backend.yaml
scripts/run.sh viewer --config /data/prepared/teacher-backend/backend.yaml
~~~

viewer 是独立进程，只订阅 `/robot/visualization/mujoco_state` 的完整物理快照，
不参与控制、物理步进和采集判定。关闭窗口或 `Ctrl-C` 不会停止终端 B 的后端。

#### 4. 采集 50 条合格数据（终端 A）

每次尝试由台账分配 seed；示教运行、录制、导出和质量判定都在容器内完成。循环在获得
50 条合格 episode 后结束。

~~~bash
mkdir -p /data/campaigns/panda_stack_v1
until runtime/ros2/scripts/run.sh campaign select > /data/campaigns/panda_stack_v1/formal-select.json; do
  runtime/ros2/scripts/run.sh campaign run --purpose formal --mode teacher -- python3 /workspace/runtime/ros2/scripts/collect_stack_attempt.py || exit 1
done
~~~

采集结束后，在终端 B 按 `Ctrl-C` 停止示教后端。

### 二、训练与推理

#### 1. 转换、训练并生成模型包与 Policy 服务配置（终端 C）

~~~bash
cd /path/to/sim_dual_arm/learning/lerobot/docker
docker compose build
docker compose up -d
docker compose exec learning bash
set -e -o pipefail
learning/lerobot/scripts/run.sh convert
TRAIN_RUN=$(learning/lerobot/scripts/run.sh train --mode full | python3 -c 'import json,sys; print(json.load(sys.stdin)["run_dir"])')
learning/lerobot/scripts/run.sh bundle --mode full --run "$TRAIN_RUN" --output /data/learning/bundles/stack-act
learning/lerobot/scripts/run.sh prepare-policy --template /workspace/learning/lerobot/configs/act_policy_temporal.yaml --bundle /data/learning/bundles/stack-act --run-id infer-5k-server --output /data/prepared/infer-5k-server/policy
~~~

`prepare-policy` 会生成后续必需的：

```text
/data/prepared/infer-5k-server/policy/policy.yaml
/data/prepared/infer-5k-server/policy/policy_binding.json
```

输出目录只能创建一次；更换 bundle 或重做配置时使用新的 `--run-id`，不要覆盖已存在的目录。

#### 2. 启动无示教任务后端（终端 B）

~~~bash
# simulation 容器内
scripts/run.sh prepare --template /app/configs/backend/backend_stack_act_chunked.yaml --seed 20001 --run-id infer-5k-server --output /data/prepared/infer-5k-server/simulation
scripts/run.sh backend --config /data/prepared/infer-5k-server/simulation/backend.yaml --run-id infer-5k-server
~~~

#### 3. 打开实时可视化（终端 D）

~~~bash
# 新的 simulation 容器终端
scripts/run.sh viewer --config /data/prepared/infer-5k-server/simulation/backend.yaml
~~~

viewer 使用与推理后端相同的场景和配置，显示机器人、物体和任务执行的实时状态。

#### 4. 启动 Policy 推理服务（终端 C）

Policy 服务的日志目录必须不存在。每次重启服务时生成新目录；启动前先确认
`/data/prepared/infer-5k-server/policy/policy.yaml` 已存在。

~~~bash
# learning 容器内
POLICY_LOG="/data/logs/infer-5k-server-policy-$(date +%Y%m%dT%H%M%SZ)"
test ! -e "$POLICY_LOG"

learning/lerobot/scripts/run.sh policy \
  --config /data/prepared/infer-5k-server/policy/policy.yaml \
  --run-id infer-5k-server \
  --log-directory "$POLICY_LOG"
~~~

服务加载模型需要十几秒。可在另一个 learning 容器终端检查就绪事件：

~~~bash
grep -R '"event":"policy.ready"' /data/logs/infer-5k-server-policy-* 2>/dev/null | tail -5
~~~

看到 `policy.ready` 后，再执行下一节的 runtime 交互命令。

#### 5. 准备运行端配置（终端 A，仅一次）

~~~bash
# runtime 容器内
test -f /data/prepared/infer-5k-server/policy/policy_binding.json

runtime/ros2/scripts/run.sh prepare \
  --template /workspace/runtime/ros2/configs/native_task_act_chunked.yaml \
  --policy-binding /data/prepared/infer-5k-server/policy/policy_binding.json \
  --seed 20001 \
  --run-id infer-5k-server \
  --output /data/prepared/infer-5k-server/runtime
~~~

该命令只执行一次；正常运行阶段复用生成的
`/data/prepared/infer-5k-server/runtime/runtime.yaml`。

#### 6. 交互式运行推理（终端 A，每次任务重新执行）

每个任务重新运行一次 `interactive`。脚本会自动生成唯一的 `run-id`、`episode-id`
和原始记录目录，并在 `y` 后通过 `StartEpisode` 发送环境复位指令：

- MuJoCo 后端：机器人回 home，夹爪复位，物体位置和堆叠任务状态复位。
- 真机后端：只将机器人复原到 home，不复位外部环境。

~~~bash
runtime/ros2/scripts/run.sh interactive --config /data/prepared/infer-5k-server/runtime/runtime.yaml --seed 20001
~~~

提示：

~~~text
输入 y 复原环境并开始本次任务，其他键取消:
~~~

输入 `y` 后执行一局。任务结束后进程保持 IDLE 并打印 episode 结果和录像目录，
按 `Ctrl-C` 结束本次 runtime。同一仿真后端和 Policy 服务可继续复用；下一次任务
只需重新执行：

~~~bash
runtime/ros2/scripts/run.sh interactive --config /data/prepared/infer-5k-server/runtime/runtime.yaml --seed 20002
~~~

不要复用同一个手工 `run-id` 或 `episode-id`；interactive 自动生成的 ID 已避免
`FileExistsError` 和 `REQUEST_ID_CONFLICT`。失败记录和旧录像保留，不覆盖。


## 常见问题

### `policy.yaml: No such file or directory`

说明只准备了 simulation/runtime，没有执行 `prepare-policy`。在 learning 容器补执行：

~~~bash
cd /workspace
learning/lerobot/scripts/run.sh prepare-policy \
  --template /workspace/learning/lerobot/configs/act_policy_temporal.yaml \
  --bundle /data/learning/bundles/stack-act \
  --run-id infer-5k-server \
  --output /data/prepared/infer-5k-server/policy
~~~

如果输出目录已存在，说明已经生成过，直接检查其中的 `policy.yaml` 和
`policy_binding.json`，不要重复覆盖。

### `FileExistsError: /data/logs/<name>`

Policy 服务的日志目录只能创建一次。重新启动服务时换一个新的时间戳目录：

~~~bash
POLICY_LOG="/data/logs/infer-5k-server-policy-$(date +%Y%m%dT%H%M%SZ)"
learning/lerobot/scripts/run.sh policy \
  --config /data/prepared/infer-5k-server/policy/policy.yaml \
  --run-id infer-5k-server \
  --log-directory "$POLICY_LOG"
~~~

只在确认旧目录只是失败启动记录时才手工删除它；不要删除成功服务的日志目录。

### `Failed to add port to server: ... 0.0.0.0:50051`

表示 Policy 端口已被占用，通常是上一次服务没有退出。检查并停止旧进程：

~~~bash
ps -eo pid,cmd | grep '[d]ual_arm_learning.act_service'
kill -TERM <PID>
sleep 2
ss -ltnp | grep 50051
~~~

`ss` 没有输出表示端口已释放。随后使用新的 `--log-directory` 重新启动服务。

### viewer 报 X11 权限错误

仿真 Compose 会自动读取宿主图形会话的 `DISPLAY` 和 `XAUTHORITY`。若仍无法连接，
在宿主机执行：

~~~bash
xhost +SI:localuser:$(id -un)
~~~

然后重新打开 viewer。无图形环境的宿主仍可运行 backend 和推理，只是不能使用 viewer。

### `FileExistsError`、`REQUEST_ID_CONFLICT` 或录制目录冲突

不要复用同一个手工 `run-id`、`episode-id` 或输出目录。推理运行时使用：

~~~bash
runtime/ros2/scripts/run.sh interactive \
  --config /data/prepared/infer-5k-server/runtime/runtime.yaml \
  --seed 20002
~~~

`interactive` 会自动生成唯一身份；失败记录和旧录像保留，不覆盖。
