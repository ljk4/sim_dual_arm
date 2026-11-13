# P02A 交付记录：标准控制接入与执行映射

日期：2026-11-12。前置 P01A 已交付。本轮在 Marvin 场景上接通真实 Robot API：后端按场景选择装配，具名 I/O 读写左右臂与总开度，controller/motion 通过能力契约工作，并取得单臂、双臂、夹爪、越限、运行中停止和有界收尾的实际执行证据。任务插件、Teacher、真实持物、双臂 IK 和采集训练未在本轮启动。

工作树改动保留，未暂存、提交或推送。

## 环境与命令

- 仓库：`/home/ljk/work/sim_dual_arm`，分支 `main`。运行容器：`sim-dual-arm-simulation-1`（镜像 `sim-dual-arm/simulation:current`，用户 `1000:1000`，MuJoCo `3.17.0`，Python `3.12.3`，`MUJOCO_GL=egl`）。
- 挂载：`simulation/mujoco` → `/app:ro`，`shared` → `/shared:ro`，`var` → `/data`，`build/ros-dev` → `/opt/ros-dev:ro`。
- 回归测试容器：从 `simulation/mujoco/docker` 以 `docker compose run --rm --no-deps -v <repo>:/workspace:ro -e PROJECT_ROOT=/workspace ...` 启动，避免使用镜像内旧源码。
- 本轮活动 SDK：只有 `robot_api_msgs` 与 `sim_api_msgs`，`package_set_sha256=303a1f6807e69bdf5b4ec0c5b49cb4ef75b7995203c871065fd5252d33609e56`。构建与校验：

```bash
# runtime 容器 /workspace；显式包集，core 自动包含 robot_api_msgs
runtime/ros2/scripts/run.sh build-ros-sdk --shared-root /shared \
  --package sim_api_msgs

# simulation 容器 /app；校验当前活动 SDK 标记
python3 /shared/common/tools/scripts/dev_workspace.py check \
  --project-root /app --build-root /opt/ros-dev
# contracts ok: robot_api_msgs, sim_api_msgs (303a1f68...)
```

构建前已确认没有占用旧 SDK 的 backend/runtime 进程。Panda 示教需要 `panda_stack_api_msgs`；切回前用 `build-ros-sdk --scenario panda_stack --shared-root /shared` 重建并重新核对标记，不能沿用本轮的 core+sim 包集。

## 无任务插件的准备与启动

`shared/scenarios/marvin_handover/configs/simulation/backend/backend_marvin_control.yaml` 是本轮 direct `--template` 配方；`robot.task` 和 `robot.task_mode` 同时省略。准备与启动：

```bash
# simulation 容器 /app
scripts/run.sh prepare \
  --template /shared/scenarios/marvin_handover/configs/simulation/backend/backend_marvin_control.yaml \
  --seed 0 --run-id p02a-marvin --output /data/prepared/p02a-marvin
scripts/run.sh backend \
  --config /data/prepared/p02a-marvin/backend.yaml --run-id p02a-standalone
```

宿主产物：`var/prepared/p02a-marvin/{backend.yaml,run_manifest.json}`；standalone 运行：`var/runs/backend/p02a-standalone/status.json`，`exit_code=0`、`success=true`、`phase=IDLE`、`captures=114`、`reason=SIGTERM`、`renderer_closed=true`。`run.json` 记录 `robot_assembly=marvin`、`robot_model=/app/assets/marvin/urdf/Marvin_Stand_M6_2026.4.8.urdf`，不再写死 Panda 模型；有效配置中 `task` 与 `task_mode` 均为 `None`。

## 职责修复

### 场景与机器人选择

- [config/scene_loader.py](../../../simulation/mujoco/src/robot_backend_sim/config/scene_loader.py) 按场景配方中显式的 `robot.assembly` 选择严格配置类型；Panda 场景配方新增 `robot: {assembly: panda}`，Marvin 场景沿用 `robot.assembly: marvin`。任务名不参与机器人推断。
- [physics/robot_adapters.py](../../../simulation/mujoco/src/robot_backend_sim/physics/robot_adapters.py) 是该显式选择点：`panda`/`marvin` 分别注册场景构造器和 I/O 实现。Worker、owner、controller 不按机器人名分支。
- [runtime/worker.py](../../../simulation/mujoco/src/robot_backend_sim/runtime/worker.py) 不再固定 `load_scene_config`；[runtime/worker_owner.py](../../../simulation/mujoco/src/robot_backend_sim/runtime/worker_owner.py) 不再固定 `build_scene`/`PandaIO`，改为构造所选场景与 I/O 后交给同一个 `ControlSession`。线程归属、同一场景对象、启动失败收尾和契约验证保持不变。

### 具名 I/O 与总开度

- 新增 [physics/robot_io.py](../../../simulation/mujoco/src/robot_backend_sim/physics/robot_io.py)：`RobotIO` 能力契约、公共 16 维顺序、`Measurement`、profile/相机/目标校验。
- [physics/panda_io.py](../../../simulation/mujoco/src/robot_backend_sim/physics/panda_io.py) 改为实现该契约，原有按名解析、限位/执行器/相机校验和速度前馈保留。
- 新增 [physics/marvin/io.py](../../../simulation/mujoco/src/robot_backend_sim/physics/marvin/io.py)：以 `MarvinBinding` 的具名 qpos/dof/actuator 地址读写，公开顺序为左七轴、左总开度、右七轴、右总开度。物理 qpos 右臂在前，实测验证 public 索引 0 对应 `Arm_L1_Joint`，不使用相邻切片。
- 总开度复用 `gripper_opening`（两指内侧面垂直间距）和 `MarvinScene.gripper_ctrl`（`closure = open_total - opening`）。开度速度取两指关节速度和的相反数，正方向仍表示张开。动作执行器为闭合量；速度前馈对臂使用 `+kd/kp*qdot`，对夹爪使用 `-kd/kp*opening_dot`，量纲与符号由直接执行测试确认。
- `open_total=0.0814 m` 与闭端 `0.0014 m` 来自 P01A；实测复位开度 `0.0814368 m`（高出档案 3.7e-5 m），因此控制容差不取自 P01A 的 `5e-4 m` 离线一致性容差。

### 控制、运动与接触

- [config/motion_config.py](../../../simulation/mujoco/src/robot_backend_sim/config/motion_config.py) 与 [config/follower_config.py](../../../simulation/mujoco/src/robot_backend_sim/config/follower_config.py) 的类改名为 `ControlConfig`/`FollowerConfig`，只验证维数、唯一性和数值；不再强制 Panda 名称/顺序。启动时在 [motion/motion_math.py](../../../simulation/mujoco/src/robot_backend_sim/motion/motion_math.py) 校验配置、profile 与 I/O 的具名顺序一致，并保留有限值、profile 限位、时间网格和流配置验证。
- [control/control.py](../../../simulation/mujoco/src/robot_backend_sim/control/control.py)、[motion/motion_controller.py](../../../simulation/mujoco/src/robot_backend_sim/motion/motion_controller.py) 和 [motion/motion_runtime.py](../../../simulation/mujoco/src/robot_backend_sim/motion/motion_runtime.py) 改用 `CompiledScene`/`RobotIO`；夹爪索引由关节名解析，不再使用 `7/15` 邻接常量。
- [physics/contacts.py](../../../simulation/mujoco/src/robot_backend_sim/physics/contacts.py) 不再内置 Panda 手指名：`resolve_contact_binding` 接收机器人适配提供的两指 `(body, joint)` 具名绑定，通用代码按名测量每指接触与速度。`validated_gripper_completions` 保留“左后右、每侧两指”的完成规则。空爪反馈和接触跟踪因此可用于 Marvin；真实持物留给 P02B。
- `CompiledScene` 新增 `contact_object_names` 能力；Panda 场景返回两个自由物块，Marvin 场景返回自由物块 `block`（箱体固定），任务名不参与接触对象推断。

## 配方与实测参数

新增：

| 文件 | 内容 |
| --- | --- |
| `profiles/control/marvin_control_v1.yaml` | 16 维名称、最大加速度、实测位置/速度容差、稳定/超时和接触阈值 |
| `profiles/control/marvin_stream_v1.yaml` | 初始提前量、最小提前量、观测年龄、流/进展/跟踪超时 |
| `profiles/control/marvin_follower_v1.yaml` | 每窗 20 点、参考偏差预算、终端尾长 |
| `configs/simulation/backend/backend_marvin_control.yaml` | direct backend 配方；无 `task`/`task_mode`；启用速度前馈 |

实测后确定的容差：

| 项目 | 实测 | 采用阈值 |
| --- | --- | --- |
| 臂位置跟踪 | 单臂/双臂窗口最大误差 `3.51e-3 rad`（含保持臂基座漂移） | `0.010 rad` |
| 臂速度跟踪 | 最大误差 `1.42e-2 rad/s` | `0.02 rad/s` |
| 夹爪位置跟踪 | 闭合/打开最大误差约 `4e-5 m` | `0.001 m` |
| 夹爪速度跟踪 | 最大误差约 `2.6e-5 m/s` | `0.005 m/s` |
| follower 参考偏差 | 臂 `6.9e-3 rad`，夹爪 `7.2e-3 m` | `0.05 rad` / `0.01 m` |

加速度 `1.0 rad/s²`、夹爪 `0.5 m/s²` 与 stream 的 `0.25 s` 初始提前量在本次真实流中均被接受并完成；Ruckig 目标 jerk 仍为无限（未引入新的 jerk 限制）。停止时限 `3 s` 内完成制动收尾。

## 真实执行证据

证据目录：`var/reports/p02a/p02a-022c0db8/`（容器 `/data/reports/p02a/p02a-022c0db8/`），`evidence.json` 保存每个窗口的原始 `StreamRequest`、接受 `FollowerReference`、`RobotState` 序列、`ExecutionFeedback` 序列和终态。后端运行记录：`var/reports/p02a/runs/p02a-022c0db8/{status.json,log/}`。

运行命令（工作目录仓库根）：

```bash
docker compose -f simulation/mujoco/docker/docker-compose.yml run --rm --no-deps \
  -v /home/ljk/work/sim_dual_arm:/workspace:ro \
  -v /home/ljk/work/sim_dual_arm/var:/data \
  -e PROJECT_ROOT=/workspace -w /workspace simulation bash -lc \
  'source /shared/common/tools/docker/setup.sh && \
   python3 -m pytest -p no:cacheprovider -q tests/integration/test_marvin_control_ros.py -s'
```

测试从 `/data/prepared/p02a-marvin/backend.yaml` 复制配置，只替换唯一 ROS namespace 并固定 `realtime_speed=1.0`；场景、profile、control、stream、follower 路径仍指向本配方。结果 `2 passed in 12.98s`。窗口摘要：

| 窗口 | 命令/参考 | 实测 | 终态 |
| --- | --- | --- | --- |
| left-arm | 左 `joint1..7` 各 `+0.05 rad` | 左 `0.0465..0.0503 rad`，右漂移最大 `3.5e-3 rad` | `COMPLETED` |
| both-arms | 左/右 `joint3` 到 `0.10 rad` | 左 `0.10020`、右 `0.10030 rad` | `COMPLETED` |
| left-close | 左总开度 `0.0814→0.0414 m` | 终态 `0.041437 m` | `COMPLETED` |
| left-open | 左总开度 `0.0414→0.0814 m` | 终态 `0.081435 m` | `COMPLETED` |
| right-close | 右总开度 `0.0814→0.0414 m` | 终态 `0.041422 m`（左保持 `0.081435`） | `COMPLETED` |
| right-open | 右总开度 `0.0414→0.0814 m` | 终态 `0.081421 m` | `COMPLETED` |
| over-limit | `left_joint1=4.0 rad`（档案上限 `3.1067`） | 拒绝，位置 0.2 s 内不变（`1e-6`） | `accepted=false` |
| running-stop | 运行中 `Stop(reason=P02A_OPERATOR_STOP)` | `STARTED→STOPPING→CANCELED`；夹爪总开度停止前后均 `0.081435 m` | `CANCELED`，`stop_reason=P02A_OPERATOR_STOP` |

总开度速度反馈与参考同号同量纲：`left-close` 窗口实测最小 `-0.040042 m/s`，`left-open` 窗口实测最大 `+0.040017 m/s`，与档案最大速度 `0.04 m/s` 对应；反馈不裁剪。

每个 `COMPLETED` 窗口均保存接受参考、实测状态和 feedback；`left-arm` 的证据链包括请求、`reference`（`accepted_sim_time_ns=284000000`、`curve_duration_ns=972000000`）、29 个实测状态、32 条 feedback 与 `COMPLETED` 终态。执行反馈的 phase 序列为 `ACCEPTED→STARTED→COMPLETED`。停止窗口的 feedback 序列为 `ACCEPTED→STARTED→STOPPING→CANCELED`，并保留首次停止原因；停止不自动张开夹爪。后端最终 `status.json`：`exit_code=0`、`success=true`、`fault=null`、`physics_stopped=true`、`planner_stopped=true`、`renderer_closed=true`。

本次停止测试在左右夹爪已经张开的空爪状态执行，证明该状态下停止不主动改变夹爪目标。闭爪/带物状态的保持与不卸载仍须在 P02B 通过真实接触验证。

## 回归

工作目录 `simulation/mujoco/docker`；测试容器挂载仓库到 `/workspace`：

```bash
docker compose run --rm --no-deps -v /home/ljk/work/sim_dual_arm:/workspace:ro \
  -e PROJECT_ROOT=/workspace -e FOLLOWER_REPORT_DIR=/tmp/follower-reports \
  -w /workspace simulation bash -lc \
  'source /shared/common/tools/docker/setup.sh && python3 -m pytest -p no:cacheprovider -q \
     tests/unit/test_scene_config.py tests/unit/test_scene_selection.py \
     tests/unit/test_motion_config.py tests/unit/test_backend_follower_config.py tests/unit/test_contact_unload.py \
     tests/unit/test_gripper_completion.py tests/unit/test_control_state.py \
     tests/unit/test_follower_control_state.py tests/unit/test_feedback_capability.py \
     tests/unit/test_legacy_feedback.py tests/unit/test_environment_entry.py \
     tests/unit/test_contract_consistency.py tests/unit/test_profile_cameras.py \
     tests/unit/test_prepare_backend.py tests/unit/test_prepare_scenario.py \
     tests/unit/test_scenario_binding.py tests/unit/test_scene_visibility.py \
     tests/unit/test_marvin_scene.py tests/integration/test_panda_io.py \
     tests/integration/test_sim_scene.py tests/integration/test_follower_execution.py \
     tests/integration/test_marvin_io.py'

docker compose run --rm --no-deps -v /home/ljk/work/sim_dual_arm:/workspace:ro \
  -e PROJECT_ROOT=/workspace -w /workspace simulation bash -lc \
  'source /shared/common/tools/docker/setup.sh && python3 -m pytest -p no:cacheprovider -q \
     tests/integration/test_backend_ros.py -s'

docker compose run --rm --no-deps -v /home/ljk/work/sim_dual_arm:/workspace:ro \
  -e PROJECT_ROOT=/workspace -w /workspace simulation bash -lc \
  'source /shared/common/tools/docker/setup.sh && python3 -m pytest -p no:cacheprovider -q \
     tests/integration/test_marvin_io.py'
```

结果：

| 范围 | 结果 |
| --- | --- |
| Panda/Marvin I/O、控制、跟随、反馈、配置、场景、准备 | `356 passed in 15.25s` |
| Panda backend ROS（含 SIGTERM 收尾） | `4 passed in 6.87s` |
| Marvin 具名 I/O（顺序、开度/闭合、速度前馈、拒绝、线程/场景、接触绑定） | `8 passed in 7.51s` |
| Marvin Robot API（真实服务、运动、停止、证据） | `2 passed in 12.98s` |
| 独立 Marvin backend 启动/收尾 | `exit_code=0, success=true` |

直接 I/O 前馈测试在 `right_joint7` 上执行 0.6 rad 平滑运动：关闭前馈峰值位置误差 `3.11e-2 rad`，启用后 `3.47e-3 rad`；夹爪在中间开度 `0.0414 m` 处验证了 `+0.01 m/s` 张开速度使闭合命令减小、`-0.01 m/s` 使闭合命令增大。该测试是执行器映射的灵敏度证据，不是任务轨迹或 P01A 区域验证。

## 未完成项与 P02B 接入点

- 未实现真实持物、双指同物块判定、左右独立持稳或短时共持。接触绑定和空爪反馈已通；P02B 在真实控制下验证接触事实和夹爪停止语义。
- 未实现双臂 IK、抓取/交接/放置候选目标和路径净空；这些在 P02B 建立，并供 P03 Teacher 复用。
- 未启动任务插件、Teacher、采集、导出或训练。`robot.task`/`robot.task_mode` 保持省略。
- 底座网格仍仅渲染；P02B 若候选路径进入底座中轴约 0.15 m 范围，先补经验证的代理几何。
- 当前活动 SDK 不含 `panda_stack_api_msgs`；Panda 示教回归前按 Panda 方案重建 SDK 并核对标记。
- P02B 接入点：使用本轮 `MarvinIO` 的 `finger_bodies`、`scene.contact_object_names`、具名 TCP/指面站点和标准 `ControlSession`；目标/路径生成放在任务实现，接触事实由通用接触层产生，诊断入口只组装输入和保存结果。
