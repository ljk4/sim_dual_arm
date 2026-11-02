# 可运行的 ROS 原生 MuJoCo 后端

2026-11-01。DSH-027 在已验收的 `NativeController`/`NativeMotion`/`Protocol` 之上提供
真实进程：物理 owner 线程、单规划线程、独立 renderer 线程和 ROS 2 适配。它是可启动的
仿真后端，不是训练/teacher/任务成功判定；旧 mock 路径保持不变。

## 文件与职责

| 文件 | 职责 |
| --- | --- |
| `backend_config.py` | 严格解析 `backend.yaml`；路径相对配置文件解析；日志/队列/时限/命名空间 |
| `worker.py` | `NativeWorker` 编排；`_Planner` 唯一 SciPy 预编译；输出/关闭汇总 |
| `worker_owner.py` | `_Owner` 唯一读写 live `MjData` 并 `mj_step`；固定操作队列/Future；`system_stop` |
| `render_worker.py` | 独立 `SceneRenderer`/`MjData`/EGL 上下文；有界 capture/output 队列 |
| `node.py` | ROS 类型转换、发布/服务、Node 生命周期；不做物理/规划/渲染 |
| `ros_bridge.py` | worker Future → `rclpy.task.Future` 的 guard condition/timer 线程安全桥 |
| `backend_cli.py` | run 目录、信号、SingleThreadedExecutor 主循环、status.json 与非零退出 |
| `configs/simulation/backend.yaml` | 部署参数唯一手写来源；引用既有 scene/control/stream/profile，不复制物理参数 |
| `deploy/compose/native_backend.yaml` | 单服务、只读源码/SDK/资产、可写 run-root、独立网络 |
| `tools/run/native_backend.py` | 前台 Compose 启动器；只清理本项目，信号/退出码传播 |
| `tests/unit/test_backend_config.py` | 配置未知键/重复键/bool/有限值/队列/路径/命名空间 |
| `tests/integration/test_native_worker.py` | 真实 MuJoCo 单 owner、步进、队列、幂等、lease、reset、关闭、故障 |
| `tests/integration/test_native_backend_ros.py` | 真实 ROS/DDS/EGL 进程级 MOTION/流/查询/reset/精确捕获 |

## 线程、数据与有界队列

```text
ROS executor ──fixed op + Future──> owner queue (bounded)
   │                                  │
   │                        physics owner thread
   │                     Scene/PandaIO/NativeController
   │                    唯一 MjData 读/写；唯一 mj_step
   │                                  │ immutable Snapshot + measurement
   │                                  v
   │                        renderer queue (bounded, drop old unrendered)
   │                                  │
   │                        render thread (own EGL/MjData)
   │                                  │ 3×RGB
   │                                  v
   └──── state/frames/events/feedback/accepted queues (bounded) <── node timer

SubmitNative ──immutable request──> planner queue (bounded)
                                      │ prepare_request/SciPy/hash
                                      └─ PreparedControlCommand + request
                                             │
                                    owner admission recheck
```

- 物理步长只来自 `panda_stack.yaml` 的 `physics.timestep`（当前 2 ms）。owner 用
  `time.monotonic_ns()` 做实时节奏，参考/测量时间只来自 `Scene.data.time`；迟到时最多
  追赶 `max_catchup_steps` 个物理步，不跳过 `mj_step`，超出部分从当前墙钟重新计时。
- capture 默认 20 Hz。每次 capture 复制 `mjSTATE_FULLPHYSICS` 与同一时刻测量/context/
  `capture_seq`；renderer 只读该不可变快照，三路图在同一 capture 内一次产出。
- owner 队列容量 `queues.owner`，满时立即返回 `BACKEND_BUSY`。planner 同时最多一个在途
  编译，排队容量 `queues.planner`。renderer 输入/输出分别有界；只允许丢最旧的未渲染
  capture 或最旧输出并计数，控制事件/响应/接受的 reference 不静默丢弃，队列满即故障。
- 所有关闭等待都有界：拒绝新任务 → `system_stop` 真实刹车/墙钟预算 → renderer 在所属
  线程 `close` → join → logger `close`。

## 精确捕获与发布

`Capture` 字段：`context`、`capture_seq`、`sim_time_ns`、`joint_names`、`positions`、
`velocities`、`phase`、`command_id`、不可变 `rendering.Snapshot`。

节点从同一 `RenderedCapture.capture` 发布：

- `RobotState.state_seq == CameraFrame.frame_seq == capture_seq`
- `RobotState.stamp == joints.header.stamp == image.header.stamp == camera_info.header.stamp`
  `== stamp(capture.sim_time_ns)`（仿真捕获时刻，不是渲染完成时刻）
- 三路 `320x240 rgb8`，`frame_id`/`calibration_digest` 来自 profile；`CameraInfo` 由 scene
  的 `fovy` 与 profile 尺寸推导，零畸变。
- 旧 `clock_epoch` 的 renderer 输出在进入节点前被丢弃并计数；reset 后内部计数归零，公开 `capture_seq` 从 1
  重新开始。配对键包含 backend instance / clock_epoch / episode / control_epoch / seq。

高频 `/native_feedback` 每个 capture 可发布且允许在其有界队列满时丢旧周期值；控制阶段
迁移/终态标记为 critical，队列满时先淘汰一个周期项再保留它，accepted reference 使用独立
有界队列，二者都不因渲染丢帧而丢失。终态 feedback 直接来自 `ControlStatus.terminal_status`
的冻结 snapshot，后续实时 hold 步进不会改写它。

## 配置

`configs/simulation/backend.yaml` 顶层只允许 `robot/run/queues/limits/logging`。所有必填键
都显式出现；未知/重复键、bool 冒充数字、非有限值、越界队列/时限、缺失路径、非法 namespace
在打开 MuJoCo 前被拒绝。路径相对配置文件解析；CLI 根（assets/run）由显式参数给出，不使用 cwd。

`realtime_speed` 默认 `1.0`；`>1` 只加快墙钟节奏，不改变物理时间、步长或步数。
`--config/--assets-root/--asset-summary/--run-root/--run-id` 是进程入口参数；目标 run 目录已存在
时拒绝启动。可选 `--asset-source` 用于固定 `panda-source.json`。

## 公开 ROS 入口

namespace 默认 `/robot`，服务名 snake_case：

| 类型 | 名称 |
| --- | --- |
| topic | `/robot/info`、`/robot/state`、`/robot/cameras/{name}/frame` |
| topic | `/robot/native_events`、`/robot/native_feedback`、`/robot/accepted_references` |
| service | `acquire_control`、`renew_control`、`release_control`、`stop` |
| service | `submit_native`、`get_native_anchor`、`query_native_command`、`query_native_chunk`、`reset_simulation` |

`BackendInfo.protocol_major=1`、`backend_kind=mujoco`、`test_only=false`，能力为
`native_motion`、`continuous_chunks`、`exact_capture`、`simulation_reset`、`lease`、`stop`、
`command_query`、`native_events`、`accepted_reference`；不声明未实现的旧 linear action。
不提供无认证 ROS `system_stop`；`system_stop(reason, wall_ns)` 只在 owner 内部用于关闭/
日志/队列/渲染故障，且软件故障保持非零进程结果。

服务回调是 `async`，通过 `ros_bridge.RosFutureBridge` 把 owner/planner 的显式 `Future`
线程安全地桥接到 `rclpy.task.Future`：worker 完成时触发 guard condition，executor 任务在
单调 deadline 定时器到期前交付结果；不在 `SingleThreadedExecutor` 内嵌套 asyncio loop、
不 busywait、不阻塞其他 ROS 回调。完成/超时/调用方取消/节点关闭都会取消未开始的排队任务
并销毁 bridge 的 timer/guard。提交超时返回 `OUTCOME_UNKNOWN`，不声称“保证未执行”：请求
仍可能已被 owner 接受，调用方必须保留 command_id 并用 `query_native_command` 查询。
相同 command_id/块号 + 完全相同 request 的重试复用历史 `PreparedControlCommand`，重新校验租约/代际后返回原
原 accepted 时刻/digest/曲线；任一 request 字段变化返回 `COMMAND_ID_CONFLICT`，不会因当前
位置改变重新拟合。

## 启动（前台）

先在宿主设置可复制变量并确认挂载/空间：

```bash
ROOT=/home/ljk/work/sim_dual_arm-worktrees/dsh-027
BUILD_ROOT=/home/ljk/work/sim_dual_arm/.build/ros-dev
DATA_ROOT=/home/ljk/work/sim_dual_arm-worktrees/dsh-027/var
ASSETS_ROOT=/home/ljk/work/sim_dual_arm/third_party/src/mujoco_menagerie
ASSET_SUMMARY=/home/ljk/work/sim_dual_arm/.build/assets/panda.json

findmnt -T "$ROOT"
findmnt -T "$BUILD_ROOT/install"
findmnt -T "$ASSETS_ROOT"
findmnt -T "$DATA_ROOT"
df -h "$ROOT" "$BUILD_ROOT" "$ASSETS_ROOT" "$DATA_ROOT"
```

构建检查复用既有工具（不重建镜像；需要已存在的 `backend-sim:mujoco-dev` 与
`dev-ros` 生成的 `.build/ros-dev`）：

```bash
python3 "$ROOT/tools/build/dev_workspace.py" check \
  --project-root "$ROOT" --build-root "$BUILD_ROOT"
```

启动（launcher 负责独立 project、只读挂载、信号和非零退出；退出后只清理本项目）：

```bash
python3 "$ROOT/tools/run/native_backend.py" \
  --config "$ROOT/configs/simulation/backend.yaml" \
  --assets-root "$ASSETS_ROOT" \
  --asset-summary "$ASSET_SUMMARY" \
  --build-root "$BUILD_ROOT" \
  --run-root "$DATA_ROOT/runs/native-backend" \
  --run-id native-001
```

等价的容器内前台命令（调试单个服务时）：

```bash
docker compose -f "$ROOT/deploy/compose/native_backend.yaml" \
  -p native-debug --env-file <(printf '%s\n' \
    "NATIVE_PROJECT=native-debug" \
    "NATIVE_REPO_ROOT=$ROOT" \
    "NATIVE_BUILD_ROOT=$BUILD_ROOT" \
    "NATIVE_ASSETS_ROOT=$ASSETS_ROOT" \
    "NATIVE_ASSET_SUMMARY=$ASSET_SUMMARY" \
    "NATIVE_RUN_ROOT=$DATA_ROOT/runs/native-backend" \
    "NATIVE_RUN_ID=native-debug" \
    "NATIVE_CONFIG_CONTAINER=/workspace/configs/simulation/backend.yaml") up
```

正常关闭：前台 Ctrl-C 或 `SIGTERM`；launcher 显式停止本次项目容器，节点受控刹车、关闭 renderer/线程/
logger，`status.json` 的 `success=true`。GPU/渲染/日志/队列故障：worker 记录 fault、执行
`system_stop`，进程非零退出，`status.json` 保留 `failure/fault/计数`；不会把 CANCELED 的
物理终态伪装成成功。run 目录包含 `run.json`、`status.json` 与 `log/`，失败时保留异常栈和日志。

## 验收命令

宿主原生（工具头环境）：

```bash
export SIM_ASSETS_ROOT=/home/ljk/work/sim_dual_arm/third_party/src/mujoco_menagerie
export SIM_ASSET_SUMMARY=/home/ljk/work/sim_dual_arm/.build/assets/panda.json
/home/ljk/work/sim_dual_arm/.build/native-dev-venv/bin/python -m pytest tests/unit -q
/home/ljk/work/sim_dual_arm/.build/native-dev-venv/bin/python -m pytest \
  tests/integration/test_native_worker.py -q
```

监督者 Docker/EGL/ROS（真实三视图、服务与 DDS）：

```bash
export SIM_ASSETS_ROOT=/home/ljk/work/sim_dual_arm/third_party/src/mujoco_menagerie
export SIM_ASSET_SUMMARY=/home/ljk/work/sim_dual_arm/.build/assets/panda.json
/home/ljk/work/sim_dual_arm/.build/native-dev-venv/bin/python -m pytest \
  tests/integration/test_native_backend_ros.py -q
```

镜像验收必须实际配置 NVIDIA EGL/compute/utility/graphics，三路真实渲染和 ROS 服务不能由
host 注入的假 renderer 冒充。


## DSH-027 验收补充

原生后端已通过真实 ROS/GPU 与公开 Compose 入口验收，证据见[验收记录](tasks/dsh-027-result.md)。
流首块和后继使用各自的正确准入路径，按块保存幂等历史；重试不会重新拟合，也不能绕过租约。
阶段反馈用有界 FIFO 保护，周期样本只在 capture 节奏发布；背压仅替换周期值，不能替换终态。
reset 保留携带原 context 的待交付控制事实，只清可替换的捕获载荷。

run.json 保存完整有效配置、各输入文件内容/哈希、资产摘要、代码清单和依赖版本。
status.json 的 phase 是最终物理阶段，physics_stopped/renderer_closed/planner_stopped 分别报告真实退出。
关闭期间产生的控制事件在 ROS 与日志销毁前排出；无法完成关闭或日志失败均为非零结果。
logger 的 summary.json 检查自身计数/文件/complete 状态，不能套用 CPU mock 整次采集的 run_status schema。
