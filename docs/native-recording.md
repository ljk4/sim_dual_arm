# 原生诊断运行时与可审计 v3 MCAP

2026-11-01。DSH-028 在 DSH-027 真实 ROS/MuJoCo 后端之上增加诊断运行时。它通过原生
`SubmitNative` 执行配置驱动的 MOTION，按精确捕获记录状态、三路原图和原生反馈，并写成
显式 `schema_version: 3` 的 MCAP。它不是正式采集、训练或任务成功判定；`task` 始终为
`not_evaluated`。

## 宿主变量与空间检查

以下路径按本 worktree 实际部署替换；`CONFIG`（runtime 使用 `NATIVE_RUNTIME_CONFIG_CONTAINER`，后端保留 `NATIVE_CONFIG_CONTAINER`）必须位于仓库内，因为容器内由只读
`/workspace` 挂载解析相对路径。

```bash
ROOT=/home/ljk/work/sim_dual_arm
BUILD_ROOT=/home/ljk/work/sim_dual_arm/.build/ros-dev
DATA_ROOT=/home/ljk/work/sim_dual_arm/var
ASSETS_ROOT=/home/ljk/work/sim_dual_arm/third_party/src/mujoco_menagerie
ASSET_SUMMARY=/home/ljk/work/sim_dual_arm/.build/assets/panda.json
CONFIG=$ROOT/configs/runtime/native_collect.yaml

findmnt -T "$ROOT"
findmnt -T "$BUILD_ROOT/install"
findmnt -T "$ASSETS_ROOT"
findmnt -T "$DATA_ROOT"
df -h "$ROOT" "$BUILD_ROOT" "$ASSETS_ROOT" "$DATA_ROOT"
```

`BUILD_ROOT` 必须已有通过 DSH-027 校验的 `.build/ros-dev`（`dev-ros:stage2` 依赖镜像
+ `dev_workspace.py check`），本项不重建镜像、不安装 Docker。资产只读挂载，不复制进
工作树。运行时容器不需要 GPU；只有 `native-backend` 需要 EGL/GPU。

## 启动

后台服务使用同一 Compose project/network；overlay 只增加 runtime 服务，不复制生命周期
启动器。先创建独立数据根，再启动两个服务：

```bash
mkdir -p "$DATA_ROOT/raw" "$DATA_ROOT/runs"
RUN_ID=native-diagnostic-$(date -u +%Y%m%dT%H%M%SZ)
PROJECT=native-runtime-${RUN_ID##*-}

export NATIVE_PROJECT="$PROJECT"
export NATIVE_REPO_ROOT="$ROOT"
export NATIVE_BUILD_ROOT="$BUILD_ROOT"
export NATIVE_ASSETS_ROOT="$ASSETS_ROOT"
export NATIVE_ASSET_SUMMARY="$ASSET_SUMMARY"
export NATIVE_RUN_ID="$RUN_ID"
export NATIVE_DATA_ROOT="$DATA_ROOT"
export NATIVE_RUN_ROOT="$DATA_ROOT/backend-runs"
mkdir -p "$NATIVE_RUN_ROOT"
# Host: $ROOT/configs/runtime/native_collect.yaml
# Container (read-only /workspace): /workspace/configs/runtime/native_collect.yaml
export NATIVE_RUNTIME_CONFIG_CONTAINER=/workspace/configs/runtime/native_collect.yaml
# Backend keeps its own variable and host file:
#   $ROOT/configs/simulation/backend.yaml -> /workspace/configs/simulation/backend.yaml
export NATIVE_CONFIG_CONTAINER=/workspace/configs/simulation/backend.yaml
export NATIVE_UID=$(id -u)
export NATIVE_GID=$(id -g)
export ROS_DOMAIN_ID=42

docker compose \
  -f "$ROOT/deploy/compose/native_backend.yaml" \
  -f "$ROOT/deploy/compose/native_runtime.yaml" \
  -p "$PROJECT" up --abort-on-container-exit --exit-code-from native-runtime
```

`native-runtime` 的 `--once` 会按 `native_collect.yaml` 的 `reset.seed: 0` 先 reset，再
执行两条诊断 MOTION。正常完成退出码为 0；信号中断为 130/143；实际控制、记录或日志故障
为非零。后端仍以前台服务运行，可用同一 project 的 `stop`/`down` 有界停止：

```bash
docker compose \
  -f "$ROOT/deploy/compose/native_backend.yaml" \
  -f "$ROOT/deploy/compose/native_runtime.yaml" \
  -p "$PROJECT" down
```

容器内等价的 runtime 命令（调试时）：

```bash
python3 -m dual_arm_runtime.native_node \
  --project-root /workspace \
  --config /workspace/configs/runtime/native_collect.yaml \
  --output "/data/raw/$RUN_ID" --runs-output /data/runs \
  --run-id "$RUN_ID" --once --shutdown-timeout 20
```

## start/status/end 与产物

runtime 复用既有公共入口：`/runtime/episode/start`、`/runtime/episode/end`、
`/runtime/status`。带 `--once` 时自动启动 `native-diagnostic`；人工运行可省略 `--once`：

```bash
# 在容器内或已有 ROS 环境的客户端
ros2 service call /runtime/episode/start robot_api_msgs/srv/StartEpisode \
  "{task_id: native-diagnostic}"
ros2 topic echo /runtime/status --once
ros2 service call /runtime/episode/end robot_api_msgs/srv/EndEpisode \
  "{reason: OPERATOR_STOP}"
```

阶段与既有 CLI 兼容：`PREPARING`、`ACTIVE`、`ENDING`、`IDLE`，故障为 `FAULT`。
EndEpisode ACK 只表示 `ENDING`；必须等 worker 停止、Stop/Release 后真实终态反馈/查询、
录制排空和 audit 完成。

产物：

| 内容 | 路径 |
| --- | --- |
| 已提交 episode（仅 recording.complete=true） | `$DATA_ROOT/raw/$RUN_ID/episode-*/` |
| 未完成记录（保留诊断） | `.partial-episode-*/` |
| MCAP | `episode-*/recording/` |
| manifest | `episode-*/manifest.json`（`schema_version: 3`） |
| 运行事件/解析配置 | `$DATA_ROOT/runs/$RUN_ID/` |

v3 记录包含 `BackendInfo`、`RobotState`、三路 `CameraFrame`、`ObservationIndex`、原始
`NativeCommand` 请求、`AcceptedReference`、`NativeControlEvent` 和 `NativeExecution`。
`recording.complete` 只表示排空、计数、引用、audit 和文件哈希通过，与
`runtime.outcome`/`task.outcome` 分开。合法取消/中断可有完整记录但 runtime 非 completed；
未确认执行的意图不会伪造 ACCEPTED。

## 独立审计

```bash
python3 -m dual_arm_runtime.audit "$DATA_ROOT/raw/$RUN_ID/episode-..."
# 或容器内
inspect_episode "/data/raw/$RUN_ID/episode-..."
```

v3 audit 流式读 MCAP，验证 context/序号、精确捕获配对、profile/标定、命令与事件迁移、
接受曲线/源窗口/限位/digest、实际 STARTED/终态事实、测量误差、终态冻结和参考重建。
BRAKE/HOLD 不会被当作原预测计划重建。缺少必要命令/参考/终态、同 ID 冲突、篡改或计数/
哈希不符会以非零失败，且不会发布完整目录。运行时完成的 episode 还要求配置中的每条
诊断 MOTION 都真实 COMPLETED 且关节确有运动。

## 测试边界

不需要 ROS 的 unit 覆盖精确捕获、严格配置、v3 版本选择和共享 DTO 校验。真实
ROS/DDS/EGL、后端进程、MCAP 写入和独立 audit 由
`tests/integration/test_native_runtime_ros.py` 在监督者固定镜像中执行；宿主缺少 ROS 时
明确 skip，不把 fake 消息类当作 CDR/物理证据。
