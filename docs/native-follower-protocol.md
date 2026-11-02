# 受限跟随公共协议与 ROS 后端（DSH-031）

DSH-031 在 DSH-030 已验收的同一物理控制器上增加**受限跟随的公共记录、ROS 服务和可选后端入口**。它不改变原 `NativeRequest` / `AcceptedReference` / `NativeExecution` 的语义，不创建第二个物理线程、不复制 `NativeController`，也不接 runtime、Policy RPC 或 MCAP4。

## 1. 独立类型：`FollowerReference`

`contracts/python/src/robot_contracts/follower_reference.py` 是纯标准库模块，不导入 Ruckig、NumPy、SciPy、MuJoCo、ROS，也不使用 pickle、动态 Python 字段或 JSON 包裹代替字段。`FollowerReference` **不是** `AcceptedReference` 的子类或别名，因为受限跟随明确不承诺逐点通过原预测。

记录显式保存：

- 完整 raw `NativeRequest`（只允许 `STREAM_FIRST` / `STREAM_NEXT`），包括未被选中的前缀和尾部；
- 原 `ActionWindow`、`ChunkAnchor`、观测/交接时刻和 selected indices；
- 实际导出的 `ReferencePlan` 系数；
- `max_accelerations`、`max_target_deviation_limits`、`max_target_deviations`、`duration_grid_ns`；
- raw / restricted 端速，实际导出端点位置/速度/加速度，请求时域、Ruckig 时域和 `extension_ns`；
- 接受 sim/wall 时刻、`motion_seq`；
- `FOLLOWER_ALGORITHM_VERSION`、`FOLLOWER_SAMPLER_VERSION`、`FOLLOWER_NORMALIZATION_VERSION`；
- 各轴 `velocity_limited` / `acceleration_limited` / `boundary_preserved`；
- canonical `reference_digest`。

构造器先做身份、维度、时间、有限值和版本校验。未知算法/sampler/规范化版本直接拒绝，不会把旧记录静默解释成新语义。提供 digest 时逐字段校验；任何受保护字段被篡改（包括未选尾部、系数、限制、模式、版本和接受时刻）都会以 `FOLLOWER_DIGEST_MISMATCH` 失败。

### 独立重建

```python
from robot_contracts.follower_reference import FollowerValidationLimits, validate
from robot_backend_sim.follower_protocol import validation_limits

limits = validation_limits(control_config, follower_config, duration_grid_ns=2_000_000)
prepared = validate(record, profile, limits)
```

`FollowerValidationLimits` 是纯标准库的明确约束值类型，含 `joint_names`、`max_accelerations`、`max_target_deviations`、`target_count`、`max_terminal_tail_duration_ns` 和可选 `duration_grid_ns`。backend 从已知配置转换，runtime 以后可从快照构造；contracts 不依赖 backend，也没有可选 duck-typed 属性被静默跳过。

`validate()` 不信任记录里的派生值，而是用记录保留的 anchor/window/系数/限制/模式重建 DSH-030 的 `PreparedFollowerPlan`，复用唯一共享的端速、刹车和连续偏差实现，并逐项比较实际端点、偏差、flags 和 `extension_ns`。它还检查：

- `is_final == request.settle_at_end`；
- `observation_time_ns <= accepted_sim_time_ns <= anchor.start_sim_time_ns <= accept_before_ns`（0 是合法实际时刻）；
- `window.target_count`/raw `target_count` 与约束的 `target_count` 一致；
- 请求时域和实际执行 `plan.duration_ns` 都不超过配置的 terminal tail；
- 记录的 `duration_grid_ns` 与约束一致。

### 采样与边界规范化

```python
raw = record.sample_at(sim_time_ns)              # 未规范化的系数采样
applied = record.applied_sample_at(sim_time_ns, profile)
# applied.sampled_positions / applied.applied_positions / applied.corrections
# applied.applied_positions 即 physically applied reference positions
```

`applied_sample_at()` 使用 `robot_contracts.follower_math.normalize_follower_positions()`；`corrections = sampled - applied`。位置规范化版本固定为 `hard-boundary-normalization-v1` 并进入 digest。速度和加速度不被位置边界规范化改写。

### 重试

`follower_protocol.follower_reference_from_chunk_status()` 从每块自己的 frozen `ChunkStatus.accepted_sim_ns/accepted_wall_ns` 重建记录，而不是取当前测量。因此合法重发同一块返回原接受时刻和原 digest；不同块分别保留完整源和接受事实。

## 2. ROS IDL

新增：

- `contracts/ros/robot_api_msgs/msg/FollowerReference.msg`
- `contracts/ros/robot_api_msgs/srv/SubmitFollower.srv`

`SubmitFollower.request` 复用已有 `NativeCommand`。服务入口显式选择 follower 规划；请求 `kind` 只能是 `STREAM_FIRST` / `STREAM_NEXT`，`settle_at_end` 是显式 final-block 标志。响应与 `SubmitNative` 同形：`accepted`、`reason`、可选 `NativeExecution`、可选 `NativeChunkStatus`、可选 `FollowerReference`。没有复制 feedback 消息。

`libraries/ros_support/src/robot_ros_support/follower_conversions.py` 逐字段双向转换，重建并重新校验 raw request、window、anchor、selected indices、系数、限制和 digest。生成的 ROS 类不被直接信任。

后端公布：

- `/robot/follower_references`：具体 `FollowerReference`；
- `/robot/accepted_references`：仍只发布严格 `AcceptedReference`；
- `/robot/submit_follower`：新服务；原 `/robot/submit_native` 仍走 STRICT。

只有配置启用 follower 时才公布 `bounded_target_follower_v1` capability；`follower_reference` 也只在启用时出现。未知 ACK 仍用同 command id / chunk seq 查询并幂等重发获取原记录。跨 `SubmitNative` / `SubmitFollower` 或跨算法的同 ID 由包 fingerprint 和 planner 显式模式检查拒绝。

## 3. 同一个 worker 与生命周期

`NativeWorker.submit_follower()` 与原 `submit_native()` 共用一个有界 planner 队列，队列元素显式携带 `NATIVE` / `FOLLOWER` 模式。规划线程只做只读快照、Ruckig 和 DTO 编译；admit/apply/step/measure 仍只在唯一 physics owner 线程。首块走 `submit`，后继走 `queue_chunk`，严格块与 follower 块不能混在同一 stream。

owner 增加独立有界 `follower_references` 队列；接受记录满时 latch `QUEUE_FULL` 并走既有故障停止，不静默丢弃。严格 `accepted_references` 队列不变。lease 续期/失效、Stop/Release、reset 旧 epoch、planner Future 取消和关闭顺序沿用 027/030 实现。

## 4. 可选配置

`configs/simulation/backend_follower.yaml` 与原 `backend.yaml` 使用相同的 scene/profile/control/stream，并增加：

```yaml
robot:
  follower: panda_follower.yaml
```

`robot.follower` 是可选键，路径按配置文件所在目录解析。未知键和缺失文件仍拒绝。原 `backend.yaml` 不设置该键，follower 服务返回 `FOLLOWER_DISABLED`，其他 API 完全正常。`run.json` 保存 effective config、follower 源文件 hash 和 `ruckig` 包版本。

## 5. 固定 Ruckig 构建

`deploy/docker/backend-sim/Dockerfile` 增加：

- `ruckig-builder`：独立阶段安装编译器和固定 build 工具，所有安装使用 `--require-hashes`；用 `--no-build-isolation --no-deps` 构建锁定 sdist，`CMAKE_BUILD_PARALLEL_LEVEL=2`；
- `follower-dev`：从 `mujoco-dev` 延伸，只复制生成的 wheel 和 `build.json` 来源记录，`--no-index --no-deps` 安装，不复制编译工具。

锁文件：

- `deploy/docker/ruckig-requirements.lock`（Ruckig 0.15.3，sdist SHA-256 `48f68535...f34183`）
- `deploy/docker/ruckig-build-requirements.lock`（scikit-build-core 0.9.10、nanobind 2.4.0、cmake 3.31.6、ninja 1.11.1.3 及锁内依赖）
- `deploy/docker/ruckig-source.json`（来源、许可证、约束和参考预检 wheel）

`build.json` 在构建时写实际 wheel SHA、字节数、包版本、Python 和全部 build 工具版本。默认 Docker target 仍是 `mock`，原 `mujoco-dev` 和 15 个 native 依赖不变。真实 Docker build 由监督者执行，DSH 不编造镜像 ID。

## 6. 测试

- `tests/unit/test_follower_reference.py`：标准库依赖、digest/字段篡改、未知版本、独立重建、采样修正、稳定接受事实。
- `tests/unit/test_follower_protocol.py`：真实 Ruckig plan 的 follower packet、chunk 接受时刻/digest 重试、模式与 final 标志。
- `tests/unit/test_backend_follower_config.py`：可选路径、unknown key、follower 队列。
- `tests/integration/test_follower_protocol_ros.py`：真实 CDR 双向往返、ROS 服务三块 follower、稳定重试、modified retry 冲突、最终整体/块终态、发布记录与旧队列隔离。
- 原 `tests/unit` 与 030 物理/ROS 回归继续运行；真实 EGL/Docker 入口由监督者验收。

DSH 本机没有 ROS 运行时，ROS 测试在宿主上按 `ImportError` skip；本机单元测试通过，且不把 skip 记成真实 ROS 通过。
