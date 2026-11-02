# 原生参考与反馈公共类型

DSH-026 定义传输无关的原生请求、可审计参考记录和反馈 DTO。当前只提供类型、
校验与显式 ROS 转换；没有新增 ROS 服务实现、队列、线程、MCAP writer，也尚未
声明 Native capability 已上线。

## 模块与边界

| 模块 | 职责 |
| --- | --- |
| `robot_contracts/native_reference.py` | `NativeRequest`、`AcceptedReference`、canonical digest、独立校验与按绝对时刻重建 |
| `robot_contracts/native_feedback.py` | `NativeFinger`、`NativeGripper`、`NativeExecution`、`NativeChunkStatus`、`NativeControlEvent` |
| `robot_contracts/native.py` | 稳定导入点，re-export 上述公共值 |
| `robot_ros_support/native_conversions.py` | 每个公共 DTO 的显式双向 ROS 转换 |
| `robot_backend_sim/protocol.py` | `prepare_request`、`accepted_reference`、`execution_feedback`、`chunk_feedback`、`control_event` |

共享 DTO 只依赖标准库和既有 `robot_contracts`。参考数学继续使用既有
`ReferencePlan`/`CubicSegment`/`ActionWindow`，没有第二套插值或极值算法。
`native_reference.py` 与 `native_feedback.py` 不导入 ROS、backend、MuJoCo、
SciPy 或 NumPy。`protocol.py` 不导入 ROS、不写盘、不步进、不创建线程。

## 模式

`NativeRequest` 包装既有 `Trajectory`（context、lease、command id/seq、
observation、deadline、joint_names、完整 source targets）以及：

| 字段 | MOTION=1 | STREAM_FIRST=2 | STREAM_NEXT=3 |
| --- | --- | --- | --- |
| `anchor` | `None` | `ChunkAnchor`，`predecessor_seq=0` | `ChunkAnchor`，`predecessor_seq>0` |
| `observation_time_ns` | 0 | 源观测捕获的绝对时刻 | 源观测捕获的绝对时刻 |
| `target_count` | 0 | 2..128 | 2..128 |
| `settle_at_end` | 必须 true | 可选 | 可选 |
| `final_velocities` | 必须 `None` | `None` 或显式向量 | `None` 或显式向量 |

- **MOTION**：source 时间相对本次实际 `STARTED`。调用方必须提供最后 hold
  参考；使用既有 `prepare_motion`，初末速度均为零。不能以测量重定基。
- **STREAM_FIRST / STREAM_NEXT**：source 时间相对源观测捕获时刻；
  `anchor.start_sim_time_ns` 是固定绝对 hand-off。`ActionWindow` 只选择严格晚于
  hand-off 的前 `target_count` 个点；完整 source（含不执行前缀/尾部）保留且
  校验，坏限位或坏数值不能被窗口隐藏。不 retime。
- 同一整体 stream 保持 command id/seq；每个后继请求的 `observation_id` 可变，
  但必须等于其窗口观测。跨块只声明 p/v 连续，不声明 a 连续。

## 时间与系数约定

- 所有时间、持续时间、超时均为整数纳秒；`int64` 非负，身份/序列为正整数。
  布尔值不会被当作整数接收。
- 多项式使用局部秒 `s` 和升幂系数：
  `p(s)=c0+c1*s+c2*s^2+c3*s^3`。
  不是 SciPy 的降幂 `c[k]` 顺序。
- `ReferencePlan` 是显式分段三次计划；`duration_ns` 为正，horizon ≤ 3 s，
  最多 128 段、64 轴。
- `ReferenceSample` 的 positions/velocities/accelerations 都在同一关节顺序下。
- 真实时刻 0 是合法时间；可选状态用 `None` 或显式 `has_*` 表达，不能把
  “未 started” 与 `started_sim_time_ns=0` 混为一谈。

## 可审计参考记录

`AcceptedReference` 保存：

- 原始 `NativeRequest`（完整 source，不被 selected targets 替换）；
- `motion_seq`、`accepted_sim_time_ns`；
- joint_names 与显式 `ReferencePlan`；
- 实际边界 `initial_positions`/`initial_velocities`/`final_velocities`、
  `max_accelerations`；
- `selected_indices`；
- `interpolation_version="explicit_cubic_v1"`；
- `reference_digest`。

`reference_digest` 是公开 DTO 的稳定 canonical JSON 的 SHA-256，格式版本为
`native-reference-v1`，覆盖请求身份、完整 source、anchor、全部 segment 系数、
时间、选择、边界和末端模式。它不是 backend 私有对象的哈希。构造时若未提供
digest 会自动生成；提供时立即校验，篡改会在反序列化/构造阶段以
`REFERENCE_DIGEST_MISMATCH` 拒绝。

`validate_accepted_reference(record, profile, max_accelerations)` 复用
`validate_reference_inputs` 和 `verify_reference_plan`：

- 关节/单位顺序与 profile 一致，完整 source 全部点通过限位；
- 实际系数通过连续 p/v/a 极值限制；
- MOTION 初末速度为 0、`selected_indices` 为空；
- STREAM 用 `ActionWindow` 重新计算 selection 和局部 targets，anchor p/v
  与计划起点一致；显式末速度按请求，`None` 按最后两个选中点后向差分，
  `settle_at_end=True` 强制零速；
- digest 再次重算，拒绝时间/目标/系数/身份/selection/末端模式篡改；
- `accepted_at` 不要求等于实际 started。MOTION 的实际计划零点来自后续
  `NativeExecution.started_sim`；STREAM 计划零点是 anchor 固定绝对时刻。

`sample_accepted_reference(record, sim_time_ns, *, actual_started_sim_ns=...)`
按绝对应用时刻重建参考。MOTION 必须显式传实际 started；其已声明 final 模式为
末端 hold，horizon 之后返回末目标 p 与 v=0/a=0（NativeMotion 实际也继续采样
末目标并等待实测稳定），起点之前仍严格 `REFERENCE_EXPIRED`。STREAM 从 anchor
取零点；非 final stream 超出 horizon 抛 `REFERENCE_EXPIRED`；final 零速 stream
可显式 HOLD 末端，未续接块不会被自动补零。

BRAKE 不伪装成原 normal 计划。完整取消审计留给后续接线；反馈仍保留实际
p/v/a 和真实 `reference_kind`。

## 反馈 DTO

- `NativeFinger`：side、finger_index(1/2)、body_name、joint_speed_m_s，以及
  三个等长有界数组 `object_names`/`normal_forces`/`contact_counts`（≤8）。
  保留每物体合力与计数，不只传单个 contact bool。force 非负、count 为非负
  整数、object_names 唯一。
- `NativeGripper`：side、mode(NONE/POSITION/CONTACT)、position_ok/contact_ok/
  contact_object/ambiguous、原 target/initial/final_reference、实测总开度/
  速度/误差、两个 `NativeFinger`。CONTACT 只表示接触模式，不表示物体堆叠成功。
- `NativeExecution`：完整 context、整体 command id/seq、初始 observation_id；
  整体 phase/reason/stop_reason；motion_seq、stream_id、当前实际 chunk_seq
  （0 表示未应用块）；accepted/start/finish sim 与 wall 及 has_started/
  has_finished；真实 motion_phase；reference application/measurement/elapsed/
  stable 时间；joint_names、参考 p/v/a、测量 p/v、位置/速度误差；reference_kind、
  tracking_quality/violation_ns；左右两个 `NativeGripper`。
  误差必须等于参考−测量。terminal 反馈只取 `ControlStatus` 冻结 snapshot，
  后续 `RobotState` 实时测量不写回。
- `NativeChunkStatus`：保留真实 chunk phase；`RETIRED` 不会改写成运动
  `COMPLETED`。
- `NativeControlEvent`：复制 `ControlEvent` 的 context/event_seq/command id/seq/
  observation_id/is_chunk/chunk_seq/phase/reason/sim_time_ns，不生成新身份或时间。

`protocol.py`：

```python
prepare_request(request, profile, control_config, *, initial_reference=None)
accepted_reference(request, packet, accepted_snapshot)
execution_feedback(control_status)
chunk_feedback(chunk_status)
control_event(control_event)
```

`prepare_request` 在规划调用方运行，不读 Scene。MOTION 只接受显式 hold 初值；
stream 从 anchor 取。`accepted_reference` 拒绝 request 与已编译 packet 不对应。

## 新 ROS 类型

消息：`ReferenceAnchor`、`NativeCommand`、`ReferenceSegment`、
`AcceptedReference`、`NativeFinger`、`NativeGripper`、`NativeExecution`、
`NativeChunkStatus`、`NativeControlEvent`。

服务：`SubmitNative`、`GetNativeAnchor`、`QueryNativeCommand`、
`QueryNativeChunk`、`ResetSimulation`。Acquire/Renew/Release/Stop 继续复用
既有服务。`ResetSimulation` 是可选仿真能力，不要求真机实现。

数组尽量使用 ROS bounded sequence；`NativeFinger`/`NativeGripper` 固定两个手
指/夹爪。`NativeCommand.anchor.stream_id` 为空是 MOTION 的“无 anchor”约定；
`has_final_velocities` 为 false 时 `final_velocities` 必须为空。转换器不信任
生成类：重复校验长度/范围/有限性/模式/has 字段，并明确拒绝非零 header、
非空 frame、point velocity/acceleration/effort 等不支持语义，不静默丢字段。
所有异常均为稳定 `ContractError`。

## 验证入口

- `tests/unit/test_native_contracts.py`
- `tests/unit/test_native_reference_record.py`
- `tests/integration/test_native_protocol_ros.py`
- `tests/integration/test_native_protocol_physics.py`

host 无 ROS/native/SciPy 时后两者按既有测试约定 skip；固定镜像/原生 SDK 下由
监督者执行真实序列化和物理对照。旧 msg/srv/action 源字节保持不变，只更新
`robot_api_msgs` 的 CMake 注册。


实际参考类型在首次轨迹应用前为 HOLD，普通运动有效区间为 TRAJECTORY，区间后明确
保持末端为 HOLD；连续块有效区间为 CHUNK。停止失败沿用最近真正应用的类型，不能
因仅创建了刹车计划就标为 BRAKE。该区分只修正反馈事实，不改变控制参考或限位。
接触传输验收复用运行前预载的自由块夹持夹具，目标0与实测约4cm分别保留，不计正式数据。
