# 原生控制会话：租约、幂等命令、有界历史与复位

2026-11-01。DSH-025 在已验收的 `NativeMotion`（单运动 + 连续块）之上增加无 ROS 的
单所有者控制会话。它仍不是 ROS 后端、Policy 接线、正式采集或任务成功判定：只把
控制身份、显式租约、整体命令/块幂等、聚合状态和 episode 边界复位组合到现有唯一物理
循环，不新增第二次 `mj_step`。

## 文件与职责

| 文件 | 职责 |
| --- | --- |
| `contracts/python/src/robot_contracts/control.py` | 纯标准库 `RunContext`/`ControlCommand` 严格身份、共享 generation 比较、租约时长/墙钟/seed 校验 |
| `components/backend_sim/src/robot_backend_sim/control_state.py` | 具体租约表、有界命令/块历史、冻结状态/回复/事件、完整请求 canonical SHA256 |
| `components/backend_sim/src/robot_backend_sim/control.py` | 单所有者 `NativeController`，组合现有 `Scene`/`PandaIO`/`NativeMotion` |
| `libraries/backend_core/src/robot_backend_core/engine.py` | 仅把原 `_check_generation` 委托给共享比较；mock 其余行为不变 |

`control.py` 的方法不导入 SciPy、不拟合曲线、不读隐藏时钟/文件/网络、不创建线程，
也不直接调用 `mj_step`。物理只经 `NativeMotion.start/start_stream/queue_chunk/step/
poll/request_stop` 推进；`reset` 只调用 `Scene.reset(seed)` 并重建同场景的
`NativeMotion`。

## 身份与预编译请求

```python
ControlCommand(
    context: RunContext,
    lease_id: str,
    command_id: str,
    command_seq: int,
    observation_id: int,
    accept_before_ns: int,
)

PreparedControlCommand(command: ControlCommand, prepared: PreparedMotion | PreparedChunk)
```

- `RunContext` 必须是非空有界字符串加非负 `clock_epoch`/`control_epoch`；`ControlCommand`
  还要求非空有界 `lease_id`/`command_id`、`command_seq`/`observation_id` 在 `1..int64max`、
  `accept_before_ns` 在 `0..int64`。bool 不冒充整数。
- `PreparedControlCommand` 由调用方在规划阶段构造。运动包校验
  `prepared.profile_digest == context.robot_profile_hash`；块包校验
  `anchor.robot_profile_hash` 相同，且 `command.observation_id == window.observation_id`。
- 构造时一次性计算完整不可变请求的 canonical SHA256。payload 包含 context/lease/命令
  身份、关节顺序与全部 `JointSpec`、初值/全部目标、全部 cubic segment、源窗口
  `source_targets`/`selected_indices`/handoff/observation、边界速度、加速度和
  `settle_at_end`；不是只哈希最后目标。非有限 JSON 值被拒。
- 控制准入只检查已预编译对象和身份/边界，不重新拟合或重新校验整条曲线。

### generation 与完整租约

共享比较字段固定为 `robot_id`、`robot_profile_hash`、`backend_instance_id`、
`time_domain_id`、`clock_epoch`；不一致为 `STALE_CONTEXT`，并带字段名。完整租约额外要求
`episode_id`、`control_epoch` 相同。旧 mock 的 `_check_generation` 现在直接调用同一函数，
错误码与字段名语义不变。

## `NativeController` 精确接口

```python
NativeController(
    scene: Scene,
    io: PandaIO,
    control_config: PandaControlConfig,
    *,
    stream_config: StreamConfig | None = None,
    max_history: int = 256,
    max_chunk_history: int = 128,
)

@property context -> RunContext
@property phase -> Phase

acquire(context, owner, duration_ns, wall_ns) -> str
renew(context, lease_id, duration_ns, wall_ns) -> None
release(context, lease_id, wall_ns) -> ControlReply
submit(packet, wall_ns) -> ControlReply
queue_chunk(packet, wall_ns) -> ControlReply
stop(context, lease_id, reason, wall_ns) -> ControlReply
step(wall_ns) -> ControlReply
poll(wall_ns) -> ControlReply
query_command(context, command_id) -> ControlStatus
query_chunk(context, command_id, chunk_seq) -> ChunkStatus
chunk_anchor(context, lease_id, wall_ns, start_sim_time_ns=None) -> ChunkAnchor
reset(context, lease_id, seed, wall_ns) -> RunContext
```

`max_history`/`max_chunk_history` 必须是严格正整数，分别不超过
`MAX_COMMAND_HISTORY=4096`、`MAX_CHUNK_HISTORY=4096`，bool/零/超大拒绝。构造固定使用
同一 `Scene`/`PandaIO`，调用 `PandaIO.require_scene`，并要求创建线程就是唯一所有者线程。

`ControlReply` 为冻结值：

```python
ControlReply(
    snapshot: MotionSnapshot,
    status: ControlStatus | None,
    events: tuple[ControlEvent, ...],
    accepted: bool | None,
    reason: str,
)
```

`snapshot` 是本次调用的实时物理快照；`status` 是整体命令聚合状态。`submit`/`queue_chunk`/
`release`/`stop` 使用 `accepted/reason` 表达本次提交结果；`step`/`poll` 为 `None`/空串。
`status.snapshot` 是聚合状态自己的冻结快照，终态后不随物理继续而改写；它与
`ControlReply.snapshot`（可继续前进）明确分开。`MotionSnapshot.chunk_events` 原样保留；
控制器事件是独立的少量 `ControlEvent`，不另建无界 outbox。

`ControlEvent` 含组件内递增 `event_seq`、原 `context`、`command_id/command_seq`、
`observation_id`、可选 `chunk_seq`、`phase/reason` 和真实 sim 时刻；`scope` 为
`COMMAND`（`chunk_seq=None`）或 `CHUNK`。整体 ACCEPTED/STARTED/STOPPING/终态最多各发
一次；块事件按原 `StreamSchedule.event_seq` 去重，重复 `poll` 不重发。

控制器维护一个有界 deferred FIFO（`MAX_DEFERRED_EVENTS=256`），只保存已经创建、尚未
交付的不可变 `ControlEvent`。`acquire`/`renew`/`chunk_anchor`/`reset` 等不返回
`ControlReply` 的入口若在 `_settle_expiry` 中产生 STOPPING/CANCELED 事件，随后因
`LEASE_EXPIRED`/`CONTROL_BUSY` 等拒绝，事件不会丢失；下一次 `step`/`poll`/`submit`/
`queue_chunk`/`stop`/`release` 的 `ControlReply` 按原 `event_seq`、context、reason 和
sim 时刻交付一次。已在 deferred 中的事件不会因重置、新 acquire 或幂等重试被清空或重造；
容量不足时明确失败为 `DEFERRED_EVENTS_FULL`，不静默覆盖。

## 租约与墙钟

- `acquire`：owner 非空、`episode_id` 非空、`control_epoch >= 1`、duration
  `0.1..10 s`；显式单调 wall 必须是 `0..int64` 且不相对本控制器上次调用回退。成功后
  context 成为当前只读身份，返回唯一 `lease_id`。
- 租约到期用 `wall_ns >= expires_at_ns` 判断，不依赖仿真时钟。到期立即撤销授权并请求
  原生刹车；`reason=LEASE_EXPIRED`。`release` 成功则先撤销租约，再请求原生刹车，
  `reason=CONTROL_RELEASED`。
- 到期或 `release` 只把活动整体置 `STOPPING`，不立即置终态。真实 `NativeMotion`
  达到 `STOPPED` 后才把整体置 `CANCELED`；`NativeMotion` 的 `FAILED` 仍为 `FAILED`，
  并保留原 `stop_reason`。重复 stop/release/poll 不重建计划、不延长停止墙钟期限。
- 仿真停钟时租约仍按 wall 到期；此后停止墙钟耗尽只会产生 `FAILED/STOP_TIMEOUT`，
  不伪造实测停止。`stop_reason` 保留 `LEASE_EXPIRED` 等原触发原因。
- 新 `acquire` 不能接管活动或 `STOPPING` 控制器。无活动命令时可在 `FAILED` 获取租约
  供显式 `reset`，但 `FAILED_LATCHED` 仍阻止直接开新运动。
- `renew` 只更新到期时刻；坏 duration/context/lease 不会续期或清除合法租约。

## 整体命令、阶段与幂等

整体阶段为 `ACCEPTED`、`STARTED`、`STOPPING`、`COMPLETED`、`CANCELED`、`FAILED`。
`ACCEPTED` 只表示预编译请求已准入；第一次真实应用参考并步进后才发整体 `STARTED`。
`STARTED` 覆盖正常运动与最终 `SETTLING`；`STOPPING` 只表示已请求刹车。

- 一个连续流整体只有一个 `command_id`/`command_seq`/初始 `observation_id`；后继块保持
  context/lease/id/seq，仅 `observation_id` 随来源窗口变化且必须等于该窗口。
- `submit` 接受停稳的 `PreparedMotion` 或流首块 `PreparedChunk`。流首块必须
  `seq=1, predecessor_seq=0`。
- `queue_chunk` 只接受同一整体流的下一个后继；最多一个 pending，实际准入/迟到/断供沿用
  `NativeMotion` 行为。块 `seq` 与各自 observation 保留在 `ChunkStatus`，一条整体命令
  跨全部块共用一个 `motion_seq`。准入先做当前流/前驱/序号/terminal/phase 等结构检查，
  再检查 `accept_before_ns`；对结构匹配且会触发原生迟到/观测故障的候选，仍交给
  `NativeMotion.queue_chunk` 产生 `CHUNK_LATE`/观测原因并进入有界刹车，绝不由 envelope
  过期直接跳过原生故障路径。旧流、错序、错误锚点/profile 等无效包不会因此触发停止；
  已接受同包的幂等重试不受 deadline 影响，也不重新执行或刹车。
- `RETIRED` 只表示参考窗口已交接，不终结整体，也绝不报 `COMPLETED`；最终块 `RETIRED`
  仍需真实稳定才整体 `COMPLETED`。
- 相同 ID/seq/完整 payload 重试（`submit` 或 `queue_chunk`）返回保存事实，不重复执行、
  不消耗序号、不续租、不发新 ACK。相同 ID 但 seq/context/lease/来源/计划不同为
  `COMMAND_ID_CONFLICT`；相同 command/chunk seq 但完整包不同为 `CHUNK_ID_CONFLICT`。
- 新整体 `command_seq` 严格递增；被拒绝的新请求（坏预编译、过期、历史满、`FAILED_LATCHED`
  等）不消耗序号。`accept_before_ns` 按当前仿真/机器人时间（`Scene.data.time`）检查，过期新请求为 `COMMAND_EXPIRED`；
  已保存命令的幂等重试不走该检查。
- 未接受的无效块不占历史槽。块序号小于当前最大序号视为已退休/淘汰，返回
  `CHUNK_RESULT_EXPIRED`，不能重新当新块执行。命令记录缺失返回 `RESULT_EXPIRED`。
- `query_command`/`query_chunk` 严格只读：只做 generation/原 context 校验，不续租、
  不步进、不触发到期/新事件；同 generation 内可按原 context 跨租约读取，其他 context
  为 `STALE_CONTEXT`。
- 命令历史与块历史有严格上界。新增前淘汰最老 terminal 记录；活动整体和
  active/pending 块不可淘汰。无 terminal 可淘汰时拒绝新记录（`HISTORY_FULL` 或
  `CHUNK_HISTORY_FULL`）。不维护无界 tombstone；已淘汰块用上述序号规则明确过期。

## 时钟、事件与停止语义

- 所有 wall 参数由调用方传入；控制器不读 `time.monotonic_ns()`，也不 sleep。物理参考
  时刻只来自 `Scene.data.time`，事件中的 sim 时刻来自实际应用/测量快照或原生块事件。
- `step` 只调用一次 `NativeMotion.step`：先落实租约到期，再真实应用/步进一次。
- `poll` 只处理显式 wall 的租约/原生期限，不推进仿真、不伪造进度。`step`/`poll` 都先
  落实租约到期。
- 停止故障优先级沿用 `NativeMotion`：迟到人工 stop 不覆盖已触发的 wall/tracking/
  progress 故障；`STOP_TIMEOUT` 后整体为 `FAILED`，`stop_reason` 仍保留原触发原因。
- 同一快照被 `poll` 重复返回时不生成新事件；事件序号在本控制器生命周期内递增，reset
  不清零，避免旧事件序号复活。

## 复位与 generation

`reset(context, lease_id, seed, wall_ns) -> RunContext` 只允许当前所有者、显式 wall 和
安全原生阶段 `IDLE/COMPLETED/STOPPED/FAILED`；活动或 `STOPPING` 返回
`RESET_NOT_ALLOWED`，必须先实际停止。seed 是严格非负 int64，bool 被拒。

成功时：

1. `Scene.reset(seed)`；
2. 用同场景、同 I/O、同控制/stream 配置重建 `NativeMotion`，hold 用 profile 初值；
3. 撤销租约，清空命令/块历史和当前引用；
4. `clock_epoch += 1`，`episode_id=""`、`control_epoch=0`；保持
   `backend_instance_id` 与 `time_domain_id="mujoco-sim"`；
5. 返回新只读 `RunContext`。

旧 context 的包/anchor/控制请求因 clock epoch 变为 `STALE_CONTEXT`；旧查询同样
`STALE_CONTEXT`（或命令/块记录缺失的 `RESULT_EXPIRED`/`CHUNK_RESULT_EXPIRED`）。同一
seed 的两次 reset 在相同场景、无额外物理推进时 qpos 逐值一致；不承诺跨不同 MuJoCo
版本/资产摘要的位级一致。

## 配置、所有权与验证边界

- 控制/stream 参数仍来自既有 `PandaControlConfig`/`StreamConfig`；本任务不新增配置键、
  不安装依赖、不改镜像/启动器。
- 构造与全部公开读/写入口（`context`、`phase`、acquire/renew/release/submit/
  queue_chunk/stop/step/poll/query/chunk_anchor/reset）都检查创建 Thread；跨线程为
  `WRONG_THREAD`。`NativeMotion`/`PandaIO`/`Scene` 归属继续由原检查约束。
- 纯单元测试覆盖身份/租约/历史/deferred 事件上界/完整请求哈希；原生集成测试覆盖真实
  停稳、三块连续整体、迟到块刹车、release/到期、停钟 `STOP_TIMEOUT`、幂等冲突、失效
  renew/acquire/anchor/reset 后事件保留、旧 context 查询与淘汰、reset 确定性和跨线程
  拒绝。宿主缺 MuJoCo/SciPy/固定资产时原生测试 skip，必须由监督者在固定镜像实跑；
  本报告不把 skip 当通过。
- 未接 ROS 命令/ACK/MCAP、Policy、三图/测量精确配对、正式采集或 S3 整体验收；不宣称
  这些链路通过。


新 acquire 成功后，command_seq 的单调顺序从该租约重新开始，与现有 runtime 每个 episode
从 1 发送的行为一致；错误 acquire 不重置序号。旧命令历史保留原 context，跨租约仍可
只读查询；复用旧 command_id 仍是 COMMAND_ID_CONFLICT，不会被当成新命令。
