# 无ROS物理运动、接触完成与有界停止

2026-11-01。DSH-022 在已验收的 `PandaIO`、DSH-021 定时参考计划和真实 MuJoCo
场景之上增加单运动状态机。它仍不是 ROS 后端、lease/reset、命令幂等历史、
任务成功判定或完整采集链路；局部 phase 不冒充 ROS command ACK。

## 文件与职责

| 文件 | 职责 |
| --- | --- |
| `components/backend_sim/src/robot_backend_sim/motion_config.py` | 严格加载不可变控制配置；纯 Python/PyYAML，不导入 MuJoCo |
| `components/backend_sim/src/robot_backend_sim/contacts.py` | 具名双指接触、法向力求和、单指速度与接触完成规则 |
| `components/backend_sim/src/robot_backend_sim/braking.py` | 纯数学常加速度刹车与显式合并分段计划 |
| `components/backend_sim/src/robot_backend_sim/motion.py` | 组合 `PandaIO`、参考计划和真实 MuJoCo 的单运动状态机 |
| `configs/simulation/panda_control.yaml` | 控制阈值、期限和接触门槛的唯一手写来源 |

`contracts/python/src/robot_contracts/reference.py` 只提取了公开的
`check_continuous_limits(extremes, joints, max_accelerations)`，normal 与 brake 共用同一
连续区间限位判断；公开入口校验极值/限位的有限性、名称顺序与有效范围。原有样条编译、采样、求根和限位错误码保持。

`PandaIO.require_scene(scene)` 是公开的小型归属校验：先检查调用线程是创建
`PandaIO` 的 Thread，再检查传入对象就是构造时绑定的同一 `Scene`。`NativeMotion`
在读取任何可写状态前调用它，因此 `NativeMotion(sceneB, PandaIO(sceneA))` 会以
`SCENE_MISMATCH` 拒绝，跨线程调用会以 `WRONG_THREAD` 拒绝。

## 公开 API

```python
prepared = prepare_motion(initial_reference, targets, profile, config)

motion = NativeMotion(scene, io, config, initial_reference=initial_reference)
accepted = motion.start(prepared, wall_time_ns)
live = motion.step(wall_time_ns)
live = motion.poll(wall_time_ns)
live = motion.request_stop("OPERATOR_STOP", wall_time_ns)
result = motion.result              # 首次终态后冻结
frozen = motion.terminal_snapshot   # 与实时 snapshot 分开
```

- `prepare_motion`：在物理循环外调用一次 `compile_reference_plan`，再用共享
  `verify_reference_plan` 验证完整曲线；不访问 `Scene`。`PreparedMotion` 直接构造也会
  重新验证曲线、关节顺序、目标/初值和加速度，不能绕过。
- `start`：只检查已通过的 `PreparedMotion` 与当前 profile 摘要、关节对象、控制配置和
  当前保持参考匹配。它不重新拟合、不在 `step` 内验证整条曲线。`start` 返回
  `ACCEPTED`，但第一次实际应用参考并 `mj_step` 后才算 started。
- `step`：在仿真时刻 `t` 通过 `PandaIO.apply_targets` 写 16 个 ctrl，然后只调用一次
  `mj_step` 到 `t+dt`，再 `mj_forward` 刷新同一测量时刻的接触力。每次调用只推进一步，
  沿用场景 2 ms 步长。`IDLE` 时也应用当前 hold 参考并正常推进物理，不要求先有命令；
  terminal 后仍执行 hold step 并返回新的物理快照，但不改写冻结 result/terminal
  snapshot。
- `poll`：不步进、不睡眠、不读隐藏墙钟；只返回状态并处理期限转换。墙钟期限到点时，
  即使没有新的物理步进，`poll` 也会把 `RUNNING/SETTLING/ACCEPTED` 转入 `STOPPING`；
  若 `STOPPING` 的独立墙钟期限继续耗尽，则锁存 `FAILED/STOP_TIMEOUT`。terminal 时
  `poll` 返回冻结的 terminal snapshot；`step` 才返回步进后的实时物理 snapshot。
- `request_stop`：先检查 motion/stop 墙钟期限，再只进入 `STOPPING`，不立即把参考速度
  置零。重复请求不重建计划、不延期。已 `start` 但尚未第一次步进、以及 `SETTLING`
  中都可请求；`IDLE` 未 start 或终态后不属于运行中停止（终态后为无操作）。晚到的
  人工 stop 不能把已到期的 `WALL_TIMEOUT` 改写成正常 `STOPPED`。
- `result` / `terminal_snapshot`：首次进入 `COMPLETED`、`STOPPED` 或 `FAILED` 时冻结。
  之后的 hold `step` 仍可推进真实物理，但不能改写冻结结果或终态 snapshot。

所有带 `wall_time_ns` 的入口拒绝 `bool`、负数、非整数和相对上次调用回退的值。时钟由
调用方提供；测试使用显式假时钟，真实调用方应传 `monotonic_ns`。`NativeMotion` 不创建
线程、不睡眠、不读写文件/网络。

## 阶段与时间

`Phase` 为 `IDLE`、`ACCEPTED`、`RUNNING`、`SETTLING`、`STOPPING`、`COMPLETED`、
`STOPPED`、`FAILED`。active/settling/stopping 之间拒绝重叠 `start`；正常完成或停止后
允许下一次运动；`FAILED` 锁存，恢复留给后续 reset 任务。

参考和刹车 elapsed **只**来自 `Scene.data.time`：

- normal 从第一次实际应用参考前的仿真时刻开始计，第一次应用为 0，下一次真实步进为
  2 ms；墙钟不变或跳变都不会改变参考进度。
- brake 的仿真起点是最后实际应用参考的时刻；下一步按已经过去的一个物理周期采样，
  避免重复一帧移动目标。未应用过 normal 参考时使用当前仿真时刻和零速保持参考。
  wall 冻结而物理继续步进时刹车仍会完成。
- settling、completion 稳定窗口和 settle timeout 都相对本次 motion 的仿真起点计算，
  不使用绝对 `data.time` 与计划时长相加。
- 墙钟只负责整体 motion 和 stopping 期限。

`MotionSnapshot` 使用带含义的字段：

- `reference_application_sim_time_ns`：应用该参考前的仿真时刻；
- `reference_elapsed_ns`：相对 normal/brake 仿真起点的参考 elapsed；
- `measurement_sim_time_ns`：步进并 `mj_forward` 后的仿真时刻；
- `accepted_wall_ns`、`started_wall_ns`、`started_sim_ns`：接受、第一次实际步进的
  墙钟和仿真时刻；
- `reference_positions/velocities/accelerations`、`measured_positions/velocities`；
- `tracking_errors = reference - measured`，与既有 `CommandStatus` 方向一致；
- 每夹爪完成模式/物体/双指力/单指速度/原目标误差，以及有界接触证据。

`MotionResult` 的终态时间字段为 `accepted_wall_ns`、`started_wall_ns`、
`started_sim_ns`、`finished_wall_ns`。高频轨迹、阶段和结果由后续唯一 ROS 层记录；
本模块不新建 writer。

## 配置

`configs/simulation/panda_control.yaml` 固定 16 维 `joint_names` 与逐轴数组：

| 项 | 候选值 |
| --- | --- |
| `max_accelerations` | 臂 1.0 rad/s²；总开度 0.5 m/s² |
| `position_tolerances` | 臂 0.012 rad；总开度 0.002 m |
| `velocity_tolerances` | 臂 0.025 rad/s；总开度 0.004 m/s |
| `stable_duration_s` | 0.2 s 仿真时间 |
| `settle_timeout_s` | horizon 后 2 s 仿真时间 |
| `motion_wall_timeout_s` / `stop_wall_timeout_s` | 10 s / 3 s 单调墙钟 |
| `contact_force_min_n` / `finger_velocity_max_m_s` | 每指 0.5 N / 0.003 m/s |

位置/速度硬限位仍来自 `RobotProfile`，配置不重复手填。加载器拒绝未知/缺失/重复键、
bool、非有限或超整型 ns 的值、错误长度/顺序和非正阈值/期限；错误带来源路径和字段。
`PandaControlConfig.to_document()` 提供可序列化快照，重新解析得到相同配置。

## 完成判定

14 个臂关节必须同时满足逐轴位置误差绝对值和绝对速度。每只夹爪满足以下任一模式：

1. `POSITION`：总开度误差和总开度速度都达标；
2. `CONTACT`：本次最终参考不比本次初始参考更开，实测总开度大于最终参考且位置门槛
   尚未达到；两根具名 finger body 接触同一个 scene 配置的具名 free-joint 物体，
   每指对同一物体的所有接触点法向力相加达到下限，且每指关节绝对速度达标。

接触模式不使用“双指总速度相消”来掩盖单指运动，也不接受 world/桌/机器人 link。无接触、
单指、不同物体、力不足、单指过快或多于一个双方共同候选对象都不得完成；多候选判歧义，
不任选。模式或物体身份改变、任一臂/夹爪失稳，整个连续稳定窗口清零。稳定时间只在
参考 horizon 之后计时，并始终使用相对本次仿真的时间。保持目标是原最终参考，不改成
实测开度；`GripperCompletion.target_error_m` 同样为 `target - measured`。

执行完成只说明参考已应用、实测稳定且接触证据满足上述规则；不表示堆叠成功，也不把
物体位姿送入 Policy。

## 有界刹车与故障

正常计划仍使用 DSH-021 clamped cubic。`request_stop` 从最后实际应用参考的 p/v 出发，
每轴按配置最大减速度做常加速度刹车到 0 再保持。停止时长向上取整 ns，实际减速度不超过
阈值；各轴停止边界合并为显式分段计划，位置/速度连续，并逐段验证 profile 位置、速度和
加速度限位。加速度可在边界阶跃；本模块不声称 jerk 受限或刹车 C2。

全部零速度轴走明确的空 hold 路径，不造零时长段。刹车完成后继续真实步进并保持刹车终点；
所有 16 轴实测速度连续满足 `velocity_tolerances` 达 `stable_duration` 才 `STOPPED`，不强制
张开夹爪。`STOPPING` 有独立墙钟期限；持续不步进或超期失败为 `STOP_TIMEOUT`。settling
有独立仿真时间上限，超时进入刹车但保留 `TRACKING_TIMEOUT`，停稳后仍为 `FAILED`。整体
墙钟超时保留 `WALL_TIMEOUT`。无法生成合法刹车时锁存 `FAILED/STOP_PLAN_REJECTED`。

`STOP_TIMEOUT` 和 `STOP_PLAN_REJECTED` 都把下一 hold 目标设为**最后实际应用参考**的
位置，而不是尚未执行的刹车计划终点；失败仍锁存，原 stop 触发原因保留在
`MotionResult.stop_reason`。没有隐藏容差扩大或失败转成功。

## 线程与验证边界

`NativeMotion` 与创建它的 `PandaIO` 绑定同一 `Thread` 和同一 `Scene` 对象；`start`、
`step`、`poll`、`request_stop`、`result`、`terminal_snapshot`、`phase` 在其他线程调用均以
`WRONG_THREAD` 拒绝。构造与运行不创建线程、不访问网络/文件（配置文件在构造前由调用方
加载）。`PandaIO.require_scene` 在读取可写状态前同时校验线程和 Scene 身份。

纯单元测试覆盖严格配置、快照不变/序列化、接触规则正负例和闭式刹车/多轴合并/连续限位/
零速度/非法输入。原生集成测试 `tests/integration/test_native_motion.py` 使用真实 MuJoCo、
固定资产和显式假墙钟，覆盖真实小幅运动、wall 不变/跳变而仿真只推进 2 ms、非零仿真起点、
两次非 home 连续运动、IDLE 步进、horizon 等待、刹车连续性与实测停稳、settling/未 started
停止、重复 stop、墙钟/stop 期限、晚到人工 stop 不清除故障、未执行刹车超时 hold、终态冻结
但 step 时间更新、Scene/IO 与线程错配、独立 raw ctrl/step/forward 重放，以及真实双指接触
正例和真实无接触反例。宿主缺少 MuJoCo/SciPy/固定资产时按既有约定 skip；监督者在固定镜像
内实跑。

本模块不实现 lease、网络命令幂等/query/reset/epoch、ROS 队列/三相机/RPC/teacher、任务
成功判定、日志 writer 或启动器；不宣称 S3 通过或完整 ROS 日志/ACK/MCAP 链路已接入。

最后实际应用的参考、测量、接触和时间由内部不可变 `_AppliedStep` 一起保存；停止始终以该步为依据。用于poll的最新反馈缓存与冻结terminal结果各自有明确用途，不分别维护五份可独立改变的“最后应用”字段。

## DSH-024 有界连续动作块（扩展，不另起控制器）

同一 `NativeMotion` 可选接收仅关键字 `stream_config`（`stream_config.py` 从
`configs/simulation/panda_stream.yaml` 严格加载）。`None` 时旧 `start/step/poll/
request_stop` 行为不变，stream 入口抛 `STREAM_NOT_CONFIGURED`。构造后实测物理步长
`dt`，要求 `min_lead_s` 至少一个 `dt`。

```python
anchor = motion.chunk_anchor()                 # 下一 motion 初块预览
anchor = motion.chunk_anchor(explicit_sim_ns)  # 显式时刻不取整/不偷偷改
motion.start_stream(prepared_chunk, wall_ns)   # ACCEPTED，到点实际 step 才 STARTED
submission = motion.queue_chunk(successor, wall_ns)
# ChunkSubmission(accepted, reason, snapshot)
```

`chunk_anchor` 只读且所有者线程：无活动流时用最后 hold 的 p、v/a=0，`predecessor_seq=0`，
`stream_id` 含本实例唯一标识及下一 motion 序号；`None` 按 `initial_lead_s` 向上对齐物
理步长。活动非 final 块时返回该块 end 的 p/v 和 `predecessor=active.seq`；已有 pending、
SETTLING/final 块、STOPPING/FAILED 均拒绝。读取不改变 phase/期限/序号/物理状态。

`start_stream` 要求首块 `predecessor_seq=0`、`stream_id` **精确等于**本原生实例
`uuid4` nonce 加下一 `motion_seq`（前缀相同但代号错误/旧实例/正常结束后的旧流均
`STALE_STREAM`，且不消耗代次）、profile/joints/加速度匹配、起点和终点在物理网格、
当前 sim ≤ 起点−`min_lead`（不足为 `CHUNK_LATE`，不 retime）、观测不来自未来且不超过
`max_observation_age_s`、初值等于当前 hold 且速度为零。等待期间继续 hold 并真实步进，直到 `application_sim_time` 恰等于
计划起点才应用参考、`mj_step`/`mj_forward` 一次并记 STARTED。旧 `start` 会清理上一流
的私有调度状态；新实例和正常结束后的旧 `stream_id` 均以 `STALE_STREAM` 拒绝。

`queue_chunk` 最多一个 pending，检查流身份、前驱序号、起点等于前块 end、profile/
joints/加速度、边界 p/v、观测时效和 `handoff-min_lead` 准入截止（恰好截止可接受）。
错误的流/顺序/锚点/网格/队列满只拒绝，不破坏当前计划、不消耗槽位或序号；合法候选
迟到或观测过期返回 REJECTED 并锁存刹车故障。准入前先落实调用方已提交 wall 的流/
进度/stop 期限：已超 120 s 或已停钟时返回准确的 `WALL_TIMEOUT/PROGRESS_TIMEOUT` 与
当前 `STOPPING/FAILED` 快照，不返回 ACCEPTED，也不重置预算。控制方法不 sleep、不读网络/文件，也不在
步进中拟合 SciPy。

块边界只写一组 ctrl 并只 step 一次：后继初点与前块终点共享同一 p/v；前块 RETIRED
与后继 STARTED 引用同一真实应用/测量事实，不插 hold、不按实测重定基。非 final 块无
pending 时仍应用该块末端一次，再从最后应用 p/v 触发 `UNDERRUN` 刹车；绝不调用过期
`sample_at` 或零速 hold 假装停止。final `settle_at_end=True` 块末速度为零，终点后显式
hold 末目标进入原 SETTLING，仍按原绝对速度/位置/接触/稳定窗口才 COMPLETED。一个流
跨全部块共用一个 `motion_seq`；只有最终停稳才 COMPLETED，刹车实测停止后
STOPPED/FAILED。

`MotionSnapshot` 旧字段不变，新增 `stream_id`、当前实际应用 `chunk_seq`（等待时
None）、`reference_kind`（HOLD/TRAJECTORY/CHUNK/BRAKE）、
`tracking_velocity_errors`、`tracking_quality`（HOLD/FOLLOWING/CONSTRAINED/
OUTSIDE_TOLERANCE）、`tracking_violation_ns` 和本次操作产生的不可变
`chunk_events`。每个 `ChunkEvent` 有本流递增 `event_seq`、motion/stream/chunk 身份、
`observation_id`、计划起止和真实 sim 时刻、phase（ACCEPTED/STARTED/RETIRED/
CANCELED/REJECTED）及 reason。每次 API 只返回本次少量事件；poll 可重复返回同一
event_seq，不产生新事件，正常步进不会反复退休旧块。RETIRED 只表示参考窗口已交接，
不代表运动/停稳/抓取成功。

驱动/自由夹爪按位置误差和**参考速度-实测速度**判断跟踪；只有参考仍处于闭合方向、
实测开度大于参考且超出位置容差、双指同物体足力且足够慢的真实闭合约束才可判
CONSTRAINED；张开目标/错误开度不能被接触证据豁免。目标、开度与速度误差始终保留。任一轴连续超限达到
`tracking_timeout_s`（仿真时间）进入刹车，停稳后 FAILED/TRACKING_TIMEOUT；全体重新
达标才清除计时，queue、换块或转入最终 SETTLING 均不清零。流整体墙钟期限为 `stream_wall_timeout_s=120`，
独立仿真进度停滞墙钟 3 s 触发停止；旧停稳仍用原 10 s/3 s 期限。`poll` 只检测期限，
不推进仿真、不伪造进度。

`motion.py` 全模块只保留一个 `_apply_and_measure` 真实 `apply_targets → mj_step
一次 → mj_forward → measure/contact` 位置，旧停稳、hold、stream 和 brake 共用。


连续执行的控制入口分为 `_step_stream`（选择参考并执行一次）、
`_step_stream_start`（等待与首次实际应用）、`_select_stream_boundary` /
`_apply_stream_boundary`（选择和落实交接）以及 `_finish_stream_step`（依据测量更新状态）。
所有分支复用 `_apply_and_measure`，没有第二套物理循环或步进中重新拟合。

连续模式的 `reference_elapsed_ns` 相对当前块起点；final HOLD 继续使用最后块起点，
BRAKE 则相对最后实际应用参考的刹车起点。绝对应用/测量时刻始终独立保留，
整个 motion 的起点见 `started_sim_ns`。final 块末端的 RETIRED 原因为 `FINAL_HOLD`，
此时仍待实测稳定，不等于 COMPLETED。验收与已知边界见 [DSH-024 结果](tasks/dsh-024-result.md)。
