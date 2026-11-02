# DSH-025：原生控制会话、幂等命令与复位

2026-11-01。用户要求连续推进至训练/仿真闭环，除非主动暂停。DSH-024 已验收：643 unit、81 native、18 独立组。先补物理控制会话，再接 ROS/观测/Policy。首轮 30 分钟，最多一次 10 分钟集中返工；交付报告即退出。

## 范围与组织

允许新增：

- `contracts/python/src/robot_contracts/control.py`：纯标准库控制身份与共用 generation 检查。
- `components/backend_sim/src/robot_backend_sim/control_state.py`：具体的租约与有界命令/块记录，不含物理循环。
- 同包 `control.py`：单所有者 `NativeController`，组合现有 NativeMotion/PandaIO/Scene。
- `tests/unit/test_control_contracts.py`、`test_native_control_state.py`、`tests/integration/test_native_control.py`。
- `docs/native-control.md`，允许更新 `docs/README.md`。

仅允许对 `libraries/backend_core/src/robot_backend_core/engine.py` 做一处小提取：原 `_check_generation` 委托真正共用的 generation 比较，保留原语义及其余 mock 行为。不得复制 mock 测量/即时取消来做物理控制，不做无关重构。

不改 NativeMotion、I/O、参考编译器、场景/profile、旧消息/ROS/记录/Policy/依赖/镜像/启动器。不安装依赖、不 Docker、不请求权限、不启动代理、不改其他工作树、不提交 Git。host 缺原生依赖可 skip，但必须如实报告，监督者固定镜像实跑。

分清租约、幂等记录、物理驱动，使用具体小类/冻结返回值，避免泛型框架、回调总线、继承 mock 或千行新类。控制方法不拟合、不读取隐藏时钟/文件/网络、不创建线程；所有 wall 参数由调用方传入。只在既有 NativeMotion.step 里推进物理，不增加第二处 mj_step。

## 身份与预编译请求

共享 `ControlCommand(context, lease_id, command_id, command_seq, observation_id, accept_before_ns)` 不含物理类型；冻结并严格校验 RunContext、非空有界 ID、bool/整型范围、seq/obs>0、ns≤int64。generation 比较与旧 mock 一致：robot/profile/backend/time_domain/clock_epoch；完整租约还要求 episode/control_epoch 一致。

仿真侧提供冻结 `PreparedControlCommand(command, prepared)`，prepared 仅为 PreparedMotion 或 PreparedChunk。构造在调用方规划阶段完成，校验关节/profile 关联和块 observation_id，并预计算完整不可变请求的 canonical SHA256（包括身份、全部源/窗口/边界/计划及 final 模式，拒非有限，不能只哈希最后目标）。控制准入不重新拟合/校验整曲线。

一个连续流对应一个整体 command_id/command_seq，块使用 PreparedChunk.seq 与各自 observation_id。后继 packet 保持 context/lease/command_id/command_seq，observation_id 随来源变化且须匹配窗口。初始命令记录保留初始 observation_id；不得把后继当成另一整体命令，也不得因 RETIRED 报 COMPLETED。

## 建议 API（等价清楚名称可以，文档必须列精确接口）

`NativeController(scene, io, control_config, *, stream_config, max_history=256, max_chunk_history=128)`：构造于物理所有者线程，Scene/IO 归属与配置沿用原检查。所有公开读/写入口都检查创建 Thread 对象。context 为只读 RunContext，backend_instance_id 用唯一 nonce，time_domain_id 明确 MuJoCo 仿真；clock_epoch 初始 0，episode/control 初始空/0。

- `acquire(context, owner, duration_ns, wall_ns) -> lease_id`，`renew(context, lease_id, duration_ns, wall_ns)`，`release(context, lease_id, wall_ns)`。
- `submit(packet, wall_ns) -> ControlReply`：新停稳 PreparedMotion 或流首块 PreparedChunk；ACCEPTED 只准入，实际应用才 STARTED。
- `queue_chunk(packet, wall_ns) -> ControlReply`：同一整体流的后继；有界、幂等并沿用 NativeMotion 迟到/断供行为。
- `stop(context, lease_id, reason, wall_ns) -> ControlReply`。
- `step(wall_ns) -> ControlReply`、`poll(wall_ns) -> ControlReply`；前者只推进一步，后者只处理墙钟，二者都先落实租约到期。
- `query_command(context, command_id) -> ControlStatus`、`query_chunk(context, command_id, chunk_seq)`：严格只读，不续租、不步进、不发新事件。
- `chunk_anchor(context, lease_id, wall_ns, start_sim_time_ns=None)`：先落实显式墙钟和所有权再委托 NativeMotion；不能从回调读可写 MjData。
- `reset(context, lease_id, seed, wall_ns) -> RunContext`：仅静止 IDLE/COMPLETED/STOPPED/FAILED 可复位；活动或 STOPPING 拒绝，先走实际停止。严格非负有界整数 seed，拒 bool。

ControlReply 冻结包含本次 `snapshot: MotionSnapshot`、当前整体 `status: ControlStatus | None`、本次产生的少量整体 `events`；queue 另有 accepted/reason 或等价明确提交结果。保留 snapshot.chunk_events，不另造无界 outbox。事件包含本组件递增序号、原 context/command 身份、phase/reason、真实 sim 时刻。日志 writer 留接线层，由调用方使用已有 EventLogger。

## 租约和完成语义

租约沿用现有协议 0.1～10 s、非空 owner/episode、正 control_epoch，按显式单调 wall 计时。拒 bool、负数、回退、溢出和坏 context，拒绝不能续期/清除合法租约。新 acquire 不能接管活动/STOPPING 控制器；无活动时可在 FAILED 获取租约供显式 reset，但不能绕过 FAILED_LATCHED 开新运动。

release/到期立即撤销授权并请求原生刹车；不能立即把活动命令设为 terminal。整体阶段为 ACCEPTED、STARTED（涵盖运动/最终SETTLING）、STOPPING、COMPLETED、CANCELED、FAILED。STOPPING 只代表已请求停，真实 Native STOPPED 后才 CANCELED；Native FAILED 仍 FAILED 并保留原 stop_reason。新 acquire/命令必须等真实停止。重复 stop/release/轮询不能延长刹车期限。停钟情况下租约仍到期，之后独立 stop wall 超时为 FAILED，不伪造物理停止。

NativeMotion 一个 motion_seq 跨所有块；正常 STARTED 和最终事件各一次，中间块仅产生 chunk 事件。停止故障优先级沿用 NativeMotion，不让迟到人工 stop 覆盖既有超时。aggregate status 与冻结终态/后续实时 snapshot 分开。

## 幂等、有界与 reset

同 ID/seq/完整 payload 重试返回保存事实，不重复执行/消耗序号/续期/发新 ACK；先验证当前租约与 generation。相同 ID 但 seq、context/lease、来源或计划不同为 COMMAND_ID_CONFLICT。新整体 command_seq 严格递增，拒绝不消耗序号。query 可在同 generation 内按原 context 跨租约读取，其他 context 拒绝；缺记录 RESULT_EXPIRED。

块重试按 command_id/chunk_seq+完整 packet 指纹，已经 ACCEPTED/STARTED/RETIRED/CANCELED 的同包不能再执行；冲突 CHUNK_ID_CONFLICT。未接受的无效包不能占槽。保留 chunk 对应的真实阶段/原因，查询不能猜测；回执按原流/event_seq 去重，不把 poll 返回同一快照重复发新事件。已淘汰旧块给 CHUNK_RESULT_EXPIRED，不能重当新块。

命令历史和块历史均有严格正整数上界；活动整体与活动/pending 块不可被淘汰。拒绝新的记录或淘汰最老 terminal 有明确规则，不维护无界 tombstone/list。max 参数拒 bool/零/超大；内存随上界，不随整个运行次数增长。

reset 只由所有者在安全终态执行 Scene.reset(seed)，重新构造同场景的 NativeMotion（真实 hold 用 profile 初值），clock_epoch 增1、episode/control 清空、撤销租约、清空旧命令/块和待执行引用，保持 backend_instance_id，time_domain 同域。旧 context/packet/anchor/查询均 STALE_CONTEXT 或 STALE_STREAM；即使 sim 时间/位置/profile 相同也不能复活。不在运行中重写对象；这是明确 episode 边界复位。

## 验收与交付

保留 643 unit / 81 native / Ruff。新增纯行为覆盖严格输入/幂等与冲突/淘汰/不可变快照；native 至少覆盖：

1. 真正停稳运动：ACCEPTED≠STARTED，测量/参考分离，终态冻结但后续物理继续。
2. 三块连续整体命令：块退休不终结整体，后继幂等不重排，不同包拒绝，最终实测完成。
3. 运动中 release/租约到期：立即撤销授权但只 STOPPING，实际减速后才 CANCELED；停止中拒新 acquire。
4. 仿真停钟时到期→STOPPING→STOP_TIMEOUT；重复请求不延期、原原因保留。
5. 相同 command_id 重试只实际启动一次，改变 observation/计划/seq/lease 冲突；坏请求不消费序号。
6. normal terminal 后续 motion；旧 context 可查询旧终态但不能控制新租约；有界淘汰明确过期。
7. 运动中 reset 拒绝，实际停稳或 FAILED 后 reset 恢复，旧 generation 全部失效、同种子重置确定。
8. 所有新入口跨线程拒绝，不在 submit/queue/step 拟合，不从真值伪造测量。

PROJECT_PYTHON/RUFF 用任务头绝对路径。不得用 host skip 冒充通过。报告 `.build/reports/dsh-025/report.md` 说明精确 API、状态/身份/时钟与所有权、实际自测、待监督者验证、限制；未接 ROS/Policy，不宣称 S3 或正式采集通过。
