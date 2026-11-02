# 连续动作块：时间、衔接与反馈

2026-11-01用户确认采用连续接续，并补齐实现和验收。保留DSH-022停稳执行能力，供完整示教阶段和单步调试使用；连续能力逐项验收后才接ACT。目标、数据量、30s任务时限和训练预算不变。

## 时间和参考

ACT仍只预测双臂16维SI位置，图像/测量输入边界不变。预测点时间锚定**该次观测的机器人捕获时间**，不是服务器墙钟，也不是收到响应后随意选的“现在”。任务书会在Policy接线阶段同步修正尚未生成的proto草稿注释，不能沿用其中与此冲突的执行起点约定。

当前块的结束时刻在请求下一次预测前就已确定。下一块只选择该时刻之后的预定未来窗口，记录原始预测、选中索引和实际执行参考；窗口之前与当前块重叠的预测明确不执行。响应必须在配置的提前量之前到达。迟到则拒绝并刹车，不能移动原时间、缩短旧块、跳过迟到部分来补救。

初值使用当前参考计划在接续时刻的p/v；不能用实测跟踪误差重新定基。继续块的末速度可显式提供，位置型策略默认按选中最后两点的后向差分估计，并检查原速度/加速度限位。最后一个明确要求停稳的块使用零末速度，若其原目标不允许合法减速则拒绝，不能retime或clip。左臂预测不替换成常量，待机偏差守卫在运行接线时独立验收。

每块内部使用固定SciPy CubicSpline，指定起止参考速度；块间p/v连续，允许加速度阶跃，不宣称jerk受限。共享显式计划、连续极值检查仍用标准库。连续块只能在有效区间采样，不能落入旧数学采样器的“过horizon立即零速hold”；无后继时必须走真实减速和停止反馈。

固定版本实验已保留：合法3s轨迹切成三个1s/20Hz块，强制每块零端点速度产生约4.59～4.65rad/s²峰值，携带原边界速度时约0.10rad/s²。限位仍为1.0，不以放宽阈值消除问题。实验见 `var/runs/s3-control-design/20261101/action-chunk-boundaries/`。

## 执行与ACK

只有一个物理数据所有者和一个活动运动序列，最多一个已验证待执行后继。编译在物理循环外完成，队列准入检查流身份、前驱序号、profile、计划起点、提前期限和p/v衔接。没有无界预取、任意中途抢占或自动重试。真实MuJoCo只经具名ctrl和正常步进驱动。

动作块的ACCEPTED、STARTED、RETIRED分别说明已接收、已实际应用、参考窗口已交接；RETIRED不是停稳或抓取成功。一个连续运动跨越多个块，只有明确收尾并满足原实测稳定/接触口径才COMPLETED；取消/缺块/故障经刹车和实测停止再产生终态。块间不强制停稳，也不把到时作为运动成功。

运动中参考/测量、位置/速度跟踪误差、夹爪接触和跟踪质量持续反馈。持续跟踪失配必须按配置有界停止，计时不能因换块而清零。夹爪闭合接触建立过程中可处于未达标状态，不能一边声称块退休等于抓取完成，一边忽略接触证据；最终完成规则保持用户已确认口径。

仿真时钟用于计划/衔接/任务时限，进度停滞与通信/队列期限使用本进程单调墙钟。不能跨主机比较monotonic绝对值。墙钟断流、无后继、旧流/epoch、迟到、队列满都必须有结构化原因与受控停止证据。运行日志沿用既有EventLogger，高频状态和计划进入之后的MCAP协议，不另建日志框架。

## 分阶段实施

1. DSH-023：携带端点速度的数学计划、观测时间锚定的窗口选择、不可变接续锚点和预编译动作块；不接控制状态机/ROS。
2. 前项验收后分派无ROS连续执行：组合现有物理I/O/刹车，验证有界后继、真实衔接、迟到/缺块/停钟、接触及停稳收尾。沿用一个物理循环，避免复制成两套控制器。
3. 再补公共参考/反馈、租约/命令身份、reset/epoch、ROS及精确三图状态配对，最后接fake Policy。保留旧ROS/MCAP格式，不提前宣称新capability。

每项独立worktree/headless session，最多一次定点返工，监督者运行固定镜像和独立反例后才整合。只有全部S3门槛通过才进入teacher和试采。

技术来源：[SciPy端点导数和系数定义](https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.CubicSpline.html)、[ROS2 control轨迹表示](https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/trajectory.html)、[路径与目标反馈边界](https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)。参考其成熟的时间/期望与测量分离思路；本项目尚未接入该ROS控制器。在线SciPy文档为1.18.0，本地固定1.17.1须实际复核，不升级依赖。

## DSH-023 已实现接口（仅预编译/时间映射）

本项只提供数学计划、观测锚定窗口和预编译块，不接控制状态机、队列、抢占、ROS或Policy，也不让`NativeMotion`/`PreparedMotion`接受新块。共享时间类型只用标准库；SciPy拟合仍只发生在仿真侧预编译。

`robot_contracts.action_chunks`：

- `ActionWindow(observation_id, observation_time_ns, handoff_time_ns, source_targets, target_count)`：`source_targets`时间相对该次观测。校验完整源（1..128点、1..64维一致、有限实数、正且严格递增、horizon≤3s），再用整数计算绝对时刻；`observation_time_ns + target.time_ns`不得超过int64上界。选择绝对时间**严格大于**`handoff_time_ns`的前`target_count`（2..128）个点，只把时间改写为相对handoff的正差值，位置不变；恰在handoff的点不重复。全部过期是`CHUNK_EXPIRED`，未来点不足是`INSUFFICIENT_HORIZON`。完整源、`selected_indices`、`skipped_overlap_count`、`targets`（改写后）和`end_time_ns`（相对handoff）可追溯，尾部不静默丢弃。这是预定窗口选择，不是迟到预测补救。
- `ChunkAnchor(stream_id, predecessor_seq, start_sim_time_ns, reference, robot_profile_hash)`：在预定交接点的不可变参考p/v/a。`stream_id`非空且≤128字符，`predecessor_seq`为0..uint64max-1，`seq`由`predecessor+1`派生；`reference`是`ReferenceSample`。p/v/a是控制元数据，不是新的ACT特征；跨块只匹配p/v，不声称a连续。本项不生成身份、不读Scene。

`robot_backend_sim.chunk_plan`：

- `prepare_chunk(anchor, window, profile, config, *, final_velocities=None, settle_at_end=False) -> PreparedChunk`：要求`anchor.start_sim_time_ns == window.handoff_time_ns`、profile摘要与`anchor.robot_profile_hash`一致、profile关节顺序与控制配置一致。初值p/v来自`anchor.reference`；显式`final_velocities`优先，`None`时用选中最后两点`(q_last-q_prev)/(t_last-t_prev)`估计，并逐轴严格限位。`settle_at_end`必须是bool；True使用零末速度，显式非零末速度与其冲突则拒绝。末速度只作插值边界，不改位置目标或时间。编译前复用共享`validate_reference_inputs`对anchor初值、完整`window.source_targets`（含不执行前缀和尾部）、joints、加速度和边界速度做规范化/全源限位检查；随后用配置中已有最大加速度编译一次并共享验证，采样不再拟合。
- `PreparedChunk`冻结保存`anchor`、`window`、`plan`、`joints`、`max_accelerations`、`final_velocities`、`settle_at_end`；名称/维数/时长/序号由属性派生。直接构造同样校验计划、声明的p/v边界和完整源。除当前曲线p/v/a合法外，还必须用计划**实际末端p/v**调用既有`.braking.plan_braking`预演原关节/原最大加速度下的缺块刹车；该刹车计划只用于验证、不保存。零末速度时无额外停车距离，非零末速度若需越过位置限位则拒绝。`PreparedChunk`要求最大加速度为正数且与`PandaControlConfig`一致；数学`ReferencePlan`本身仍支持零加速度的旧语义。`sample_at(sim_time_ns)`只接受整数并检查绝对区间`start..end`，用整数差转本地elapsed后走`ReferencePlan.sample_interval`；区间外`REFERENCE_EXPIRED`。它不读时钟/Scene、不做IO/网络/日志/线程。

缺块退路是预执行门槛，不是运行时才发现的属性：曲线p/v/a全部合法不等于该块结束后可以安全等待后继；非零末端速度必须仍能在原限位内停车，否则准备阶段即拒绝，不能把速度瞬间置零或放宽限位。

时间约定：窗口和chunk内部时间均为整数ns；`ActionWindow.targets`以handoff为零点，`PreparedChunk`的绝对有效区间是`[anchor.start_sim_time_ns, anchor.start_sim_time_ns + plan.duration_ns]`。速度单位仍为各关节SI单位/秒。host缺少固定SciPy时native测试只能skip，不能据此宣称原生通过；固定镜像仍需复验边界导数、3s轨迹切块恢复、16维静止左臂/夹爪m通道和坏前缀拒绝。

## DSH-024 已实现接口（连续执行，尚未接 ROS/Policy）

`NativeMotion(..., stream_config=StreamConfig)` 以可选关键字扩展现有单运动状态机；
`None` 保持 DSH-022 行为，stream 入口为 `STREAM_NOT_CONFIGURED`。新增
`chunk_anchor(start_sim_time_ns=None)`、`start_stream(first, wall_time_ns)`、
`queue_chunk(successor, wall_time_ns) -> ChunkSubmission(accepted, reason, snapshot)`。
一个活动块、一个 pending、一个 `motion_seq` 跨越全部块；等待起点期间继续 hold/真实
物理，边界只写一组 ctrl/step 一次，非 final 无后继从最后应用 p/v 触发 `UNDERRUN`
刹车，final settle 块终点后显式 hold 并沿用原实测稳定/接触完成规则。事件、跟踪诊断、
墙钟/进度/跟踪期限和精确字段见 [native-motion.md](native-motion.md)。流身份用实例
uuid nonce + 精确下一 motion 代次；queue 前先落实已提交 wall 的流/进度期限。本项仍未接
lease/reset、ROS ACK/MCAP、Policy 或正式 episode，不宣称 S3 整体通过。


DSH-024 的原生执行层已经固定镜像验收；详见 [验收记录](tasks/dsh-024-result.md)。
这里的 chunk 事件由调用方写入已有 EventLogger；公共 ROS ACK、MCAP、lease/reset、
精确观测配对和 fake Policy 接线仍是 S3 后续门槛。示教与正式采集从这些门槛全部通过后开始。
