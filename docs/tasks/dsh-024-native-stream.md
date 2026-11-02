# DSH-024：有界连续动作块的原生执行

2026-11-01。DSH-023已验收，608 unit/65 native，用户选择连续接续。[总体决定](../continuous-action-design.md)。首轮35分钟，最多一次10分钟返工，交付报告即退出。

## 范围与可读性

允许修改 `components/backend_sim/src/robot_backend_sim/motion.py`、`chunk_plan.py`（只增加必要的派生只读属性）、`tests/integration/test_native_motion.py`（保留旧断言，仅为确实复用的API变化作必要调整）、README、`docs/README.md`、`docs/native-motion.md`、`docs/continuous-action-design.md`。

允许新增同包 `stream_config.py`、`stream_schedule.py`；配置 `configs/simulation/panda_stream.yaml`；测试 `tests/unit/test_stream_config.py`、`test_stream_schedule.py`、`tests/integration/test_native_stream.py`。

不改数学契约/编译器/刹车/接触/PandaIO、ROS/Policy、mock/runtime、profile/场景、依赖/镜像/启动器；不安装依赖、Docker、非固定native缓存、申请权限、修改其他工作树、启动代理、提交Git。host缺native可skip，由监督者固定镜像验收。不宣称ROS/Policy或S3整体通过。

扩展**现有NativeMotion**，以小型具体调度对象组合单活动块/单后继；不继承mock、不复制第二个物理控制器。严格提取共用的“apply ctrl→mj_step一次→mj_forward→measure/contact”辅助函数，整包NativeMotion只保留一个真实mj_step调用位置。旧停稳、hold、stream和brake共用它。step通过清楚的参考选择/阶段处理函数组成，避免再堆一个几百行分支。无通用插件、事件总线、无界历史或新线程。

## 配置与API

严格不可变StreamConfig单独加载，不给原PandaControlConfig增加必填项。新构造参数为仅关键字 `stream_config=None`；None时旧NativeMotion行为不变，stream API明确STREAM_NOT_CONFIGURED。建议API（等价清楚命名可接受，文档列精确名称）：

- `chunk_anchor(start_sim_time_ns=None)`：只读、所有者线程，返回ChunkAnchor。idle/正常终态时预览下一motion的初块，stream_id含本NativeMotion实例唯一标识及下一motion序号；predecessor=0，参考是最后hold的p、v/a为0。None按配置initial_lead选对齐物理步长的未来时刻；显式时刻不偷偷改。active stream时只返回当前块end处的未来参考p/v/a和其seq，已有pending则拒绝；final settle块/STOPPING/FAILED不再给后继锚点。
- `start_stream(first: PreparedChunk, wall_time_ns)`：准入后进入ACCEPTED，到预定起点实际应用/step才STARTED/RUNNING。等待期间继续hold和真实物理，不暂停等预测。拒绝与旧停稳/其他stream重叠。旧start开始新停稳动作时清理前一stream的私有调度状态。
- `queue_chunk(successor, wall_time_ns)`：返回清楚的 `ChunkSubmission(accepted, reason, snapshot)`；最多一个pending。验证流身份/前驱序号/起点/profile/joints/accel及边界参考匹配，不重验整条曲线、不在物理线程调用SciPy。队列不是任意抢占，下一块只能在前块结束处接续。

所有新入口/只读状态都检查创建Thread；Scene/PandaIO仍同一对象。新实例的旧锚点必须STALE_STREAM，正常结束后旧流也不能再提交。单调wall参数规则保持；锚点读取不改变phase、期限、序号或物理状态。编译在调用方外部做，控制方法不能sleep或读网络/文件。

配置候选（秒，转换为有界整数ns，拒未知/缺失/重复键、bool/非有限/溢出/非正值，错误含路径字段，提供完整新鲜快照）：initial_lead_s=0.25、min_lead_s=0.05、max_observation_age_s=0.5、stream_wall_timeout_s=120、progress_wall_timeout_s=3、tracking_timeout_s=2。initial_lead≥min_lead，NativeMotion结合模型检查min_lead≥一个物理周期。位置/速度/加速度和接触阈值仍来自原profile/PandaControlConfig，不放宽或隐藏新容差。

start/queue检查window.observation_time≤当前sim，且不超过max_observation_age；不能接受未来观测。计划开始/结束必须落在当前物理步长网格（当前2ms），不对输入时间取整来修复；内部路点无需都落网格。20Hz动作周期50ms支持该网格，其他组合须明确校验。准入截止为handoff-min_lead，恰到截止可接受。匹配本流/前驱的合法候选迟到或观测过期应REJECTED并启动锁存故障的刹车；旧流、错序、错误锚点、队列满等无效包仅拒绝、不破坏当前合法计划。拒绝不能消耗待执行槽位或序号。

## 参考选择与真实交接

开始前HOLD；有效区间严格sample_at。当前块终点与后继起点在同一物理时刻共享p/v；该时刻只写一组ctrl、step一次。可以在该边界使用后继的初点，同时明确记录前块RETIRED、后继STARTED，二者引用同一真实应用/测量事实。不插一帧hold，不重定基到实测，不改任何预测位置/时间。实际参考与独立完整原曲线/原生ctrl重放必须可核对。

非final块到终点没有pending：仍实际应用该块合法末端一次，然后从最后应用p/v及其时间触发UNDERRUN刹车；绝不调用过期sample_at或零速hold假装停止。最后明确settle_at_end=True的块末速度已验证0，可在其终点后显式hold末目标、进入原SETTLING，原实测稳定/接触规则达标才COMPLETED。不能把它重新包装成新motion、改ID或重复started。末端刹车余量已经由PreparedChunk预验。

外部stop、缺块/迟到、跟踪故障均复用现有有限减速和实测停止；不强制张开夹爪。丢弃pending有明确CANCELED回执且从未STARTED。重复stop不延期，STOP_TIMEOUT保持最后实际应用参考，故障后新start/stream均不能绕过锁存。计时/原原因保留。整个stream墙钟期限用新120s，旧停稳仍用原10s；仿真进度停滞独立按墙钟3s触发停止，poll不能伪造物理进展。stop仍按原3s墙钟期限，持续停钟必须失败。

## 反馈与回执

MotionSnapshot保留旧字段，新增stream_id、当前实际应用chunk_seq（等待时无active chunk）、reference_kind（HOLD/TRAJECTORY/CHUNK/BRAKE）、速度跟踪误差、连续跟踪质量/超限时长，以及本次操作产生的不可变chunk_events。名字可等价清楚，默认值保证旧调用成立。

ChunkEvent含本流递增event_seq、motion/stream/chunk身份、源observation_id、计划起止sim时刻、该事件的真实sim时刻、phase/reason。阶段至少ACCEPTED/STARTED/RETIRED/CANCELED/REJECTED。每次API操作只返回少量本次事件（通常≤2，边界+故障可≤4），无无界outbox；poll重复读取同一快照可以返回同一event_seq，但不得产生新事件，正常step也不得反复退休旧块。调用方按事件ID记录一次。RETIRED只表示参考窗口已交接，**不是运动成功/停稳/抓取成功**。

NativeMotion的一个motion_seq跨越所有块，整段仅最后停稳时COMPLETED，或刹车实测停止后STOPPED/FAILED。冻结MotionResult/terminal_snapshot与后续实时物理快照仍分开。不要让“块结束时间到”变成成功终态。

## 连续跟踪与接触

运动中臂与自由夹爪用位置误差及 **参考速度-实测速度** 判断跟踪，不能要求非零参考下实际速度为0。阈值沿用原position/velocity_tolerances。夹爪闭合被同物体双指约束时，可用原接触证据规则判CONSTRAINED，仍保留目标、宽度/速度误差；接触建立尚不达标可显示OUTSIDE_TOLERANCE，不伪造完成。

任何轴持续跟踪不达标达到新tracking_timeout（2s仿真）进入刹车，停稳后FAILED/TRACKING_TIMEOUT。全体重新达标才清除超限计时，**不能因queue/换块清零**。接触建立过程不必每个块都达停稳口径，但持续失配有界；最终SETTLING仍使用旧的绝对速度/位置、同物体/稳定窗口，未达标不能COMPLETED。状态明确区分FOLLOWING/CONSTRAINED/OUTSIDE等诊断与goal完成，不能改测量来通过。

## 必须验收

608 unit/Ruff与原65 native保持。新纯配置/调度测试覆盖直接构造/快照不可变、严格配置、边界、单pending、身份/顺序、提前期限、时钟和终点模式；不以实现的相同函数互相证明正确。

固定原生至少覆盖：合法3s/右q1+0.15rad轨迹按50ms采样分三个1s块，在非零预定起点启动、运动中预编译/排队后继、第三块settle；p/v连续、每步时间/真实测量、左臂保持、唯一started/retired序列、跨块不停车、最终实测稳定，独立raw ctrl/step/forward重放qpos/qvel/自由物体一致。source观测时刻/窗口要真实符合测试当前sim，不用过期观测凑正例。

另测：缺后继、合法但迟到、队列满、错误流/前驱/锚点/profile、未来/过期观测、非网格时间、实例/正常结束后的旧流、stop丢弃pending、final禁止后继、wall固定/跳变、物理停钟poll/停止超时、总体wall上限；持续跟踪故障跨块不能重置（至少一个真实物理故障注入，例如只在测试中让I/O丢失一个执行器目标，测量仍真实）、新入口跨线程、不能在step/queue重拟合。

沿用仅运行前初始化一次的自由块接触夹具：右夹爪目标0/实际约4cm，两指真实合力/速度证据，连续小幅移动且最终CONTACT完成；不焊接/重写物体/补外力/改模型来通过，不计teacher/正式episode。

不新建日志writer/启动器。返回有界事件和状态，由监督者用既有EventLogger记录阶段/回执/故障，高频轨迹独立诊断；完整ROS/MCAP与日志故障停止留接线阶段验收。报告 `.build/reports/dsh-024/report.md`：API、事件顺序/时钟、共用物理步进、host实跑/skip、待native项与限制。只用任务头PROJECT_PYTHON/PROJECT_RUFF，不以native skip冒充通过。
