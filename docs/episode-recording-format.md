# Episode 记录结果与 v2 迁移设计

2026-10-30，DSH-010 已实现本设计中的 manifest v2 结果分类、版本分支读取、记录完整性目录发布和人工 EndEpisode 语义。v1 的 `complete` 同时承担写入完整性、正常执行和mock演示检查，不适合人工停止或后续物理任务。新格式分开这些含义，历史记录不原地改写。

## 结果字段

新文件 `contracts/schemas/v2/episode_manifest.schema.json` 只升级episode产物格式，不升级Robot/Policy协议或其他v1配置。

| 字段 | 含义 |
| --- | --- |
| `schema_version: 2` | 明确选择v2读取规则；未知版本拒绝 |
| `recording.complete / recording.failure_reason` | 已接受的原始记录是否排空、关闭、通过文件与引用校验；不代表任务成功 |
| `runtime.outcome / runtime.reason` | `completed`、`stopped`、`interrupted`、`failed`；分别是流程正常完成、人工结束、进程信号中断、实际故障 |
| `task.outcome / task.reason` | `not_evaluated`、`succeeded`、`failed`；当前mock始终not_evaluated，不能由脚本退出自动推导成功 |
| 其余溯源字段 | 保留episode ID、profile/observation/action哈希、起止时间、实际消息数量、文件哈希和配置；另明确run ID、source与test_only |

v2不保留含混的顶层 `complete` 或 `failure_reason` 别名。只有实际任务判定器才能在后续阶段填写任务标签。新数据被纳入训练前仍需S4/S5的成功、图像、动作标签和完整性质量门槛；文件完整不能代替这些检查。

## 完成与目录

写入中的目录为 `.partial-<episode_id>`。原生writer无法排空、审计失败或收尾异常时保留partial；磁盘本身不可写时可以没有最终manifest，此时只能判未完成。

记录排空且文件/引用校验通过后可发布为 `<episode_id>`，即使runtime是stopped/interrupted/failed。目录名只说明原始记录产物完成，不能据此判断运行或任务成功。异常状态保留在manifest和事件中，原始记录不混入合格数据清单。

合法取消、静止左臂、提前停止，以及没有成功执行的命令，都不自动等于文件损坏。generic audit检查已记录消息的内容、上下文、引用和合法事件顺序；命令未获得terminal ACK的数量另报。丢帧/吞吐/任务质量由对应质量门槛处理，不能声称generic audit证明传输链路从未丢包。

## 三相机 profile 与 audit（DSH-018）

schema v2 的 RobotProfile 使用有序 `cameras` 列表，当前双 Panda 为
`global`、`left_wrist`、`right_wrist`。runtime 按该顺序订阅
`~/cameras/<name>/frame`，BackendInfo 和 ObservationIndex 必须包含全部相机；
记录和 audit 使用同一份从 profile 快照规范化的相机列表，不另写一套解析。

对每个相机，audit 流式检查：话题名与相机名匹配、图像宽高、`rgb8` 载荷长度、
frame_seq 递增、与 ObservationIndex 的 state/frame 引用及因果时间差。v2 profile 还
逐路检查紧密 RGB 步长（`width*3`）、`Image.header.frame_id`、`CameraInfo` 的宽高与
帧 id、以及 `calibration_digest` 与 profile 一致；漏一路、未知/重复相机名、顺序
不符、跨相机引用或错误元数据都会使记录失败。历史 v1 profile 仍按其原有单相机
规则读取，不强制它未声明的校准摘要或完整 CameraInfo，也不改写旧 manifest。

这只是原始流完整性与身份检查，不证明物理相机安装、同步质量或“延迟渲染精确快照”
已解决；后者仍需在 S3 物理适配中完善。

历史v1的读取保持严格旧含义：仍要求其 `complete`、mock运动和完整完成事件等规则。测试必须覆盖新写v2、旧v1读取和未知版本拒绝，不将旧数据悄悄按新规则解释。

## 命令状态与反馈记录（DSH-019）

新录制在存在 backend 时增加 `/cell_01/robot/command_feedback`
（`robot_api_msgs/msg/CommandStatus`）。记录时间戳取 `sample_time`，与 state/frame
一样保留原始机器人时间；活跃期间每个 tick 一条，terminal 至少一条，空闲后不重复。
数组不写入普通 JSONL 事件日志，只进入 MCAP。

feedback 字段包含 command/context 身份、phase/reason、accepted/started/sample 时间、
`execution_clock`、`interpolation_version`、实际执行时长 `elapsed`、joint 顺序、命令
开始参考 `initial_positions`、当前参考/测量/误差和 progress。mock 的
`execution_clock=steady`、`interpolation_version=linear_v1`，参考等于合成测量，
`tracking_error` 全零；这只证明状态、时钟与记录链路，不证明物理跟踪或接触。
`elapsed` 不是 `sample_time - started_at`，不能用两个不同时钟相减合成。

audit 在 MCAP 声明 `command_feedback` topic 时启用新检查，并流式保留每条命令的有界
摘要而非整段数组：身份、joint 顺序、有限性、`reference - measured`、sample/elapsed/
progress 单调性、terminal 快照不回退或漂移。每个样本都按实际接受轨迹、
`initial_positions`、`elapsed` 和 `interpolation_version` 重建参考与 progress，不能
只验最终目标，因而中间参考错移或 progress 不符也会失败。feedback 的 command_seq/
observation_id 必须与已记录 command 一致，accepted/started/terminal 时间必须与对应
command event stamp 一致。声明该 topic 时，所有已接受命令都必须有终止反馈；没有该
话题的历史 v1/v2 记录不新增要求。首命令的 initial 不假定等于 profile 初始位姿，只
在同一 episode 已有上一 terminal 时检查连续性。异常反馈使记录失败并保留 partial 目录。

## 人工结束与故障

EndEpisode ACK仍只是接受结束请求，返回ENDING；最终生命周期状态和manifest才说明收尾完成。人工结束设置单独的结束意图，停止后续脚本段，独立停止/释放并排空记录；它本身不是运行故障。反复请求不会覆盖第一次原因或启动第二个finalizer。

结果优先保留实际故障，其次信号中断，再次人工结束，最后正常完成。故障与后来的信号/结束原因均保留，不能用人工结束遮住控制或写盘错误。若人工结束在正常收尾已开始后到达，不倒改已经完成的脚本结果；服务响应必须如实说明请求状态。

正常人工结束后runtime服务可接受下一episode；信号中断继续使进程退出130/143；worker收尾超时继续124；实际故障使`--once`非零退出。进程退出码与episode结果分别报告；不将信号中断写成completed，也不把未评估任务写成succeeded。

## DSH-010 实现状态

- 新写记录统一使用 `contracts/schemas/v2/episode_manifest.schema.json`。顶层无 `complete`/`failure_reason`；`recording`、`runtime`、`task` 三个对象必须同时存在，`runtime.outcome` 和 `task.outcome` 由 JSON Schema 枚举约束，`schema_version`、`run_id`、`source`、`test_only` 等身份字段仍保留。
- `robot_contracts.episodes` 提供 frozen `RuntimeResult`/`TaskResult` 与枚举，以及显式版本到 schema 目录的映射；未知版本拒绝，不接受用户提供的任意路径片段。
- `EpisodeRecorder.finish()` 只根据 writer 排空、实际写入计数、流式 audit 和分块 hash 决定 `recording.complete`。合法 stopped/interrupted 不会使记录损坏；runtime 结果由 `EpisodeRunner.current_runtime_result()` 在排空、audit、hash 和发布前后反复读取。若记录确实失败，runtime 至少为 failed。
- 只有 `recording.complete=true` 才把 `.partial-<id>` 改名为 `<id>`。runtime/task 结果不参与目录命名，失败或停止但记录完整的产物仍是正式 `episode-*`，消费端必须读 manifest 维度。
- `inspect_episode()` 按 `schema_version` 分支：v1 保留旧的严格 complete/mock 运动/完整 terminal 规则并额外返回明确的兼容维度；v2 允许静止、零命令/零观测和合法命令前缀，报告 `commands_without_terminal` 与 `terminal_events`，但绝不把 `task=not_evaluated` 当作采集或任务成功。
- EndEpisode 返回 ENDING；重复请求幂等；没有活动 episode 拒绝。人工结束设置单独意图并请求 worker 停止，不写入 FailureLog；实际控制/记录/日志故障仍为 failed 并优先于停止/信号原因。正常收尾开始后的请求不会倒改已提交结果。下一 episode 会重置意图、context、recorder 和结果。


v2诊断记录继续将缺失terminal事件计入 `commands_without_terminal`，与runtime故障
分别报告；若另有terminal反馈，可保留并检查其轨迹和已有事件。声明反馈topic后，
已接受命令的terminal反馈缺失仍使记录不完整。两种结果都不能用于正式成功数据。
实时接线和离线审计共用 `validate_status_transition`，只保留每命令最后一个不可变
CommandStatus，统一拒绝起始元数据/执行时钟改写、阶段倒退和terminal快照漂移。
