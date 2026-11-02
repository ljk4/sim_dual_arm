# 运行日志与原始记录约定

2026-10-30，S1c 的设计输入。公共事件日志库已验收；DSH-008 已完成 runtime 接线，DSH-009 已完成 MCAP 有界写入、流式审计和分块哈希，DSH-010 已完成 manifest v2 三维结果、v1 严格兼容读取和人工结束语义，DSH-013 已由自动采集启动器写顶层运行汇总状态（`status.json`）。目标和阶段门槛见 [仿真闭环计划](minimal-simulation-loop-plan.md)。本文定义诊断与证据的边界，避免各组件分别发明格式。

## 文件和关联

一次采集、训练或评估有一个 `run_id`；启动器传给相关进程。独立启动可生成新 ID，不能复用旧运行目录覆盖已有结果。每次进程启动另有 `instance_id`，防止不同容器的 PID 都是 1 时冲突。

```text
var/runs/<run_id>/
  resolved/                   # 各组件实际配置，包含默认值
  versions.json               # 明确采集的代码/依赖/镜像信息；不倾倒环境变量
  status.json                 # running -> succeeded/failed/interrupted；原子替换
  logs/<component>/<instance_id>/
    events.000001.jsonl        # 轮转后递增编号，不覆盖当前 run 的审计证据
    events.000002.jsonl
    summary.json              # 写入数量、丢弃诊断计数、错误、关闭是否排空
  metrics/                    # 训练 loss、吞吐、延迟、显存、质量统计
```

原始图像/状态/动作在 `var/data/raw/` 的 MCAP，模型在 `var/artifacts/models/`，只通过路径与摘要关联。日志文件不重复保存图像、tensor 或完整高频状态。失败 episode 保留独立目录及失败原因，不混入合格训练数据清单。

Compose mock 开发路径把这两个边界同时落到宿主机：`var/data/raw/<run_id>/` 挂到 runtime 的 `/data/raw/<run_id>/`，`var/runs/<run_id>/` 挂到 `/data/runs/<run_id>/`。宿主启动入口只创建两个父目录，不预先创建 runtime 的独占 run 目录；生成契约和 colcon 输出单独放在 `.build/ros-dev/`，不写入数据或日志目录。runtime 仍按 `RunSession` 的规则独占创建 `<runs-output>/<run_id>`，已有同名目录时启动失败，保留旧产物而不是覆盖。

`status.json` 初始为 running；只有相关进程正常退出、关键日志排空和必要产物验收通过，监督启动器才写 succeeded。崩溃、断电或磁盘故障可能无法再写状态，遗留 running/partial 本身表示未完成，不能按目录存在推定成功。

整次运行的 `status.json.recording.complete` 表示启动器已独立确认完整；只有写入器声明完整且本次 audit 通过才为 true。`recording.manifest_complete` 单独保留原始 manifest 的声明，未知时为 null；未运行 audit 或 audit 失败时都不能宣称已验证完整，应结合 `audit` 与 `errors` 区分未验证和校验失败。原始 episode v2 manifest 不改写，已完成的 runtime 结果也不会被启动器/审计故障倒改。

## 整次运行监督（DSH-013）

自动采集入口 `tools/run/mock_collect.sh` / `mock_collect.py` 调用 `tools/run/collect_run.py`，复用同一 Compose 栈（`mock.yaml` + 只增加 `--once` 的 `mock_collect.yaml`）和已验收的容器身份检查；独立 audit 再叠加 `mock_audit.yaml`，把 workspace/build/raw/runs 挂载全部覆盖为只读。启动器参数来自 `configs/deploy/mock_collect.json`；字段后缀明确单位（`_s` 秒、`_bytes` 字节），未知字段、bool 冒充数字、NaN/Inf、非正数和超过合理范围的预算在启动前拒绝，有效值及 SHA-256 写入 `launcher.json`。整次工作期限包含启动、episode 和独立 audit；停止 grace、cleanup、launcher 日志 close 各有独立上限，轮询和重试不重置工作期限。默认最坏等待见 [阶段 2 使用指南](stage2-quickstart.md) 第 4 节。

`status.json` 遵循 `contracts/schemas/v1/run_status.schema.json`，从 running 原子替换为 succeeded/failed/interrupted，字段包括阶段、稳定 reason、UTC 起止时间、monotonic 耗时、组件容器 ID 与实际退出码（未知为 `null`，不补 0）、recording/runtime/task 三维结果、audit 结果与证据路径、runtime/launcher 关键日志排空、cleanup 结果和附加错误。主故障先记录，后续停止/日志/清理错误追加，不覆盖为最后一个不相关错误。

启动器自身使用公共 `robot_run_logging`（host 只把 `libraries/run_logging/src` 加入源码路径），`component=launcher`，记录启动、容器创建/退出、超时、audit、停止和收尾等低频关键事件；高频轮询不逐事件写日志。launcher 日志预算来自同一 JSON 配置，`summary.json` close 失败、关键事件队列满或写入失败都会把结果改为失败，不会先写成功再忽略 close。Docker/子进程诊断另存为有明确字节上限的 tail，截断会记录在证据中；它不冒充完整关键事件流，MCAP 也不重复保存日志。

runtime 的 `<runs-root>/<run-id>/logs/runtime/<instance_id>/summary.json` 必须唯一、身份匹配（run/component/instance）、`state=complete`、attempted/enqueued/written 自洽。启动器还会逐行流式读取实际事件文件，校验每行 JSON、run/component/instance、event_seq 合法且严格递增不重复，并要求全部文件的实际行数等于 `written`；合法诊断丢弃只允许序号出现空档，不能出现截断、坏行或身份错配。目录存在、旧 summary 或 `run.finalizing` 都不是退出证据。独立 audit 在一次性工具容器中读取候选源码/SDK 和只读 raw 数据，结果保存在 `<runs-root>/<run-id>/artifacts/audit.json`；audit 失败或关键日志不完整时保留 partial、manifest、控制日志和退出码，不自动删除失败数据；整体状态失败但匹配本次身份的 manifest 三维结果原样保留。cleanup 只移除已核对 ID 的本次 runtime/backend/tool，网络删除前检查外来端点；同 project label 的未知资源保留并写入 `cleanup.foreign`，不批量删除。

## 事件格式

每行是一个 JSON 对象，schema位于 `contracts/schemas/v1/run_event.schema.json`。`fields` 的值只允许有限数字、bool、字符串或null，不接受列表或嵌套对象；汇总用计数，单命令用上下文command_id。禁止把任意对象 `str()` 后灌入日志。

| 字段 | 规则 |
| --- | --- |
| `run_id/component/instance_id/event_seq` | 进程内序号递增，进程身份稳定；序号和计数可以解释诊断丢弃 |
| `wall_time_utc/elapsed_ns` | UTC 用于人工定位；耗时用同一进程的 monotonic，不跨主机直接相减 |
| `level/event/reason` | level 表示严重程度，event 是稳定事件名，reason 是可检索原因码，文本说明另放 message |
| `episode_id/request_id/command_id` | 适用时填写；缺少时为 null，不伪造 ID |
| `clock_epoch/sim_time_ns` | 有仿真上下文时填写；不能拿墙钟当仿真时间 |
| `critical` | 表示证据不可丢弃，不等同于 ERROR 级别；如命令接受、停止、episode 结果可以是 INFO 但 critical |
| `fields/exception` | 有界诊断信息；异常包含类型、信息与堆栈，超长说明明确标记截断 |

ID、事件、上下文和原因不能为适配大小限制而悄悄截断。详细 message/堆栈可截断并记录原长与标记。API 对不支持的数据类型明确报错，避免迟至 writer 线程才发现序列化失败。

建议事件按职责命名，例如 `run.started`、`episode.preparing`、`control.acquired`、`command.accepted`、`command.started`、`command.finished`、`policy.request_failed`、`recording.finalized`。不为每个高频状态产生一条 critical 事件；跟踪误差汇总为指标，完整序列由 MCAP 保留。

## 基础库 API（S1c 已实现）

`libraries/run_logging` 提供独立于 ROS、runtime 状态机和 MCAP 的包 `robot_run_logging`，schema 为 `contracts/schemas/v1/run_event.schema.json`。公开面只有少量类型：

- `LogSettings`：frozen dataclass，默认队列 256 条、单事件 32 KiB、单文件 8 MiB、总预算 256 MiB、关闭超时 3 s、flush 间隔 0.2 s；`console` 默认关闭，可显式打开。容量必须是正整数，时长必须是有限正数，且单事件不大于单文件、单文件不大于总预算。
- `EventContext`：frozen dataclass，保存可选的 `episode_id`、`request_id`、`command_id`、`clock_epoch`、`sim_time_ns`；未知值为 `None`。
- `EventLogger(directory, *, run_id, component, settings, instance_id=None)`：独占创建一个新目录（父级可创建，已有目录拒绝复用），未给出 `instance_id` 时使用 UUID。`directory`、`instance_id` 只读。
- `emit(...) -> bool`：在调用线程做有界校验、JSON 渲染和非阻塞入队；非关键事件因队列满丢弃时返回 `False` 并计数，关键事件失败、无效载荷或关闭后调用抛 `LogFailure`/`ValueError`。
- `check_health()`：写入、flush/fsync、轮转或摘要失败，关键事件被拒绝，或关闭超时后持续抛 `LogFailure`。
- `close()`：停止接收新事件，在 `close_timeout_s` 内排空并 flush/fsync，然后以原子替换写 `summary.json`。成功后可重复调用；失败状态不可被后续 `close()` 改成成功。

实现边界：生产者线程不写文件、不写 stderr、不 join、不回调控件；唯一的 writer 线程拥有事件文件句柄，按 `events.000001.jsonl` 递增轮转且从不覆盖，总预算只统计事件文件字节。失败状态在生产线程只做不可清除的 latch 并立即报错，有界兜底诊断由 writer 线程最多输出一次。永久失败或 writer 退出后，`emit` 对所有级别（包括非关键诊断）都抛 `LogFailure`，不再把事件放进无人消费的队列。`close` 从调用入口计算预算，不等待生产者渲染或其他 close；最终入队与关闭共用一个不含 I/O 的短临界区，迟到事件明确拒绝；writer 以最终摘要提交和 terminal 握手结束，晚到的 `complete` 提交会在握手前改写为 `failed`。`summary.json` 开始时为 `running`，正常关闭写 `complete`，故障写 `failed` 并保留首要原因与 attempted/enqueued/written/dropped_noncritical 计数；磁盘本身写坏时允许留下 `running`/partial，不能伪写 `complete`。该库不调用 ROS 停止、不退出进程、不修改全局 root logger，也不加载 YAML、不读取环境变量。DSH-008 已把该库接到 runtime；顶层 `status.json` 由 DSH-013 启动器负责，MCAP 记录语义见 DSH-009/010。

## runtime 接线（DSH-008）

runtime 每次进程启动都由 `main` 独占创建 `<runs-output>/<run-id>/`，已有同名目录直接失败，绝不覆盖。实现先解析父目录，再在父目录下用合法 run ID 做独占 `mkdir`；现存目录、文件、有效或悬空 symlink 都统一得到 `FileExistsError`，不会跟随 symlink 写到外部 target。省略 `--run-id` 时生成 UUID；省略 `--runs-output` 时使用 `--output` 父目录下的 `runs/`。`--run-id` 只允许字母、数字、点、下划线、连字符，不能包含路径分隔符或 `..`。这两项是部署路径，不进入机器人语义配置。

`configs/runtime/mock_collect.yaml` 的 `logging` 对象显式列出全部默认值：队列 256、单事件 32 KiB、单文件 8 MiB、总预算 256 MiB、关闭超时 3 s、flush 间隔 0.2 s、`console: false`。对象或字段省略时使用默认值；实际生效值写入 `resolved/runtime.json` 的 `recipe.logging`。配置校验错误包含 YAML 路径和 `logging.<字段>`，容量关系仍由 `LogSettings` 复核。

初始化在控制开始前完成：

```text
<runs-output>/<run-id>/
  resolved/runtime.json        # run_id、instance_id、实际配置（含 logging 默认值）
  versions.json                # Python、白名单依赖、实际源码路径、配置哈希、null 镜像/Git 信息
  logs/runtime/<instance_id>/
    events.000001.jsonl
    summary.json
```

`versions.json` 只收集 Python 版本、明确列出的依赖发行版版本、`dual_arm_runtime`/`robot_contracts`/`robot_run_logging`/`robot_ros_support` 的实际导入路径、配置与 recipe 的 SHA-256，以及显式的 `image.digest: null`、`git.revision: null`。它不倾倒环境变量、不读取凭据，也不在源码未提交时声称 HEAD 与工作树一致；镜像 digest 和未知 revision 留给启动器补充。

进程只使用一个 `EventLogger`，事件路径固定为 `logs/runtime/<instance_id>/`，不写入 episode 的 MCAP 目录。`summary.json=complete` 只表示日志 writer 有界排空成功；它不代表任务成功。DSH-013 启动器只有在 runtime 真实退出 0、这份 summary 完整排空、独立 audit 通过、launcher 日志 close 成功和必要 cleanup 完成后才写 `status.json=succeeded`。

关闭前只写 `run.finalizing`，字段 `exit_code_before_log_close` 和 `log_close_pending=true`；正常路径 reason 为 `CLOSE_PENDING`（不是 `COMPLETE`），表示尚未完成日志关闭，不能宣称进程已经退出或成功。`logger.close()` 完成后，main 在 stdout 输出最终摘要：`exit_code`（关闭失败时可能已从 0 改为 1）、`process_outcome`（`succeeded`/`failed`/`interrupted`）和 `logging_closed`。该摘要仍只描述 runtime 进程，不是全系统状态或物理任务成功；实际 OS 退出码由启动器复核并负责写顶层状态。

显式记录的事件包括：

- `run.started`、`run.stopping`、`run.finalizing`：run/instance 身份、配置与输出路径、停止原因、关闭前退出码；主线程异常在最终事件保留类型、信息与堆栈。`run.stopping` 的原因由信号、主线程致命错误或 `node.failure_reason` 共同分类，普通控制/episode 失败不会写成 `COMPLETE`。
- `episode.preparing`、`episode.active`、`episode.end_requested`、`episode.ending`、`episode.terminal_wait_failed`、`episode.finished`：task、record 目录、主故障与最终原因；`episode.finished` 明确给出 `recording_complete`、`runtime_outcome/runtime_reason` 和 `task_outcome/task_reason`，不使用含混的顶层 `complete` 成功字段。
- `control.acquire_requested`、`control.acquired`、`control.acquire_failed`；`command.sending` 意图；`command.action_accepted`/`rejected`/`stopped`/`result`/`failed`；后端真实 `CommandEvent` 的 `command.accepted`/`started`/`completed`/`failed`/`canceled`。action ACK 不作为执行完成，执行完成只来自 action result 或后端事件；明确受控停止为 WARNING，真实 action/timeout 故障仍为 ERROR。
- `control.stop_requested`/`completed`/`failed`、`control.release_requested`/`completed`/`failed`，两步独立尝试；`request_id` 只是本地日志关联 ID，不改变服务 wire 协议。正常续租静默，`control.renew_failed` 可查。
- `recording.completed`/`recording.failed`：路径、`recording_complete` 与 `recording_failure_reason`；日志不把记录完成等同于物理任务成功，也不把合法停止写成记录损坏。

命令事件携带 `command_id`、`episode_id`、`clock_epoch`、可用的仿真/robot 时间，以及 `command_seq`、`observation_id` 等少量标量；不复制关节数组或图像。未知时间填 `null`。

严重度由调用点显式传入，不按事件名自动推断：正常路径 `INFO`，终止信号 `WARNING`，控制/episode/recording/日志故障 `ERROR`。失败事件使用稳定的机器 reason（如 `CONTROL_STOP_FAILED`、`COMMAND_REJECTED`、`RECORDING_FAILED`、`EPISODE_FAILED`、`RUNTIME_FAILURE`），详细文本放 `message`/`fields`/`exception`；`critical` 仍独立表示证据不可丢弃。

## 日志故障与关闭

发送每条轨迹前，episode worker 先检查 `check_health()` 并写 `command.sending` 关键意图；检查或意图写入失败后不再发送下一条轨迹。main 主循环每轮检查健康，因此空闲、无新日志或 writer 异步失败也会触发 `request_shutdown`、拒绝新 episode 并以非零退出。回调内关键 `emit` 失败会记录 `LOGGING_ERROR` 并设置进程级日志故障标记，由主循环转入关闭；日志故障不通过同一个 logger 递归上报。

控制清理仍先于最终日志事件：即使 `control.acquired` 或 cleanup 的请求事件写入失败，已取得的 lease 仍会尝试 stop 和 release；两步分别调用、分别记录失败，原故障和后续日志/服务错误都保留。`EpisodeRunner`/`FailureLog` 仍是唯一错误汇总者。

关闭顺序为：停止新任务 → stop/release → 完成/失败 MCAP → `run.finalizing` → `logger.close()` 有界排空 → stdout 最终摘要。`close()` 失败把原本的 0 改成 1，并在摘要写 `logging_closed=false`、`process_outcome=failed`；SIGINT/SIGTERM 仍保留 130/143，并把日志错误写到 stderr。worker grace 由 `--shutdown-timeout`（默认 15 s）限制，日志关闭预算由 `logging.close_timeout_s`（默认 3 s）限制；两者是独立上限。超过 worker grace 的强制退出仍返回 124，不执行无界 flush/join，也不销毁 worker 仍使用的资源；SIGKILL 或强制退出不承诺完整落盘。

## 所有权、容量与故障

每进程一个日志 writer 拥有文件句柄，生产线程只做有界格式检查和非阻塞入队。使用标准库 logging/queue/threading 与小型显式封装；不修改全局 root logger，不建立跨进程共享文件锁或新的消息中间件。

| 情况 | 必须行为 |
| --- | --- |
| 队列正常 | 顺序写入，控制台保持简洁；控制回调不等待磁盘 |
| 队列满，非关键诊断 | 可丢弃，累计计数；恢复后和关闭摘要中报告丢弃量 |
| 队列满，关键事件 | 设置不可清除的失败状态，向调用方报错；runtime 必须发起停止，不默默继续 |
| 写盘/序列化异常 | writer 捕获并保留首个错误，通知健康检查；不得在线程中静默退出 |
| 文件轮转 | 当前 run 递增新文件；达到配置的总预算时失败停止，不删除旧关键事件来继续运行 |
| close | 停止接受新事件，有界排空并检查 writer 错误；超时/未写完报告失败，不能宣称成功 |
| 日志自己失败 | stderr 输出一次有界兜底诊断，禁止再递归调用同一个日志器 |

调用方至少在控制周期、阶段切换和完成前检查日志健康；异步故障不能只等到下次恰好有日志时才发现。日志模块报告错误，不直接调用 ROS stop 或终止训练，避免公共库反向依赖业务。

MCAP 使用独立有界队列和 writer，原始记录不允许静默丢帧。记录失败进入独立的 recording 错误；控制停止和释放仍各自尝试。队列容量既受条数也受已序列化载荷字节约束，正在写的条目仍占预算，防止大帧在慢盘期间突破内存预算。DSH-009 已把默认值写入配置并经测试覆盖；S2 多相机吞吐仍可在不改语义的前提下调整这些显式值。

关闭顺序为：禁止新任务 → 停止控制/释放 → 停止生产传感器记录 → 排空并完成 MCAP → 写最终事件 → 排空日志 → 发布进程退出结果。任何步骤失败都保留主故障与后续收尾错误。不得在回调仍可能写入时销毁 writer；强制退出只保留 partial，不承诺丢失的数据可恢复。

## MCAP 有界写入与流式审计（DSH-009）

每个 `EpisodeRecorder` 只有一个明确命名的后台 writer 线程，独占 `rosbag2_py.SequentialWriter` 的创建/open/create_topic/write/close；业务线程不跨线程关闭原生句柄，也不在状态锁内执行磁盘 I/O、join、回调或 logger 操作。`append` 在调用线程把消息序列化为不可变 `bytes`，再进入纯 Python 有界队列；序列化不持队列锁，也不等待磁盘或空位。这里的“非阻塞”只指不等待磁盘/容量，不表示序列化没有 CPU 成本。

队列同时按条数和已序列化字节限制，正在 `write` 但尚未返回的条目仍计入预算；单条超预算、队列满、输入 topic/时间/payload 非法、序列化或写入异常都会明确失败并永久 latch 为首要记录故障；之后 `check_health()` 和 `finish()` 保持失败，不丢帧返回成功，调用方 catch 后也不能再提交成功。只接受已注册 topic、非负整数时间戳和 bytes。异步 writer 故障保存为首要原因，`check_health()` 只读该 latch，不做文件 I/O。runtime 的定时路径（即使没有新帧）和每条命令发送前都会检查；同一故障只记录一次，随后进入既有停止/cleanup 链路。

`configs/runtime/mock_collect.yaml` 的 `recording` 对象显式列出默认值：队列 1024 条、待写字节 64 MiB、startup 5 s、close 5 s。对象或字段省略时补默认值，实际值写入 `resolved/runtime.json` 的 `recipe.recording`；正整数、有限正数和 bool 冒充数值都在启动前拒绝。新增 writer 初始化在 `startup_timeout_s` 内等待就绪，失败则 recorder 构造失败，控制不会开始。

`finish()` 先停止入队，在 `close_timeout_s` 内等待队列排空和 writer 自己关闭；超时/失败保留可识别的 `.partial-*` 并抛出记录错误，调用线程不会强关 writer。writer 线程不发布成功 manifest、不重命名 episode 目录。排空成功后才由 episode worker 执行流式 audit、分块 SHA-256 和 manifest 提交；close 预算只约束队列/原生 writer 关闭，之后的离线审计仍受现有主线程 shutdown grace 保护。迟到 writer 最多尝试关闭 bag，不能自行提交成功。

audit 一次处理一条消息，只保留 state/frame 时间与 ID、observation 引用、命令 trajectory 的紧凑语义和事件序列；关节运动范围在线更新 min/max，不保留 `Image`/`CameraFrame`/`RobotState` 或像素数组。引用顺序在末尾核验，因此不依赖 MCAP 到达顺序。参考元数据仍随 state/frame/observation/command 消息数增长，但不再与像素总量或整个 MCAP 文件大小成比例；不引入数据库。文件 SHA-256 以 1 MiB 块读取，writer 提交和 `inspect_episode` 复用同一函数。DSH-010 起新记录使用 v2 规则，v1 旧读取仍要求 mock 运动、完整 ACCEPTED/STARTED/COMPLETED 和旧 `complete`。

recorder 暴露只读 `RecordingStats`：accepted/written、当前与峰值条数/字节、失败原因和已写入 topic 计数（不可变 tuple）。manifest 的 `message_counts` 只取实际完成 write 的数量；正常关闭后 accepted 与 written 对齐、pending 归零。最终 `recording.*`/`episode.finished` 事件附带这些标量统计，不高频打印队列状态。

## 记录结果与验收

记录完整性、运行结果、任务成功是三个独立维度：合法停止可以保留完整记录，但未完成堆叠；控制流程正常返回也可能任务失败。DSH-010 已把新写 episode 统一为 v2：`recording.complete` 只由 writer 排空、实际写入计数、流式 audit 和分块 hash 决定；`runtime.outcome` 严格为 `completed/stopped/interrupted/failed`；`task.outcome` 严格为 `not_evaluated/succeeded/failed`，当前运行一律 `not_evaluated`。历史 v1 由 `schema_version` 分支按旧严格规则读取，未知版本和混入旧顶层 `complete`/`failure_reason` 的 v2 文件拒绝。`inspect_episode` 返回明确版本和三个维度；v1 仍保留 `complete=true` 的旧调用含义，并额外标出兼容维度。

只有 `recording.complete=true` 才把 `.partial-<id>` 发布为正式 `<id>`；runtime/task 结果不参与目录命名。因此出现正式 `episode-*` 只能说明原始记录完整，消费端必须继续检查 `runtime.outcome` 和 `task.outcome`，不能把目录存在、进程退出 0 或 `task=not_evaluated` 当作任务成功。writer close/超时/写失败、计数不一致或 audit/hash 失败保持 partial，记录维度为 incomplete；若磁盘本身不可用，可以只有 partial 而没有 manifest。

必须验证正常轮转/排空、多线程顺序、低级诊断计数、关键队列满、慢写盘、写失败、关闭超时、重复 close，以及真实 runtime SIGINT/SIGTERM 后的事件和 manifest。测试故障注入仅在 tests，不能留下生产环境变量开关。MCAP 审计按流校验引用，文件 hash 分块计算，不把整段图像或整个文件一次读入内存。

一个可追溯的失败报告至少能回答：哪个 run/episode、最后被接受的命令、停止是否发出及结果、记录是否完整、具体失败原因、产物路径。训练和推理额外关联数据集与模型摘要；不得靠多个控制台截图拼接执行历史。
