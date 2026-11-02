# DSH-007：S1c 公共结构化日志基础库

前置：DSH-006 已验收。本任务实现独立可测的日志文件输出；下一任务才接入 runtime 和运行状态，不把“基础库通过”写成“整个日志系统完成”。设计依据为 [日志约定](../logging-and-recording.md)。

## 范围

允许新增 `libraries/run_logging/pyproject.toml`、`libraries/run_logging/src/robot_run_logging/{__init__,events,writer}.py`（文件可更少，不创建额外层次）；新增 `contracts/schemas/v1/run_event.schema.json`、`tests/unit/test_run_logging.py`；只在根 `pyproject.toml` 增加该库的测试 pythonpath；只在 `docs/logging-and-recording.md` 补基础库 API 和实现边界。

不修改 runtime、MCAP、Docker、旧测试、其他协议或依赖版本；不增加第三方运行依赖，不提交 Git，不启动其他代理。必要阅读限上述设计、根 pyproject 和一个现有 libraries 包的打包声明。

## 对外 API 与配置

包 `robot_run_logging` 导出以下少量明确类型，不建立通用插件、事件总线或根 logger 框架：

- `LogSettings`：frozen dataclass。默认 queue_capacity=256、max_event_bytes=32768、max_file_bytes=8 MiB、max_total_bytes=256 MiB、close_timeout_s=3.0、flush_interval_s=0.2；可有明确的 console 开关。验证正数/有限值/严格整数及 event <= file <= total。默认值随后由运行配置加载，不在当前任务另写 YAML 加载器。
- `EventContext`：frozen dataclass，episode_id/request_id/command_id/clock_epoch/sim_time_ns，未知为 None。
- `EventLogger(directory: Path, *, run_id: str, component: str, settings: LogSettings, instance_id: str | None = None)`：独占一个新目录，拒绝覆盖已有目录；instance_id 默认 UUID。目录父级可创建，不自动读取环境变量或扫描代码/凭据。
- `emit(event: str, *, message: str = "", level: str = "INFO", reason: str | None = None, context: EventContext | None = None, fields: Mapping[str, Scalar] | None = None, exception: BaseException | None = None, critical: bool = True) -> bool`：非阻塞入队；非关键诊断因容量丢弃返回 False；关键失败抛 `LogFailure`。Scalar 仅为有限数字、bool、str、None，不接收图像、任意对象或嵌套大数组。
- `check_health() -> None`：异步写失败/关键丢失/关闭超时后持续抛 LogFailure。
- `close() -> None`：按配置有界排空；成功关闭可重复调用；失败状态不可因再次 close 而变成功。关闭后不能接受新事件。公开只读 directory/instance_id 便于定位。

可调整内部函数组织，但保持这些公开用途与调用方式。使用 queue/threading/json/logging 等标准库中合适的设施；不要为了使用 QueueListener 而依赖其私有线程或引入不可控的 join。采用显式 writer 线程也可以，所有文件 I/O 由该线程拥有。

## 格式与容量

1. 每个事件包含日志约定中的版本、身份、event_seq、wall_time_utc、elapsed_ns、level/event/reason/message、context 字段、critical、fields 和 exception。缺少的可选上下文用 null；schema 定义严格字段及类型，json.dumps 不允许 NaN。序号在并发生产者入队时按单一顺序分配；丢弃诊断的序号空隙由计数解释。
2. ID/事件名有明确长度和字符边界，不截断身份。限制 fields 数量/键/字符串长度；message/异常堆栈可有界截断，但必须有截断标记及原长。无效类型/字段在 emit 调用处明确报错，不迟到 writer 才发现。避免通过任意对象 repr/stringify 吞下不支持的数据。
3. 写 `events.000001.jsonl`，达到单文件大小前递增编号；不删除/覆盖本次运行的旧事件文件。总预算只针对事件文件字节，摘要预留少量空间；达到总预算进入失败，不绕过配额。任何单条都不能突破 max_event_bytes。
4. `summary.json` 在开始时为 running，正常排空并 flush/fsync 后才 complete；失败为 failed，包含首要原因、attempted/enqueued/written/dropped_noncritical 计数和文件清单。使用临时文件原子替换。写盘本身失败时允许保留 running/partial，但不能写 complete；不承诺断电或 SIGKILL 完整记录。

## 并发与失败传播

- 生产线程只做有界校验/格式化和非阻塞入队，不写文件/控制台。不要持队列状态锁等待磁盘、join 或用户回调。
- 普通诊断队列满允许丢弃并计数；critical 队列满设置不可清除的失败状态并抛 LogFailure。已入队的证据继续尽力排空，不重复提交已经接受的事件。
- writer 捕获写入、flush、fsync、轮转/摘要失败，保存首个错误；check_health/close 可观察到，不在线程中静默退出。
- close 停止接受新事件后等待有限时间。慢/卡住的 writer 不能令调用者无限阻塞；超时永久标为失败，不能由调用线程跨线程强关 writer 的句柄。测试必须释放自己制造的阻塞，避免遗留线程。
- 失败向 stderr 输出一次有界兜底诊断，不递归调用当前 logger；普通控制台输出由 writer 做且保持简洁。不改变全局 root logger。
- 日志基础库不调用 ROS stop、不退出进程；业务接线将在下一任务完成，不能在公共库反向导入 runtime 或 ML。

## 验收

测试真实文件和可观察结果：结构/schema与上下文、并发顺序、轮转保留全部事件、总预算失败、非关键丢弃计数、关键队列满、写/flush失败、关闭超时、重复关闭、关闭后写入、无效载荷与NaN、长诊断截断。使用测试侧 monkeypatch/event 控制慢盘与异常，不添加生产环境变量/测试专用延时参数。

生产代码可读性优先：少量用途明确的类型/函数，锁/文件所有权和失败状态有简短说明。测试不要只镜像类字段或在每个测试硬写同样实现。监督者会独立制造阻塞/写失败并读取产物，不以 dsh 自述替代验收。

运行全量纯 Python 回归、Ruff lint/format，输出放 `.build/reports/dsh-007/`；不申请 Docker。交付实际执行结果及限制。首轮上限20分钟，最多一次定点返工10分钟。
