# DSH-008：S1c runtime 事件接线与日志故障停止

前置：DSH-007 已验收。依据 [日志约定](../logging-and-recording.md) 与 [闭环计划](../minimal-simulation-loop-plan.md)。本任务将可用日志库接到真实 runtime；不改 MCAP writer/manifest、Robot API、仿真、学习或 Compose 启动方式。

验收补充：关闭前事件最终采用 `run.finalizing`，带 `exit_code_before_log_close`；关闭后 stdout 才报告进程结果。新增 unit 文件改名为 `test_runtime_logging_config.py`，避免合并收集时与 integration 同名。初始任务要求与修正原因保留在 [结果记录](dsh-008-result.md)。

## 范围和代码组织

允许修改 runtime 的 `configuration.py`、`node.py`、`episode.py`、`control_client.py`，新增一个职责明确的 `run_logging.py` 用于进程日志初始化与元信息。允许修改 `contracts/schemas/v1/runtime_config.schema.json`、`configs/runtime/mock_collect.yaml`、runtime Dockerfile（仅安装新库）；新增 `tests/unit/test_runtime_logging.py`、`tests/integration/test_runtime_logging.py`，并按需扩展 `tests/integration/runtime_shutdown_bootstrap.py` 的测试侧故障注入。旧测试只允许迁移必要构造参数/配置快照期望，不能删断言或降低失败门槛。文档只更新 `docs/logging-and-recording.md`、`docs/stage2-quickstart.md`。

不要修改已验收公共日志库、recording.py、shutdown.py 或其他模块，不提交 Git、不运行其他代理、不申请 Docker。不要引入通用事件总线、装饰器、全局单例或自动拦截所有ROS日志。保持少量明确调用；日志字段直接可见，错误路径能逐行追踪。

## 配置、身份和元信息

1. runtime YAML 新增可选 `logging` 对象，完整字段对应已有 `LogSettings`。示例配置显式列出全部默认值；缺少对象/字段时使用默认值。`RuntimeRecipe` 持有 frozen `LogSettings`，校验 bool/整数/有限值及容量关系，错误包含配置路径。实际配置快照包含生效默认值（旧没有 logging 的配置也要补全）。保持原profile/observation/action语义哈希。
2. main 增加 `--run-id`（省略生成 UUID，校验目录安全）与 `--runs-output`（省略为 `--output` 的父目录下 `runs/`）。这是部署路径，不写进机器人语义配置。独占创建 `<runs-output>/<run-id>/`，拒绝覆盖；将 `run_id` 关联进各 episode 的配置快照和最终事件，不改变现有 RunContext wire schema。
3. 初始化时在控制开始前保存 `resolved/runtime.json`、`versions.json`；版本只收集 Python、明确依赖版本、实际源码导入路径与配置哈希等白名单，不倾倒环境/读取凭据，不凭空声称版本或 Git revision。镜像 digest/未知 revision 可为 null，由后续启动器补充。源码未提交时不能只记录 HEAD 并暗示源码一致。
4. 事件路径 `logs/runtime/<instance_id>/events.*.jsonl` 使用 EventLogger。进程拥有同一个实例。main 始终启用日志；独立业务类的测试可显式注入 None 或 fake sink，不允许生产配置静默禁用日志。日志不得放在 episode MCAP 目录冒充原始传感器数据。
5. 本任务不写顶层 `status.json=succeeded`：启动器尚未汇总各组件、退出码及必要产物。`summary.json=complete` 仅代表日志排空，`run.finished` 仅报告 runtime 进程结果。文档明确区别；全局状态留 S1d。

## 必须记录的可观察事件

- `run.started`、`run.stopping`、`run.finished`：身份、配置/输出路径、退出原因与退出码。异常保留类型、信息、堆栈；信号处理函数仍只记信号，不加文件/ROS调用。
- episode preparing/active/ending/finished：episode_id、任务、记录目录、主要与收尾原因。元信息与日志里的 run_id 能关联MCAP快照。
- acquire 成功/失败；每个命令发送意图、action ACK接受/拒绝、backend CommandEvent 的 ACCEPTED/STARTED/COMPLETED/FAILED/CANCELED（按收到的真实阶段记录，不能把 action ACK 当执行完成）。携带command_id、episode_id、clock_epoch和已有的仿真/robot时间；未知时间置null。保留 observation_id/command_seq 等少量标量，不复制关节数组/图像。
- 控制停止、释放的请求和响应/失败分别记录。request_id可为本地生成的日志关联ID，不改变服务wire协议，也不能暗示后端支持新幂等语义。正常续租无需每次 critical 日志，续租失败必须可查。
- MCAP完成/失败及路径；不改变当前manifest的旧语义，不提前标成物理任务成功。

## 故障与关闭

1. 每次控制发送前检查日志健康并写关键意图事件。失败后不能发送下一条轨迹。main 每个循环或独立低频timer检查健康，包括无新日志/空闲时；异步日志故障必须触发停止、拒绝新episode并以非零退出，不能等下一条命令才发现。
2. 错误汇总仍由 EpisodeRunner/FailureLog 所有。不要让日志故障再次进入同一个 emit 形成递归；关键日志异常不可被 catch 后静默继续任务。可以用小型显式 best-effort helper 处理收尾日志，但只能用于收尾，并且必须保留日志失败、导致非零结果。
3. **日志坏了仍必须尝试 stop 和 release**。发送前的收尾事件失败，不能使这两个实际服务调用被跳过；两步独立，保留原故障和后续错误。已经 acquire成功后，记录 acquired事件失败也不能丢失lease使cleanup失效。
4. 先收尾控制与MCAP，再最终事件，再有界close。日志close失败改变正常退出为1；SIGINT/SIGTERM保留130/143并记录日志错误。原 worker 超时路径仍124且不执行无界flush/join/销毁worker仍使用的资源。worker grace与日志close预算均有明确上限，文档说明两者；不承诺SIGKILL完整落盘。
5. 主线程构造、executor、episode或ROS teardown异常时同样尽力完成日志；不能只在正常 while 退出路径 close。close不能在线程仍可能写日志时提前成功。已有R1～R3退出/清理语义不得退化。
6. main输出简短的 run_id、日志/记录目录，人工可查；保留旧 episode JSON 摘要供既有工具解析。

## 验收

运行全量纯Python回归及Ruff。增加有意义的配置默认值/非法值/独占目录/元信息白名单测试；真实ROS集成由监督者运行。

集成测试至少覆盖正常一次的command关联、两个episode共享run且context不串；SIGTERM/SIGINT后的事件与退出码；运行中异步写失败或关键emit失败后无后续命令、stop和release仍尝试；只在close失败时退出1；故障后空闲进程退出；初始化失败不留下正常完成证据。故障注入只在tests，用EventLogger公开接口或writer边界替换，不加生产环境测试开关。

监督者独立跑真实双容器、注入故障、检查事件schema/MCAP/控制响应；不以测试全绿替代代码审查。输出实际完成及未验证项到 `.build/reports/dsh-008/report.md`。首轮上限25分钟，最多一次定点返工10分钟。
