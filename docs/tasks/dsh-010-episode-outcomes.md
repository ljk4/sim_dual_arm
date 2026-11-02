# DSH-010：S1c 结果分类与人工结束语义

前置：DSH-009已验收。执行 [v2迁移设计](../episode-recording-format.md)，本任务完成数据格式分类及EndEpisode语义；Compose启动/状态命令和整个run监督仍留下一任务。当前仍使用mock，不放开物理/真机接入guard。

## 允许范围

- 新增 `contracts/schemas/v2/episode_manifest.schema.json`；可新增纯Python `contracts/python/src/robot_contracts/episodes.py`，放少量不可变结果类型与版本校验，不依赖ROS/ML。
- runtime修改 `recording.py`、`audit.py`、`episode.py`、`node.py`；`control_client.py`仅允许必要的普通执行停止检查，不能改变已成功获取lease的保存/清理或ACK语义。不要修改已验收队列和日志库。
- 新增unit/integration结果迁移与人工结束测试（不同文件名）；可迁移 `test_recording.py`、`test_recording_backpressure.py`、`test_episode_lifecycle.py`、`test_runtime_shutdown.py`、`test_runtime_logging.py` 和测试bootstrap中的v1专属断言/fixture。
- `tools/run/mock_collect.py`仅适配审计报告的新字段，暂不重写启动方式；文档限日志约定、quickstart、v2迁移设计与architecture-and-interfaces中的EndEpisode说明。

不改v1 schema含义，不改ROS/Policy wire类型，不加第三方依赖，不写顶层成功status，不改Docker/Compose/学习/物理，不提交Git/运行其他代理/申请Docker。首轮上限30分钟，一次定点返工最多10分钟。

## v2格式和兼容

1. 新写episode统一v2：顶层无 `complete`/`failure_reason`；使用 `recording`、`runtime`、`task` 三个明确对象，字段按设计。保留原计数/hash/配置/时间/身份，并包含run_id（直接测试可显式给测试ID）、source非空字符串、test_only布尔。生产runtime从当前BackendInfo传来源，不根据任务是否成功猜来源；直接mock fixture给出明确来源。
2. `runtime.outcome`严格枚举completed/stopped/interrupted/failed；task严格枚举not_evaluated/succeeded/failed，当前运行始终not_evaluated。类型以frozen dataclass和Literal/Enum等简洁方式表达，不能用任意dict和字符串前缀猜结束原因。
3. `recording.complete`只由writer排空、实际写入计数、流式audit与hash决定；控制失败/信号/人工停止本身不直接令文件损坏。记录失败必须使运行不为completed。写盘不可用可只有partial无manifest，不强行假装已完整分类。
4. 历史v1由版本分支按旧严格规则读取；未知版本拒绝。schema目录选择显式，不拼接不受限的版本路径。v2读取不能接受顶层旧complete别名。离线inspect返回明确版本和三个维度；v1保持原调用含义，文档说明差别。
5. 新产物目录按记录是否完整发布：recording.complete=true才从partial改名episode_id；runtime或task结果不参与目录命名。所有消费端/测试必须读维度，不能通过目录存在/退出0推导任务成功。旧v1记录不移动、不改写。

## 审计与原始记录

- v2 generic audit允许静止采集和零命令/零观测等短暂启动记录；仍验证实际存在的消息、时间、关节/图像形状、上下文、所有引用、轨迹全部语义、事件序号与实际计数。mock必须运动的断言移到mock演示测试；v1旧读取仍保持它。
- 原始命令可以是未被接受的意图或中止执行。v2的合法历史是ACCEPTED→STARTED→COMPLETED/CANCELED/FAILED及其前缀，也允许ACCEPTED后直接终止（启动前取消）。空历史作为未确认意图报告。拒绝倒序、重复terminal、terminal后又事件、孤立事件找不到命令、轨迹不一致和缺引用；报告 `commands_without_terminal` 与terminal数量，不把前缀当成完整执行ACK。
- 审计仍仅保留元数据、hash分块，不能恢复整段图像/文件读取。保留数据损坏、错误引用和frame_id反例。已验收队列拒绝/超时仍永久失败，迟到writer不能发布成功。
- finish仍只有一个finalizer，短锁认领；外部结束/信号在排空、audit、hash期间到来时更新runtime结果，不能提前写completed。队列close预算与后续audit的边界仍按DSH-009，不新增不受控后台finalizer。

## Episode与EndEpisode

- EpisodeRunner明确拥有正常完成、人工结束意图、信号中断和实际故障状态；不要把USER_STOP塞进FailureLog当故障，也不要通过搜索reason文本决定分类。使用已有shutdown标志/明确的结束意图及实际异常类型。真正的控制/记录/日志故障不得被同时到来的End请求掩盖。
- 观测/后端/事件等待被停止时返回明确受控中止原因，区别于真正超时。每段命令下发前检查停止；已有in-flight命令通过Stop/terminal反馈收尾，ACK仍不代表已经停止。
- EndEpisode接受请求后回ENDING；重复请求幂等，不覆盖第一次原因或再次finish。没有active episode明确拒绝。正常收尾已开始后的请求不能倒改已经完成的脚本结果，文档与服务reason保持一致。
- 人工结束后保持服务可用，下一episode重置结束意图、context、记录器和结果；上一episode原因不能串入下一次。SIGINT/SIGTERM继续130/143、worker超时124、真实故障`--once`非零，故障日志和清理路径不可退化。
- `RuntimeStatus.phase`继续表达生命周期（如IDLE/FAULT/ENDING），最终结果读manifest与结构化episode事件，不新增wire字段。人类可读/JSON摘要和事件明确三个维度，移除容易被误读的顶层complete成功声明。task当前一律not_evaluated。
- 首要故障与后续清理原因都保留；Stop/Release独立尝试。合法stopped/中断不要求动作正常COMPLETED，真实ACK事件按收到的阶段记录，不伪造成功terminal。

## 验收门槛

1. v2正常mock：recording完整、runtime completed、task not_evaluated；运动断言仍在mock演示测试。
2. 正常人工End：收到ENDING后最终recording完整、runtime stopped、task not_evaluated；未发送下一脚本段，Stop/Release实际完成；同一服务再start一次能正常完成且原因不串。
3. 信号中断（执行中与audit中）、双重信号、worker超时、cleanup失败、磁盘失败保持原语义。原R1～R3测试迁移只能改变v1字段/目录假设，必须改为更明确的v2维度断言，不能删除错误和退出码验证。
4. 真实文件支持静止/合法取消且generic audit通过；不完整引用、轨迹frame_id变化、损坏checksum、非法事件顺序仍失败。task not_evaluated不能被当作采集成功。
5. 真实v1 fixture可读；v1 complete=false仍按旧规则拒绝，即使其文件完整。未知版本/混入旧顶层别名拒绝。

监督者独立做旧v1读取、公开start/end/status连续运行、信号、失败与MCAP验证。不要仅靠更新旧测试得到全绿；在报告中列出每类迁移前后含义及对应新断言。运行纯Python回归和Ruff，真实ROS由监督者运行；报告 `.build/reports/dsh-010/report.md`。维护文档只记已经实现和实际检查，不能将S1全部完成或mock视为真实堆叠闭环。
