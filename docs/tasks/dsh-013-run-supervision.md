# DSH-013：S1d 自动采集与整次运行监督

前置：DSH-012 已独立验收并整合。此任务通过后才可判定 S1 工程基础完成。继续使用现有 CPU mock，不接入 MuJoCo/训练/推理。

## 目标和允许范围

将旧自动演示统一到 Compose 开发栈。维护者运行一次 collect 后，可从一个 status.json 和关联证据回答：进程如何退出、控制/记录是否完成、关键日志是否排空、MCAP 独立审计是否通过、失败发生在哪一步。succeeded 表示这次 mock 控制采集操作通过，不表示任务成功或正式训练数据合格。

允许修改/新增：

- `tools/run/mock_collect.py`、`tools/run/mock_collect.sh`：原入口保留，改为 Compose 的薄调用；可增加一个明确命名的 `tools/run/collect_run.py` 放监督职责，避免再产生千行入口。优先复用 dev_compose 和已验收的容器身份检查，必要时把相关小型工具提取到 `tools/run/mock_stack.py`，人工入口只做相应导入调整，保持原行为。
- 如配置读取或证据文件处理确实需要独立职责，可新增 `tools/run/collect_config.py`、`tools/run/collect_artifacts.py`；主流程应能直接读出准备、启动、监督、审计、收尾的顺序，不引入继承/插件/通用任务引擎。
- `deploy/compose/mock.yaml` 的最小配置路径参数化；可新增 `deploy/compose/mock_collect.yaml`，只覆写 runtime 命令增加 --once，不复制环境、网络、镜像和挂载。所有主业务服务由同一 Compose 组合管理。
- `configs/deploy/mock_collect.json`：主机标准库可读的启动器配方；`contracts/schemas/v1/run_status.schema.json`：整次运行状态格式。
- 新增 `tests/unit/test_collect_run.py`；必要的故障注入夹具仅放 `tests/integration/collect_*.py`。
- 文档：README、deploy/README、stage2-quickstart、logging-and-recording、maintainability-review 中对应当前状态和操作段。不重写历史证据。

不改 runtime/backend/记录/公共日志库业务，不改 Robot/Policy wire，不增加宿主 pip 依赖，不运行 Docker、不调用其他代理、不提交 Git。首轮上限 25 分钟，最多一次 10 分钟定点返工。报告 `.build/reports/dsh-013/report.md`。实际 Docker、ROS、磁盘/进程故障由监督者验收。

## 配置与使用

- 保留 `bash tools/run/mock_collect.sh` 和 `python3 tools/run/mock_collect.py`；默认产生新 run id/隔离项目，支持显式 project/image/build-root/raw-root/runs-root/run-id。源码和输出都挂载，不用匿名卷/docker cp。
- 运行期限等参数来自 JSON 配置，字段带单位；包括整次工作墙钟期限（建议 60 s）、单命令上限、轮询间隔、独立 audit 上限、停止 grace、总 cleanup 上限、诊断文件字节上限、launcher 日志预算；拒绝未知字段、bool 冒充数字、NaN/Inf、非正值和不合理预算。有效值和配置哈希持久保存。配置覆盖是显式路径，不引入多层配置框架。
- 整次工作期限包含启动、episode 和独立 audit；失败/信号后的 cleanup 另有明确上限，日志 close 还有自己的明确预算。任何重试不重置工作期限。文档写清最坏等待边界；不能只给 docker wait 一个期限，随后做无界 audit/cleanup。
- 旧 `--output` 若保留，明确映射到原始数据目录并拒绝已有目录，日志仍单独保存；若个别旧选项不再支持，应给出可执行迁移说明，不能默默忽略。`--probe-backend` 可以在启动 --once runtime 前，用现有公开 probe 先验证 backend，仍在整体期限内；不得与正式动作争抢 lease。

## 运行身份、目录与证据

1. 启动前确认 project 不存在、run/raw 产品未被占用。不得替换其他项目，不操作 Docker socket 之外的外部服务，不读取认证或倾倒环境。
2. runtime 仍独占创建 `<runs-root>/<run-id>`。监督者不能预建它或放宽 RunSession 的排他性。可先在 `<runs-root>/.launch/<run-id>/` 独占保存启动证据；观察到 runtime 创建其 run 目录后，再把监督者自己的文件移入其中，不覆盖 runtime 的 resolved/versions/logs。
3. 若 runtime 在目录创建前失败，只有已识别本次 runtime 容器且确认其进程已停止后才可由监督者创建最终 run 目录保存失败结果；若尚未识别 runtime、Docker 不可达、不能排除迟到创建，则保留启动证据目录并报告其路径，不能仅因一次 ps 为空就和 runtime 抢建或覆盖。
4. `status.json` 初始 running，随后原子写 succeeded/failed/interrupted；包含 schema_version、run_id、阶段、稳定 reason、起止 UTC、monotonic 耗时、组件容器 ID/实际退出码、recording/runtime/task 维度、audit 结果/证据路径、关键日志排空、cleanup 结果和附加错误。未知退出码用 null，不用 0 补齐。
5. launcher 单独保存元数据（例如 launcher.json），不覆写 runtime versions.json：实际镜像不可变 ID、生成契约摘要、有效配置及哈希、源码来源路径/摘要、Git HEAD 仅作参考。工作区未提交代码不能被说成等同于 HEAD；来源变化可检测时应报告，未知值明确 null/unknown。只摘要明确的项目源码/config/profile/部署文件，排除 var、.build、.git、deploy/local、凭据和缓存。

## 监督与成功门槛

- Compose 启动 --once runtime 和 backend，随后轮询真实容器状态，保留容器直到已读取实际退出码和诊断。backend 提前退出、runtime 非零、整体超时、SIGINT/SIGTERM 都进入失败/中断和有界收尾。
- 正常成功要求 runtime 实际退出 0；backend 在 runtime 和 audit 完成后由监督者正常停止，记录实际退出码并按该后端的正常值检查。任何 137/未知强杀不能当作正常关闭。停止结果和自然退出结果分别记录，不只看 docker compose down 返回码。
- 检查本次唯一 runtime 事件日志 summary 完整排空，计数自洽、身份匹配；launcher 关键日志也必须成功 close。目录存在、旧 summary 或 run.finalizing 不是退出证据。
- 在独立、有界的工具容器中调用现有 `dual_arm_runtime.audit`，挂载候选源码/SDK及只读 raw 数据，防止旧镜像代码遮蔽。不要复制 audit 或 JSON schema 业务。工具可使用 Compose run --no-deps 覆写 runtime 命令，直接复用准备好的环境，不能误触“fresh run”检查；超时后清理本次工具容器。
- audit 必须与本 run/episode 对应，v2/recording.complete/runtime.completed/source=mock/test_only=true，观测/帧非空，三段命令都有真实 terminal，commands_without_terminal=0，mock 运动大于既定阈值。task 原样 not_evaluated，不能将其写成 succeeded。
- 保存 audit JSON。保留 partial、失败 manifest、控制日志、退出码与失败原因；不自动删失败数据或重试成一条“成功”覆盖证据。
- 成功状态只能在真实退出、audit、关键日志关闭及必要 cleanup 检查均完成后发布。主故障保留，后续停止/日志/清理错误追加，不覆盖为最后一个不相关错误。

## 日志、信号和清理

复用公共 `robot_run_logging`，host 可通过明确源码路径导入这个仅标准库的包；不要新写通用 logger 或改全局 root logger。component=launcher，事件包括启动、容器创建/退出、超时、audit、停止、收尾；高频轮询不逐次写关键日志。fields 仍为标量。日志写失败/关键队列满/close 失败须影响结果，不先写成功再忽略 close。

保存本次 Docker/子进程诊断，不在内存无界收集 stdout/stderr。可以有明确字节上限或标明范围的诊断 tail；截断必须记录，不能宣称它是完整关键事件流。Docker 诊断与结构化关键事件的作用不同，MCAP 里不重复存日志。任何命令/诊断收集都有期限；错误路径保存证据失败也要返回非零，不能在 stderr 报错后仍 success。

信号处理器只设置意图；主循环停止新工作，先让 runtime 有机会完成 Stop/Release/MCAP/日志关闭，再停 backend。grace 到期才强杀确认属于本次的容器，实际记录 137/超时。正常工作 deadline 到期不会无限延长，cleanup 不重置预算。audit 工具和子进程同样归本次监督，不能留下后台进程。只清理本次明确创建的资源，保留挂载数据。进程退出码建议正常 0、失败 1、配置错误 2、中断 130/143、工作期限超时 124，具体保持文档一致。

## 自测与独立验收

纯 Python 行为测试：成功门槛逐项缺失均不能成功、非零/未知退出码、进程提前退出、总期限不被轮询/重试重置、信号优先级、有界 cleanup/日志 close、startup 失败目录归属、已占用项目/run 拒绝、坏 manifest/audit/关键日志摘要、配置数值与未知字段、原子状态写入失败。测试调用边界和结果，不做源码字符串快照。

监督者用真实 Compose 检查正常采集/独立审计/宿主产物、backend 退出、runtime 崩溃、整体超时/信号、实际篡改 MCAP 后 audit 失败、日志写入/关闭失败，以及已有项目/run 不受影响。故障注入只在测试 bootstrap 或公开边界替换，不留生产环境故障开关。必要回归通过后记录并整合，不能开始 S2。
