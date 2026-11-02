# DSH-013 结果：自动采集与整次运行监督

2026-10-30。已验收并整合 19 个文件，未提交 Git。[任务书](dsh-013-run-supervision.md)。S1 工程基础至此完成，当前结果仍是 CPU mock 链路，不是物理堆叠或 ACT 结果。

原 `mock_collect.sh/.py` 已统一调用 Compose 开发栈；源码、生成契约、原始数据和日志使用挂载。JSON 启动器配方集中管理工作期限、停止/清理预算、诊断容量和日志容量。一次采集通过真实进程退出码、独立 MCAP audit、关键日志排空和清理结果共同判定，最终原子写 status.json。未启动、未取得的退出码保持未知；启动证据不会抢建 runtime 的独占 run 目录。

原始 episode v2 manifest 不改写。整次状态里的 `recording.manifest_complete` 保留写入器原声明，`recording.complete` 只在本次独立 audit 也通过时为 true；未验证与校验失败还须结合 audit/errors 区分。runtime 和 task 保留各自结果，审计或日志故障不能倒改已完成的 runtime，mock 的 task 始终 not_evaluated。

配置、纯证据读写/校验、Docker I/O 分开；生命周期、信号和最终结果由 Collector 明确拥有，撤回了跨模块传入隐式 context、再回调私有方法的机械拆分。监督类仍较长，后续按实际责任扩展，避免为了行数引入隐蔽依赖。底层继续复用已有 Compose 准备、容器身份校验、公共日志库和原生 audit。

首版 290 项 unit 自测通过；独立验收发现日志非空截断仍假成功、按项目批量清理外来测试容器，以及审计失败倒改 runtime 结果。一次 dsh 返工补齐这些问题、有界流式诊断、共享停止预算、只读 audit、不可变配置和状态归属；返工在 600 s 上限被监督停止，没有完整最终报告，不记为 dsh 自行交付完成。

主助手接管后修复并复验：

- Docker 统一使用完整 ID，删除失败时不再把自己的剩余容器误认为外来资源。
- 拒绝旧 run 后不再进入旧目录收尾；旧 status/launcher 和其他文件逐字节保留。
- 配置/证据按实际字节限额读取并拒绝 FIFO 等非常规文件，避免 stat 后无界读取。
- 明确原始完整性声明与本次验证的区别，并将实际运行的 mock 后端源码纳入版本摘要。
- 统一验收捕获旧 mock 后端 SIGTERM 与发布回调的竞态。只在该 node 的 ROS context 已关闭时处理原生 RCLError，健康 context 的同类错误及其他异常继续失败。新增原生用例在修正前 1 失败、2 通过，修正后全通过；没有放宽监督成功条件。

验收证据：

- **399 项合并回归通过**（303 unit、96 integration）；最后补齐版本摘要范围后 **304 项 unit**、Ruff、shell 语法及实际正向采集通过。
- 固定候选的 **13 项真实 Compose 场景全部通过**：正常、旧 probe 选项、SIGINT、SIGTERM、runtime/backend 丢失、总期限、实际 MCAP 篡改、日志 fsync 失败、终态状态替换失败、非空事件截断、外来容器/网络端点保护、删除拒绝。正常 runtime/backend 实测退出 0；独立工具容器的源码、SDK 和数据挂载均只读。
- 人工入口复验通过；实际已有项目被新 collect 拒绝，原容器继续健康。旧 run 保留、真实状态 schema、FIFO 和超限读取、输出容量均有独立证据。各输出流实际产生 2 MiB 时只保留 4096 bytes，父 Python 分配峰值 154,579 bytes。
- 验收中曾有独立观察器将 Docker 小写的“no such object”当作异常，修正观察器后重跑；早期单命令期限短于工作期限的探针也已校正。旧输出保留，不计为候选通过。

证据目录 `var/runs/dsh/dsh-013/20261030T103438Z/`；主要结果在 `acceptance/ros-accepted-final`、`suite-accepted-final`、`provenance-final`、`reuse-final`、`existing-project-final`、`output-cap-final`。`rework-source-before-primary.tar.gz` 保留接管前源码，`accepted.patch` 为整合差异。输入快照 `ec673b8ded6d8e7c5ad6f365c968eeeb9227e82a`。默认开发镜像和 `.build/ros-dev` 已实际构建，并核对源码/SDK 导入来源。

dsh 两轮合计 1527.324 s、217 个已回报 steps、238 次工具调用；可汇总用量为输入 169,410、输出 217,262、cache-read 50,488,448、total 50,875,120 tokens。超时取消的最后一步可能缺少用量，主助手用量/费用未知，不能据此计算节省比例。主暂存区保持不变。

下一阶段 S2 先验证依赖、Panda 资产与本机 EGL，再搭建双臂/柱/桌/方块及三相机场景。正式 50 条数据、训练和推理尚未开始。
