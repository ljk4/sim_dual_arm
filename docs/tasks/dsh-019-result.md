# DSH-019：命令查询与执行反馈已验收

2026-11-01。对应[任务书](dsh-019-command-status.md)。20个允许文件已整合，主暂存区未变，未提交Git。S3仍未整体完成。

## 完成内容

- 新增 `CommandStatus` / `QueryCommand` 与独立 `command_feedback` 流，15个原有ROS消息/服务/action源码字节未变，历史MCAP继续可读。
- 查询按完整原context与command ID读取不可变快照；支持释放/交接后的历史结果，未知或逐出返回RESULT_EXPIRED，错代际/episode拒绝；不推进、续租或重执行命令。
- accepted/started/sample使用机器人时钟，elapsed使用显式execution_clock；当前mock为steady + linear_v1。事件记录发生时间，terminal冻结最后实际参考/执行时长。参考和测量分别保留。
- runtime关联已发送轨迹，拒绝错误参考、身份、时序及terminal漂移；记录有身份的ERROR事件并停止。已声明反馈能力时，原有有界收尾同时等待terminal事件和已验证、入队的terminal反馈。
- 实时与离线共用轨迹和状态转换校验，只保留每命令最后一个不可变快照。离线逐样本重建参考/进度；声明流后缺终止反馈不能成为完整记录。历史无该流的数据保持原语义。

人工阅读顺序：新增ROS两类型 → `contracts/python/src/robot_contracts/status.py` → runtime的 `ControlClient.query_command` / `RuntimeNode._on_feedback` → `audit.py`。物理执行尚未接入，mock测量仍等于其参考。

## 实施与接管

本地headless session `session-6a4dfb5c-ddca-46cc-8839-bfcd15c43f84`，隔离输入 `15cf4bd2393d285c224913dc479007a5165dbbb5`。12:11:03首轮启动，585.38s、exit0；12:21:22唯一返工启动，256.38s、exit0。

独立检查发现停钟时长错误、中间参考/进度/终止反馈审计遗漏、非home连续episode误拒和关键查询日志失败被吞；dsh返工修复。返工后真实回归仍发现旧v2诊断分类冲突与新增测试混用时钟的断言，由主助手接管。

主助手将runtime与audit的多字段字典收敛为已发送轨迹/最后快照，共用 `validate_status_transition`，补执行时钟改写、阶段倒退、terminal测量/原因漂移检查与拒绝日志。`_on_feedback`从90行降至46行（含拒绝日志），主审计从305行降至241行。v2缺terminal事件仍单独计数并导致runtime失败；另有有效terminal反馈时保留诊断记录，不将运行失败误当文件损坏。正式数据须额外满足成功/ACK门槛。

## 独立证据

| 检查 | 结果 |
| --- | --- |
| 整合后单测 / Ruff | 495项通过；89个Python文件格式与lint通过 |
| 冻结最终候选真实Jazzy/MCAP | 115项通过、无skip；包括原人工停止/缺terminal事件分类 |
| 实际mock CLI / 原生ROS查询 | 幂等、错context、停止、释放/交接后查询、冻结历史、后端退出后有界失败通过 |
| 查询响应与日志故障 | 错身份/顺序/NaN/拒绝响应、关键日志故障均拒绝 |
| MCAP独立反例 | 有效对照及9个错误参考/进度/终止/身份/NaN/时钟/阶段/快照/时间反例通过 |
| 实际DDS坏反馈注入 | 未知ID、错参考、改写执行时钟均触发FAULT/停止和带命令身份的ERROR事件；健康已接收记录可保留 |
| 中途End后再Start | 从保留的非home位置继续，停止/完成两条结果及记录均正确 |
| 公开Compose入口 | 最终单相机、三相机采集均exit0，日志排空/独立审计/清理通过；整合后默认入口另行复验 |
| 独立读取三相机原始MCAP | 3命令、51反馈、每命令1条terminal；图像51/51/49帧、51个有效因果观测，三路内容不同；逐反馈参考/进度和事件时间匹配 |
| 原始v1文件 | 6个原文件hash未变；完整记录可读，原失败/未知版本继续拒绝 |
| 源码 / SDK | 整合后的50个业务Python文件与冻结验收源码一致；默认SDK已由公开构建入口重建 |

首次完整ROS回归110通过/2失败，返工版113通过/2失败，均保留。早期SDK在末尾协议注释变动后被摘要保护拒绝；冻结后重新构建，未把此设置错误计为功能失败。最终SDK摘要 `b5a26e7935020a4d3d00a1633dc0f150eedb4ad7f03af7df9ec8cd1f32493e94`，19个契约源文件。

证据根 `var/runs/dsh/dsh-019/20261101T041055Z/`，含三轮源码归档、首轮/返工用量、accepted.patch；最终检查在 `acceptance/*-final`，公开三相机在 `public-three-final`，整合复验在 `main-public`。两轮dsh累计841.76s、131 steps、157 tool calls；输入123,162、输出124,577、cache-read19,512,960、total19,760,699 tokens。费用与主助手用量未知，不推算节省比例。

## 剩余边界

当前查询是显式读接口，execute超时仍失败，不自动重发或用查询结果覆盖丢失的action结果。尚无MuJoCo物理控制、reset/epoch运行实现、精确同快照三图/状态配对或Policy传输；三相机仍沿用已有因果对齐规则。正式数据数量为0，ACT未开始。接下来先验证16维SI与原生执行器/真实测量映射，再接物理控制核心及ROS。
