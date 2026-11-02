# DSH-005 结果：控制客户端提取

2026-10-30。结论：S1b 的控制客户端子任务通过，已整合主工作区，未提交 Git。[任务书](dsh-005-control-client.md)。

`ControlClient` 独占控制 RPC/action、lease、续租和清理状态；node 通过公开方法调用，不重复持有 lease/future。故障回调在锁外执行，cleanup 独立尝试 renew/stop/release 后汇总，重复 cleanup 不再发 RPC。原11项清理场景迁移到新职责，新增8项控制行为检查；生产实现没有测试专用开关。

独立 Jazzy 容器运行 **120 项测试通过**（含86项纯 Python、MCAP、控制清理、控制客户端及10项进程中断测试）。最终候选源码通过真实双容器采集：退出0，3条命令、51条观测、52帧，MCAP审计通过。Ruff及整合后的86项纯 Python回归通过。没有改协议、超时或后端权限。

首轮通过，无返工；监督者仅澄清模块 docstring 中 executor 也会发起异步续租的表述。输入快照 `41f5b6fa5b88ae93452b15ec0d8bc629cef2de72`，本地证据 `var/runs/dsh/dsh-005/20261030T043054Z/`。dsh 375.401 s、58 steps、75次工具调用；输入70,069、输出65,511、cache-read4,908,416、total5,043,996 tokens，为provider累计报告。主助手用量和实际费用未知，不能据此声称节省比例。

node 仍保留 episode 编排、观测和记录；本任务只完成 R4 的一个责任提取。下一子任务为 episode 生命周期提取，之后才实施日志与写盘容量整改。主工作区原有修改保留，暂存区未改变。
