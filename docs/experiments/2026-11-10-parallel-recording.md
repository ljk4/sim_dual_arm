# 并行示教录制验收（2026-11-10）

新增宿主机入口 `runtime/ros2/scripts/record_parallel.sh`，由一个 runtime 调度器控制多个独立的 simulation 容器。各路使用独立 ROS domain、命名空间、配置和 backend 日志，共享原 campaign 的预算与种子分配。

并行模式禁用 viewer 状态发布及导出预览 PNG/GIF，保留相机 EGL 渲染、训练 RGB PNG、标签、文件哈希和质量门禁。物理步长、录制频率及任务成功判定沿用原配置。

## 本机吞吐实测

机器：16 个逻辑 CPU、约 15 GiB 内存、NVIDIA RTX 4050 Laptop GPU（6 GiB）。所有测试使用独立诊断台账；原手动 backend 在测试期间仍然运行。每路 `realtime_speed=1.0`，BLAS/OMP 线程数为 1。

| 并发路数 | 合格条数 / 尝试数 | 录制调度耗时 | 含实例启动与清理总耗时 | 折合总耗时 / 合格条 |
| --- | --- | --- | --- | --- |
| 1 | 2 / 2 | 78.150 s | 83.620 s | 41.810 s |
| 2 | 2 / 2 | 40.469 s | 45.598 s | 22.799 s |
| 4 | 4 / 4 | 46.649 s | 52.524 s | 13.131 s |

本次样本中，4 路按总耗时折算的吞吐约为单路的 3.2 倍，所有 episode 均为任务成功、录制完整、质量合格。自动档按 CPU、可用内存与显存估计，最多选择 4 路；本机当前资源条件下选择 4 路。上述数值是本机小样本验收结果。

原始汇总：

- 单路：`var/record-pools/pool-1794302799321639482/summary.json`
- 两路：`var/record-pools/pool-1794302456594325761/summary.json`
- 四路：`var/record-pools/pool-1794302602444279216/summary.json`

## 调度与停止规则

- `--count N` 统计本批次新增合格 episode，在途数不超过剩余目标，并计入全局合格目标的剩余名额。
- 每局开始前在原台账中预留种子及预算；分配由同一调度器串行完成，后台录制同时运行。
- 连续失败按完成顺序计数，合格一次清零。达到 `--max-consecutive-failures` 后取消其余在途任务并记为 `INTERRUPTED`；取消项仍占用预算。
- 预算不足时完成已预留任务；中断时停止本批次进程组并清理本批次创建的 simulation 容器。
- 逐局控制台输出保存在 attempt 目录的 `console.log`，批次配置和汇总保存在 `var/record-pools/pool-*/`。

实测在两路均进入 `RUNNING` 后向宿主机入口发送 `SIGTERM`：入口返回 130，两次预留均追加 `INTERRUPTED`，运行锁释放，本批次容器关闭，原 campaign 台账保持不变。对应诊断目录为 `var/diagnostics/parallel-record-interrupt-1794302948976592743/`。
