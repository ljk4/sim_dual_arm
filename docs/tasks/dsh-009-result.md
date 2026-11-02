# DSH-009 结果：MCAP 队列与流式审计

2026-10-30。结论：R5 已修复并整合，未提交 Git。[任务书](dsh-009-recording-queue.md)。

每个 recorder 由一个后台线程独占原生 MCAP writer，生产线程序列化后只入有界队列。队列同时限制条数和字节，在写的大帧仍计入预算；拒绝、写入失败和关闭超时持续报错。runtime 在定时路径及下一命令前检查健康，收尾仍独立停止和释放。审计按条读取，只保留引用元数据，哈希按1MiB分块；旧manifest语义暂时保留。

首版219项回归通过，独立探针仍发现三项问题：容量拒绝未持久化故障、轨迹元数据漏掉frame_id、重复finish等待另一个finalizer的audit锁。一次dsh定点返工修复；主助手没有接管生产代码。

最终 **227项合并回归通过**（164 unit + 63 integration）、11项独立探针通过、Ruff通过；整合后164项unit通过。真实MCAP验证了writer线程所有权和append不等待磁盘；真实ROS在native write阻塞3秒期间仍完成三段命令；写失败只发送一条命令后停止，close超时保留partial，两种失败均实际收到stop/release成功响应。轨迹frame_id不一致重新被拒绝，重复finish在audit阻塞时立即拒绝。

同一份192帧320×240 RGB记录（像素总量44,236,800 bytes），`tracemalloc`测得审计Python分配峰值由48,536,309降至2,154,825 bytes，约46.3MiB降至2.1MiB。这不是整个进程RSS或GPU显存测量；引用元数据仍随消息数增长。close预算只覆盖队列和原生writer关闭，audit/hash在episode worker中，信号收尾受主线程grace保护。

证据：`var/runs/dsh/dsh-009/20261030T060052Z/`；快照 `5223fafd83a2d668b197f43718ee9dc239548595`。dsh两轮749.686s、110 steps、123次工具调用；provider报告输入109,329、输出108,993、cache-read15,294,720、total15,513,042 tokens。主助手费用/用量未知。主暂存区保持不变。

下一步迁移记录完整性、运行结果与任务标签，保留旧v1读取含义；Compose开发入口和总体运行监督仍待S1d。正式数据、物理场景与ACT尚未开始。
