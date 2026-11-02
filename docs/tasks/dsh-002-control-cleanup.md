# DSH-002：控制清理独立尝试（R3）

实现阶段 2R 的 R3。监督者会提供独立 worktree、快照基线、工具路径和报告目录；HEAD 应与快照一致且初始工作区干净。只实现本任务，完成后停止，不提交 Git，不启动其他代理，不扩大权限。

允许修改：`components/runtime/ros/dual_arm_runtime/dual_arm_runtime/node.py` 和新增 `tests/integration/test_runtime_cleanup.py`。先读 `_cleanup_control`、`_wait_future`、`_call`、`_fail` 和已有集成测试风格，不读完整历史文档。

问题：等待在途续租、stop、release 共用一个 try，前面失败会跳过后续清理。

要求：

1. 锁内关闭续租并取出 pending future。有效 context/lease 存在时，每个适用步骤独立尝试，续租等待失败仍尝试 stop，stop 失败仍尝试 release。
2. 续租 future 返回 success=false 也算失败。每一步保留有限超时；清理不受普通停止标志中断。
3. 聚合带步骤名的错误并记录。已有 episode 主失败原因必须保留，附加清理失败也必须进入最终 failure_reason，不能只打印后丢失。没有旧失败时，清理失败使 episode 失败。
4. 所有路径清空本地 lease 和 pending future，重复清理无额外服务调用；无 lease/context 时也完成本地清理。
5. 保持本轮改动局部、明确；不新增通用重试框架，不改服务/消息/schema。实现后续 R1/R4 会复用的稳定行为。

新增有意义的集成测试：正常清理、续租等待异常、续租拒绝、stop 失败、release 失败、多处同时失败且已有主失败、无控制权、二次清理。可对 RuntimeNode 方法使用轻量 receiver 和可注入 callable，但不要伪造整套 ROS import；ROS 类型由实际测试镜像提供。

宿主机没有 ROS Python，因此先写测试，明确报告 ROS 测试未执行；监督者会在现有 Jazzy 测试容器中挂载候选源码独立运行，反馈失败后再定点返工。你运行宿主机纯 Python 测试、Ruff 和 git diff --check。不要为运行 ROS 测试自行安装依赖、访问 Docker 或修改测试发现规则。

交付：改动摘要、改动文件、实际执行的检查、待监督者执行的 ROS 测试、任何阻碍。限制首轮 15 分钟；日志只写指定的 .build 报告目录。
