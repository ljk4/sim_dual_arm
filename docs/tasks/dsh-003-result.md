# DSH-003：中断收尾修复结果

日期：2026-10-30。结论：R1 经一次 dsh 定点返工、监督者局部补齐和独立验收后整合到主工作区，未提交。沿用 [任务书](dsh-003-runtime-shutdown.md) 的范围，未推进 RPC、Gazebo 或职责重构。

## 最终行为

信号处理器只记下首个 SIGINT/SIGTERM。主线程停止接受新 episode，通知 worker 停止，继续处理 ROS 通信；worker 停止动作、释放控制权、关闭记录后，主线程才销毁 ROS 资源。普通完成、运行失败、SIGINT、SIGTERM、收尾超时分别返回 0、1、130、143、124。

`--shutdown-timeout` 默认 15 秒。超时强制路径保留 partial 目录，不在活跃 worker 上销毁 ROS 对象，不保证 manifest 已生成。状态发布和记录关闭失败会保留原始故障；失效的 executor 不再反复调用，worker 在剩余期限内尝试完成本地收尾。

Docker runtime 默认直接运行 Python 模块，作为 PID 1 接收信号。test target 包含 mock 测试替身；测试注入放在 tests 内，正式 runtime 不提供测试延迟环境开关。

## 修改位置

| 文件 | 用途 |
| --- | --- |
| `components/runtime/ros/dual_arm_runtime/dual_arm_runtime/node.py` | 停止接单、worker 完成通知、有界退出、保留附加故障 |
| 同包 `shutdown.py` | 信号捕获、退出码和正数期限解析 |
| 同包 `recording.py` | 收尾期间复检中断与故障，写入失败 manifest |
| `deploy/docker/runtime/Dockerfile` | PID 1 入口与测试替身打包 |
| `tests/unit/test_shutdown.py` | 无 ROS 的信号策略和参数边界检查 |
| `tests/integration/test_runtime_shutdown.py` | 真实 ROS 子进程的正常完成、中断、异常与超时验收 |
| `tests/integration/runtime_shutdown_bootstrap.py` | 仅用于测试的 writer/executor/状态故障注入 |

## 独立验收与返工

1. 首轮候选通过 78 项检查，独立真实 ROS 探针也确认 SIGINT/SIGTERM 后动作取消、退出 130/143、失败 manifest 保留原因。
2. 同一探针放回修复前实现，复现 runtime 退出 0、manifest 数量为 0。探针断言失败，证明覆盖了原缺陷。
3. 测试通过后，审查仍发现正式代码的测试开关、记录异常覆盖主故障、IDLE 异常返回 0、测试替身打包缺口及 `ros2 run` 不转发 Docker SIGTERM，反馈 dsh 一次定点返工。
4. 监督者随后补齐 `ExternalShutdownException` 的处理：不反复 spin 失效 executor，收尾期间的新增错误进入 manifest；同时补上回归测试及意外主线程退出的有限等待。该部分明确计为监督者修改。
5. 最终候选挂载检查 **75 项通过**；重建 test 镜像后，不挂载源码、不额外设置 PYTHONPATH，默认命令仍 **75 项通过**。数量减少来自移除正式代码的测试延迟钩子及其解析测试，并增加故障验收。
6. 实际向默认 runtime 容器的 PID 1 发送 SIGTERM：约 **0.419 秒**后退出 143，后端事件为 ACCEPTED/STARTED/CANCELED，manifest 为 `complete=false`、原因为 SIGTERM。
7. 整合后原有启动器通过 **7 项后端 API 检查**，完成 **3 条命令、51 个观测**的正常采集，并通过独立 MCAP 审计。Ruff check/format 和 Git 差异检查通过。

75 项已经包含此前的纯 Python、MCAP 和 R3 测试，不与首轮 78 项相加。构建后的本地 `runtime:stage2`、`runtime-test:stage2` 标签已更新。本次临时容器与网络已清理，记录保留在任务运行目录。

## 执行记录与评价

| 指标 | 结果 |
| --- | --- |
| 输入快照 | `3e993584e3a6d70a7ebd0628f2734fc42b298a0e`，含已验收 R2/R3 |
| 隔离工作区 | `/home/ljk/work/sim_dual_arm-worktrees/dsh-003` |
| 模型 | `deepseek-account / deepseek-flash / high` |
| dsh 首轮 / 返工 | 419.973 / 242.019 秒，总计 661.992 秒 |
| 从输入快照到整合验收 | 1,322.383 秒，约 22 分钟，包含 dsh 执行、等待、审查和验证 |
| dsh 步骤 / 工具调用 | 111 / 130，包含返工 |
| provider 累计用量 | input 68,732；output 101,323；cache read 11,178,624；total 11,348,679 |
| 主助手精确用量、费用、主动审查时间 | 未取得，不能记为 0 |

用量是各 step_end 的累计值，包含跨步骤反复读取的缓存；不能把 total 当作一次请求长度或直接当作费用。任务起止、审查跨度、构建日志、原始事件、反馈及接受补丁位于本地 `var/runs/dsh/dsh-003/20261030T033715Z/`。审查跨度包含返工等待和测试，不能重复相加为总耗时。

这次证明了复杂修复可以按任务书委派，但首轮测试通过仍不足以免除人工审查。一次返工后还有监督者修改，不能据此宣称已经节省总用量或加快全流程。后续继续使用较小的任务和新 session，按一个职责一次提取。

## 保留的边界

本次覆盖正常可写存储上的中断、可控 writer 延迟和明确异常。主循环必须能够处理停止请求；同步写盘阻塞 ROS 回调、大记录审计内存、SIGKILL/断电恢复仍不在本次保证内，写盘问题继续由 R5 跟进。已有完整记录与进程退出状态应分别读取；通用记录完整性与任务结果的分类仍属 R6。

R1～R3 已完成，R4 职责提取、R7 配置和 R8 人工操作路径仍待实施，尚未恢复阶段 3。
