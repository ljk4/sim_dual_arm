# DSH-012 结果：人工操作命令

2026-10-30。已验收并整合 10 个文件，未提交 Git。[任务书](dsh-012-manual-cli.md)。人工 `up/status/start/end/down` 可在无宿主机 ROS 的条件下操作 Compose 开发栈；整次运行监督仍属于 DSH-013。

主机入口核对实际项目、容器、run id、镜像及挂载归属；初始 IDLE 和服务发现通过后才报告就绪，返回状态前复查容器存活。ROS 客户端把 PREPARING/ENDING 的服务 ACK 与等待最终 manifest 分开，按本次 episode id 等待，结果保留 recording/runtime/task 三个维度。它不替代 MCAP 审计，输出 `audit_verified=false`。当前 EndEpisode wire 没有 episode id，人工 start/end 要串行，客户端检查与服务处理之间仍不能提供原子目标选择。

首版 224 项 unit 自测通过，但独立实际 up 发现客户端成功路径缺失退出码赋值，JSON 宣称成功后进程仍退出 1；另用替身复现 Compose down 非零被掩盖。新增 ROS 夹具给客户端和服务设置了不同 domain，导致发现超时；该轮未跑完，不计通过。一次 dsh 定点返工修复上述问题，并补齐 FAULT 不可升格、按字节有界读取/非常规文件拒绝、复用契约结果类型和部分启动失败清理。

主助手另修正一处旧测试夹具：4 个生产者共突发写 100 条，原顺序测试只给 64 条队列且假定 writer 会及时被调度。改为容纳完整有限突发，保持生产队列与独立溢出测试不变。该修正之后 227 项 unit 通过；Ruff lint/format 通过。一次复验因只读 worktree 无法写 Ruff 缓存退出，改用 `--no-cache` 后通过，不计为候选功能故障。

独立验收结果：

- **320 项合并回归通过**（227 unit、93 integration），包含实际 RuntimeNode、公开服务、进程入口、人工结束、第二次 episode、超时和 manifest 边界。
- 公开主机命令正常启动、状态、start --wait、inactive end 拒绝、down 幂等、run id 复用拒绝通过；记录和日志保存在宿主挂载目录，正常停止后关键日志排空。
- 不同 run 的 status/down、同项目另一个 run 的 up、活动期间重复 start、错误 episode 的 end 均被拒绝，原栈继续可用。
- 人工 End ACK 后等待正确的 stopped manifest，再完成下一条独立 episode；独立 MCAP audit 检查实际引用、hash、ACK/terminal、帧和运动通过。
- 实际杀死 runtime 后，status 非零并给出退出码 137；非法配置启动失败后不留运行容器。验收项目已清理，临时加长轨迹和非法配置均逐字节还原。

证据：`var/runs/dsh/dsh-012/20261030T100038Z/`；最终检查在 `acceptance/ros-rework`、`public-basic-rework`、`public-safety-final`、`public-failure-final`。隔离输入快照 `e3c7b9d4c3b08498a27a13e9d974eced17c19c9a`，复用 `sim-dual-arm/dev-ros:dsh-011` 和已验收的契约构建输出，未把旧镜像内的应用代码当作候选。

dsh 两轮 915.430 s、104 steps、138 次工具调用；输入 129,034、输出 145,731、cache-read 19,143,936、total 19,418,701 tokens。主助手用量/费用未知，不能由这些数字计算节省比例。主暂存区保持不变。

下一项 DSH-013 统一自动采集入口、整次工作期限和最终状态汇总。S1 尚未整体完成，S2～S7 尚未开始。
