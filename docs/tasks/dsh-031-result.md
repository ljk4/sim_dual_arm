# DSH-031 验收记录

2026-11-01，25文件已验收整合。171个Python文件与冻结候选一致；主HEAD/暂存区不变，未提交主工作区改动。

## 实现与边界

新增独立FollowerReference/SubmitFollower，复用同一物理owner、规划器、真实反馈与块历史；
原31个msg/srv/action逐字不变。严格SubmitNative保留，配置显式启用follower才声明能力。
接收截止时间与执行起点分别校验；重试保留原接受时刻、digest、完整60点源及每块身份。
共享纯Python记录能重建原系数、actual参考及数值修正，无需NumPy/Ruckig/ROS。

Ruckig 0.15.3使用固定sdist及带哈希构建依赖，编译工具留在builder；正式镜像已实际构建。
新SDK为35个契约源文件，摘要0dcb3c17156bfbdffa68d29ab7a0f546614601910eb125fabf015e72d578ab1c，
已提升到.build/ros-dev并保留原SDK备份。正式follower-dev镜像ID为
sha256:756848276b2f5b37e3b6de64422ee259670e60cc9353c676934bf68e2f1d0809。

## 独立验收

证据目录：var/runs/dsh/dsh-031/20261101T134521Z/。

| 检查 | 结果 |
| --- | --- |
| 全部unit | 852通过 |
| 原生/真实ROS/EGL/CDR/MCAP | 104通过 |
| 旧ROS/MCAP/CLI | 115通过 |
| 真实公共Compose入口 | 正常运行、信号、总超时、已有项目保护4项通过 |
| 独立公共ROS客户端 | 三块60点连续执行到COMPLETED；16份反馈重建；跨模式/未选尾部篡改拒绝；租约实际停止；reset旧请求失效 |
| 真实队列/停止/reset | 临界引用队列满后实际停止；活动reset拒绝，停止后reset保留旧记录 |
| 独立记录篡改 | 实际终态时域/target_count/接受时间/final事实及digest重算攻击均拒绝 |
| 日志 | 34事件schema及连续序号通过，两个summary完整、零丢弃 |
| 纯契约依赖阻断、Ruff、diff | 通过 |

首轮漏验实际终态时域、count和接受时间；唯一集中返工修正。
返工误把deadline与start串联，监督者改为独立约束并加入两个合法反例。
公共首次忙轮询探针未及时供块，真实UNDERRUN证据保留；按运行时20ms轮询后在并行物理检查下通过，
不据此宣称任意负载均满足期限。队列探针原先违反活动reset前置条件，修正探针后通过，未放松backend规则。

本地headless会话session-f581c23e-160f-4b7e-a609-80eb9c3aae49，
首轮873.197秒/122步/143工具，返工273.094秒/36步/38工具。usage有缺项，不估算总费用或节省比例。

S3尚差真实Policy RPC与连续runtime/MCAP v4；正式数据0，任务ACT训练未开始。继续分阶段推进。
