# DSH-030 验收记录

2026-11-01，已验收并整合11文件；163个Python文件与冻结候选一致。主HEAD/暂存区不变，未提交主工作区改动。

## 实现

- 明确区分严格PreparedChunk与PreparedFollowerPlan，复用同一NativeController/NativeMotion/StreamSchedule。
- 新FOLLOWER请求指纹覆盖算法、原始60×16窗口、系数、限制/偏差及终态时域；旧MOTION/CHUNK指纹不变。
- 非final连续接续，继承真实系数末端p/v；final使用Ruckig离散时域，在2ms网格上必要时延长，到目标后等实测稳定。
- 浮点边界规范化只有既有数值容差内的系数舍入适用；保留raw采样、actual下发与修正量，未改原预测或PandaIO严格硬限位。
- 同一流禁止混模式；原lease/context/ACK/幂等/历史容量/迟到/刹车规则延续。

## 独立验收

证据根：`var/runs/dsh/dsh-030/20261101T131346Z/`。

| 检查 | 结果 |
| --- | --- |
| 冻结候选全部unit | 807通过 |
| 真实native/ROS/EGL/CDR/MCAP回归 | 102个不同检查通过 |
| 独立公开控制API物理探针 | 3块真实执行，3次RETIRED、整体COMPLETED，完整保存3×60 raw预测 |
| 独立双向夹爪边界 | 完整controller路径每方向142个非零修正tick，actual严格在0~0.08m内，实测稳定完成 |
| 旧模式固定指纹 | MOTION与CHUNK各一份16维真实profile基线逐字一致 |
| 纯契约依赖隔离、Ruff/diff | 通过 |

三块独立运动仿真时长1.202s，最大参考跟踪误差0.006582rad；同时逐步核对公开NativeExecution DTO、
原系数与应用参考、测量及应用/采样时钟。独立探针首次末尾查询调用参数写错，修正后通过，原输出保留。

夹爪测试先通过严格MOTION运动到0.002/0.078m，再真正执行follower闭合/张开；
raw曲线端点分别约-2.85e-11与0.08000000002849m，实际应用端点分别为0/.08m，
反馈和修正量逐tick可重建。这是控制诊断，不是抓取试采。

11块候选物理测试另实际完成：1699ticks，final请求250ms、执行324ms（延长74ms），
2ms网格、实测稳定200ms；有真实CONTACT、underrun/迟到/跟踪故障/lease过期/模式/幂等/reset反例。
综合Docker回归首轮101通过，1项仅因产物默认写只读源码目录失败；指定DSH_REPORT_DIR=/review后复验该项通过。
没有修改业务代码来规避该失败，也没有把首轮标成全通过。

## 协作与当前边界

本地headless会话`session-3ff76b7a-a3d7-487d-94df-22132c91b7e2`，
首轮1421.382秒、200步、205次工具调用；无需返工。usage缺项，不估算费用或节省比例。

本项尚未提供公开SubmitFollower ROS入口。下一项增加独立FollowerReference类型和服务，保留旧33IDL。
正式数据仍0条，ACT任务训练尚未开始，S3尚未完成。Ruckig本地容器依赖预检已通过，正式构建配方在下一项补齐。
