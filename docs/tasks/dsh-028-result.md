# DSH-028 验收记录

2026-11-01，已验收并整合25个文件；主HEAD与暂存区保持不变，未提交主工作区改动。

## 实现内容

- 原生运行时拆分为配置、ROS接线、episode生命周期、原生控制客户端、CLI与v3审计模块。
- 真实ROS租约/重置/命令ACK/查询/实测终态接线；三相机与测量按同context、capture和stamp精确配对。
- MCAP v3保留原始请求、接受系数、控制事件及真实反馈；逐块核对完整原始预测，按实际应用时间重建参考。
- 独立Compose overlay挂载源码/SDK/资产，runtime不使用GPU；后端与runtime使用不同配置变量。
- 原mock入口与v1/v2读写继续保留，recording完整性、runtime结果和task结果独立。

## 实际证据

证据根：`var/runs/dsh/dsh-028/20261101T114011Z/`。

| 检查 | 结果 | 产物 |
| --- | --- | --- |
| 纯单元测试 | 748通过 | unit-review2.xml |
| 真实ROS/物理/篡改反例 | 11通过 | ros-final/pytest.xml |
| 旧ROS/MCAP/CLI回归 | 115通过 | legacy-ros-final/pytest.xml、legacy-followup/pytest.xml |
| 独立观测缓存边界 | 4通过 | independent-exact-rework.json |
| 公共合并Compose | 运行/audit、后端关闭、资源清理全部通过 | public-final/result.json |
| 结构化日志 | 20事件schema/连续序号/summary通过，零丢弃 | logging-review.json |
| 源码和规范 | 153个Python与冻结候选一致，Ruff/diff通过 | source-accepted2-python.json、accepted.patch |

旧回归首轮103通过、1失败：历史测试将3硬编码为未知版本，本阶段新增v3后应使用99。
修正该夹具并与11项命令状态测试复验，共12通过；不是放宽未知版本拒绝规则。
最终源码较ros-final仅有两处测试夹具整理，业务源码一致。

## 协作与修复

本地headless会话`session-8e629e69-30b3-4ae1-8348-76c0ca67c545`。
首轮1127.017秒，唯一集中返工820.214秒；随后由主助手接管独立验收发现的问题。
DSH usage字段缺失，不据此估算token总量。

实际修复包括：rclpy保留属性命名冲突；嵌套DTO context解析；缓存身份淘汰、迟到淘汰、各分量墙钟过期及观测回退；
测试ROS域/配置覆盖/QoS；按块保留原始请求及冲突校验；公共Compose默认配置和数据根变量。
初次/返工失败输出保留，没有把候选自报结果直接当验收。

## 当前边界

这是诊断运动与记录基础设施，不是堆叠示教或ACT任务结果。S3尚待连续Policy链路及受限跟随接入。
正式数据0条，尚未开始任务ACT训练。用户已同意受限跟随并先作对比；下一项见DSH-029任务书。

## 整合后的捕获审计补充

审查完整数据关联时另补：同一capture身份的CDR内容冲突（只保留有界hash，不缓存图像），
测量速度的维度/有限性，以及ObservationIndex.reference_time与捕获时间一致性。
冻结`source-capture-followup`，真实MCAP三项新增篡改反例和正常运行共7检查通过，
结合之前结果共14个不同的真实ROS/物理检查通过；详见`capture-followup/pytest.xml`。
这两文件在主工作区局部修复，当前153个Python均与该补充快照一致。
