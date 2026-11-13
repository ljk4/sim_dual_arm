# P02 双臂运动接入总览

状态：2026-11-12 P02A、P02B 已交付；P03 已交付，见[交付记录](deliveries/phase-03.md)。

前置：P01A 模型整理与碰撞/联动复核通过。P02 分为两个顺序交付，既有 P01 离线检查继续作为模型回归入口。

| 子阶段 | 范围与当前状态 | 交付记录 |
| --- | --- | --- |
| [P02A 标准控制接入](phase-02-control.md) | 具名 I/O、后端装配选择、control/stream/follower、真实 Robot API 运动/反馈/停止；无任务插件 | `deliveries/phase-02a.md` |
| [P02B 接触与候选路径](phase-02-contact-kinematics.md) | 已交付：每指接触、左右独立持稳、双臂 IK/路径净空、共持全过程、带物停止，以及连续空中交接、退手、运输与入箱释放诊断 | [deliveries/phase-02b.md](deliveries/phase-02b.md) |

P02A 通过后进入 P02B，P02B 通过后进入 P03。短时共持稳态与共持全过程分别验收，抬升后稳定不能代替抓持建立至抬升结束的相对关系保持。每个子阶段同时完成必要职责修复与受影响回归。

机器人装配、I/O、夹爪/TCP/接触绑定归机器人适配；求解、步进、命令调度与反馈机制留在 simulation；抓持区、交接/避让/放置目标与允许接触归任务实现/配方。P02B 建立的目标与路径模块由 P03 Teacher 直接复用，诊断脚本只调用和记录。

P02B 的连续物理诊断通过标准 Robot API 执行，尚未接入任务生命周期、Teacher 或采集。P03 在已验证目标/路径基础上补齐这些能力。P02 不要求不完整 binding 通过 collection；具体入口、SDK 包集与完成条件分别以两个子阶段文件为准。

目录导航：[任务书入口](README.md) · [前一阶段 P01A](phase-01-readiness.md) · [P02A](phase-02-control.md) · [P02B](phase-02-contact-kinematics.md) · [下一阶段 P03](phase-03-task.md)
