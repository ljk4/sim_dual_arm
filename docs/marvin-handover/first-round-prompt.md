# 第一轮实施提示词：P00 与 P01

本轮先完成基线准备，再交付 Marvin 基础场景与档案。P02 的标准控制接入及后续 Teacher、采集、训练由后续轮次实施。将下列提示词交给实施者执行：

```text
请在 /home/ljk/work/sim_dual_arm 中实际实施 Marvin 双臂交接扩展的第一轮：完成 P00，再完成 P01。直接推进到本轮完成条件，不停留在评估或建议；完成后停在 P02 之前。

先阅读：
- /home/ljk/work/sim_dual_arm/RULES.md
- /home/ljk/work/sim_dual_arm/docs/marvin-handover/README.md
- /home/ljk/work/sim_dual_arm/docs/marvin-handover/architecture.md
- /home/ljk/work/sim_dual_arm/docs/marvin-handover/model-notes.md
- /home/ljk/work/sim_dual_arm/docs/marvin-handover/task-contract.md
- /home/ljk/work/sim_dual_arm/docs/marvin-handover/phase-00-baseline.md
- /home/ljk/work/sim_dual_arm/docs/marvin-handover/phase-01-model.md
按后续阶段文档了解边界，本轮实施范围以 P00/P01 为准。

最终任务是左臂桌面抓取、空中交给右臂、左手释放退开、右臂入箱并释放。本轮交付基础模型与场景，为该任务建立可用起点。

实施要求：
1. 保留工作区已有修改、删除和未跟踪文件，不自动暂存、提交或推送。记录实际代码基线、开始时间、工作树、会话与日志位置；若使用隔离工作树，带入本次依赖的未提交基线。
2. 完成 P00：核对源资产、容器/挂载、SDK、实际依赖和输出路径，通过现行入口建立相关 Panda 测试与示教对照。既有失败如实记录，只修复本轮必需的环境或接入问题。
3. 完成 P01：纳入 Marvin 源资产，保持几何/惯性语义，建立固定根双臂装配、执行器和显式手指联动；构造桌面、长方块、开口箱体及三路相机场景，建立 Marvin 自身的档案和配方。参数以模型测量与实际运行确定。
4. 分开机器人装配、任务环境和 MuJoCo 机制：本体/关节/夹爪/TCP/接触绑定归机器人适配，桌面/物块/箱体与布局归任务环境，编译/复位/步进/渲染机制留在 simulation。机器人选择与任务选择独立，由所属端装配入口组合，不用任务名推断机器人型号。
5. shared/common 仅放跨端契约与确实共享的能力；端内复用代码留在所属端。方案目录保存声明性选择、源资产、档案、配方与必要 IDL，业务代码归各端。已有清楚职责的模块保留原位，不为四端创建对称 common 层、空框架或占位实现。
6. 用实际 MuJoCo 编译、复位、静置步进、低幅关节/夹爪运动及三路渲染验证模型。测量开端/中点/闭端总开度、联动、方向、TCP 与编译模型绑定；分别确认渲染和碰撞，不把公开 16 维推定为 nq/nv/nu。
7. 本轮可使用 simulation 内可复用的离线检查入口，任务环境构造不依赖尚未实现的 Teacher。标准 Robot API 控制接入留给 P02，不以复制控制链或写入 Panda 假档案来凑出可运行 backend。
8. 跑与实际改动相符的 Panda 回归及新增模型验证，保留图像和测量结果。保持原 Panda 业务参数与既有功能；不进入完整交接、采集、训练或真机驱动实现。

完成后写入 /home/ljk/work/sim_dual_arm/docs/marvin-handover/deliveries/phase-00.md 和 phase-01.md，并更新任务书进度。记录实际命令/环境、资产与产物路径、测试结果、机器人/任务/机制各自变更及调用边界、交给 P02 的未验证项。

最终报告：本轮实际完成的功能、修改文件、关键职责修复、验证与证据路径、剩余限制和 P02 接入点。没有实测或没有满足的条件保持未完成；阶段内必要修复继续推进，仅在缺少无法取得的输入或明确超出本轮范围时说明具体阻塞。
```

目录导航：[任务书入口](README.md) · [架构边界](architecture.md) · [P00](phase-00-baseline.md) · [P01](phase-01-model.md)
