# P06 回归与交付

**状态：2026-11-13 已完成。** 实际验收结论、证据索引、职责修复摘要、有效命令、必要回归与集中限制见 [deliveries/phase-06.md](deliveries/phase-06.md)。

前置：P01A、P02A、P02B、P03～P05 的功能交付与相关问题修复完成，P00/P01 历史成果保留。目标：确认新任务、学习执行链和职责修复都成立，并留下可照做的使用说明。

2026-11-13 用户明确本轮以学习执行链跑通为目标，不追求短训练模型的仿真交接成功。P05 已取得新权重加载、Predict、实际接受/执行与收尾证据，可进入 P06；其 inference 专用轨迹保真容差和策略效果限制在最终交付中如实保留。evaluation 已用尽 3/3，本阶段不新增推理尝试、改用 test 种子、加训或扩大跟随门槛。

## 最终验收

| 交付 | 必须达到的结果 | 证据来源 |
| --- | --- | --- |
| 资产与模型 | simulation 内单份机器人资产，宿主机/容器定位一致，同臂/跨臂/环境碰撞有效，联动约束来源明确 | P01A 新报告；P01 历史报告保留 |
| 标准控制与运动 | 具名 I/O、开度/速度、真实 Robot API/SubmitFollower 执行、反馈与停止成立；真实持物和候选路径可供任务复用 | P02A/P02B 请求、实测、接触与净空结果 |
| 完整物理任务 | 同一最终模型/配方下 Teacher 连续 5 次复位成功，每次覆盖抓起、悬空接稳、左手释放退开、完整入箱、双手释放和稳定 | 复用 P03/P04 全程记录与物理指标，不重新采集五次 |
| 失败处理 | 右手未接稳时左手不放物，实际仿真中断可停止；失持/超时等相关判断有效 | P02A/P02B/P03/P04 故障证据与针对性测试 |
| 数据 | 6 条合格实际示教，4/2 独立划分，时间/标签/阶段质量审计通过，训练统计仅来自训练集 | P04/P05 select、质量/审计与 dataset manifest |
| 新模型执行冒烟 | 本次新权重实际加载、Predict、至少一个后端接受窗口、对应实测执行反馈与完整收尾；明确诊断配方条件 | P05 prepared、Policy/runtime/backend 日志、执行事件复核与反馈，可直接复用 |
| 架构修复 | 跨端公共、端内机制、机器人/算法适配、任务实现归属清楚；业务代码在各端；两个方案通过各自配置选择 | 阶段修复说明、代码依赖和双方案相关回归 |
| Panda 回归 | 原模型、业务数值与契约有效，受改动路径无新增失败，实际示教仍可完成 | P00 对照及 P03/P04 已有示教与相关回归；仅后续代码改动影响相同路径时补验 |

连续成功是 Teacher 在固定布局的重复性验证，不据此报告策略泛化成功率。短训练模型的交接成功和原轨迹保真容差下可用性不作为本轮完成条件；新模型执行反馈必须取得。P05 已有三个正常 HANDOFF 窗口，可复用，无需为 P06 再启动 Policy。

## 扩展边界验收

- 已实现能力支持的变化通过新增方案、档案与配方完成；新增机器人/任务/算法能力的代码集中在对应适配与实现，通用步进、调度、同步、录制和会话机制通过明确能力边界复用。
- 机器人装配/I/O/夹爪/TCP/接触解释与任务环境/Teacher/门禁/成功判定分别维护。任务消费机器人能力，机器人适配不导入 handover/stack 行为；更换任务不复制机器人执行链。
- 具体 Teacher IDL 的加载与转换位于所属端专用适配，通用采集流程不直接依赖 `panda_stack_api_msgs` 或 `marvin_handover_api_msgs`。Robot/Policy API 相同的部分复用现有接口适配。
- ACT 专用代码和 MuJoCo 专用机制保持在所属端；方案目录持有声明性资源与参数，`shared/common/` 不承接端内业务或反向导入四端。hardware 维持边界说明，等待实际驱动后再提取共性。
- 模型资产归 simulation，Panda/Marvin 各保留一份来源清楚的资产；方案选择与当前容器挂载下的资产解析都真实可用。双方案验证时切换并校验实际 SDK 包集。

以本次两个方案及实际调用/相关测试证明这些边界，无需另建交叉机器人任务组合、第二算法或对称目录树。首次修复机制对具体适配的依赖是本次工作的一部分；“机制稳定”以职责和回归行为为准。

## 回归范围

验证随改动开展，不为文档中的模块名逐一增加镜像测试。已有测试可扩展到 Marvin 的真实数据/配置，必要的新测试聚焦本次新增行为：

| 受影响路径 | 当前可复用的测试入口举例 | 本次新增验证 |
| --- | --- | --- |
| 档案、binding、prepare | `test_scenario_binding.py`、`test_profile_cameras.py`、`test_prepare_scenario.py` | 独立 Marvin 档案、路径、选择与模型一致性 |
| 场景、I/O、运动与接触 | `test_sim_scene.py`、`test_panda_io.py`、`test_scene_kinematics.py`、`test_gripper_completion.py`、`test_contact_unload.py`、`test_follower_execution.py` | 新关节/开度绑定、左臂与双臂路径、接稳与停止 |
| 任务与 ROS Teacher | `test_stack_task.py`、`test_stack_teacher.py`、`test_task_prepare_ros.py`、`test_teacher_extension_ros.py` | handover 门禁、阶段目标、扩展 capability、复位与终态 |
| 采集、导出与审计 | `test_record_scenario.py`、`test_record_workers_scenario.py`、`test_stack_label_export.py`、`test_stack_export_quality.py` | 交接阶段完整性、左右夹爪标签、独立 campaign 和实际 6/4/2 说明 |
| 学习与新模型 | `test_learning_scenario.py`、`test_learning_bundle_cli.py`、`test_prepare_policy.py`、`test_learning_cpu_lifecycle.py`、`test_policy_config.py` | Marvin 绑定、真实短训练产物及诊断/正式准入边界 |

测试文件位于 [tests/unit](../../tests/unit/) 或 [tests/integration](../../tests/integration/)。通过各端已有 `scripts/run.sh test` 在具有相应依赖的容器中执行；simulation 的脚本路径与工作目录见[测试说明](../../tests/README.md)。依赖缺失导致跳过的物理/ROS 测试不算实际运行通过；只有必要且仍缺证据的受影响路径需在对应容器补验，不因此重跑已有有效证据的实验。

公共路径变化后覆盖 Panda/Marvin；阶段已经覆盖的结果可复用，只有后续改动影响同一路径时再补跑。全程集成记录、配置/单元测试分别说明，不能用静态验证或替身测试替代物理与新模型运行证据。

同一测试在具备依赖的环境取得结果；两条既有运动学失效测试由 P02B 修正，反馈 MCAP 在 runtime 验证。交付记录保留完整路径、工作目录/容器和退出结果；历史范围简写不能冒充可复制命令，跨环境缺依赖也不能概括为功能通过。

## 使用说明与交付记录

1. 新方案 README 提供可直接使用的模型来源、SDK 构建、prepare/backend、Teacher 试采、诊断清单筛选、转换、短训练、bundle 与 policy 冒烟命令；标清宿主机/容器及路径基准。
2. 入口失败应给出实际缺失能力、档案/SDK/模型不匹配或控制原因。显式选择 Marvin 时，不回退 Panda 模型、任务、profile 或 Teacher。
3. 汇总本任务实际修复的耦合：区分机器人、任务、算法/协议和端内机制的变更，说明触发场景、当前调用/数值来源与两方案证据。未使用的未来平台能力不作为待补交付。
4. 更新本目录进度与相关现行 README，删除由本次实现淘汰的说明；保留历史实验的条件与适用范围，不把历史路径当成本次产物。
5. `deliveries/phase-06.md` 链接各阶段实际产物与结果，集中列出仍存在的限制。如果仍缺 Teacher 整项任务成功、真实新模型执行反馈或必要回归，就保持对应阶段未完成，并给出具体下一步。模型未完成交接、128 step 的效果不足、诊断配方的保真容差差异作为限制记录，不为这些项目自动开启新实验。

交付保持未自动暂存、提交和推送，保留用户原有工作。最终报告说明修改内容、验证范围与实际未完成项。

目录导航：[任务书入口](README.md) · [前一阶段 P05](phase-05-learning.md) · [文档索引](../README.md)
