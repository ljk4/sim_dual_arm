# 公共层与方案绑定迁移：进度

更新：2026-11-12（Asia/Shanghai）。基线：HEAD `c8215e0` 加阶段 00 开始时的工作树修改。任务规则见 [总要求](README.md)，文件归属见 [baseline-files.tsv](baseline-files.tsv)。

| 阶段 | 状态 | 交接或任务 | 当前结果 |
| --- | --- | --- | --- |
| 00 基线清点 | 完成 | [交接](deliveries/phase-00.md) | 逐文件归属、保护对象与历史样本已记录 |
| 01 目录与路径 | 完成 | [交接](deliveries/phase-01.md) | 129 文件迁入 shared；三容器挂载与 SDK 路径接通 |
| 02 绑定解析 | 完成 | [交接](deliveries/phase-02.md) | ScenarioBinding、load_binding 与只读 CLI |
| 03 仿真与运行端 | 完成 | [交接](deliveries/phase-03.md) | 22 份配方/说明迁移；prepare 消费方案；模型目录身份校验 |
| 04 学习端 | 完成 | [交接](deliveries/phase-04.md) | 数据、训练、bundle、prepare-policy 与 policy 接通 |
| 05 采集与并行 | 完成 | [交接](deliveries/phase-05.md) | 容器预检、选择传递与 worker 身份隔离 |
| 06 SDK 与 Teacher | 完成 | [交接](deliveries/phase-06.md) | 核心/完整包集分离与 Panda Teacher 扩展 |
| 07 末端反馈 | 完成 | [交接](deliveries/phase-07.md) | 有界反馈与能力接通；导出/审计解码前识别布局，legacy 按完整旧定义读取 |
| 08 轨迹来源 | 完成 | [交接](deliveries/phase-08.md) | 公共曲线约定与生产者信息分离；`curve_duration_ns`；旧轨迹按已知改名读取 |
| 09 部署检查 | 完成 | [交接](deliveries/phase-09.md) | 三端 `docker/deployment.yaml` 声明 GPU/源码挂载/网络/挂载/命令；公共检查器只验证共同规则与声明 |
| 10 方案模板 | 完成 | [交接](deliveries/phase-10.md) | 模板说明与 test_only 第二方案替换验证 |
| 11 代码精简 | 完成 | [交接](deliveries/phase-11.md) | 选择记录与摘要复用；采集/SDK/历史读取去重；边界检查保留 |
| 12 集成回归 | 完成 | [交接](deliveries/phase-12.md) | 双 worker Teacher、新 episode、并行中断、真实 bundle Policy 闭环与历史 episode 只读导出/审计 |
| 13 最终收口 | 完成 | [交接](deliveries/phase-13.md) | 活跃路径与导航收口；SDK、采集、转换/训练、模型准备与推理入口归入方案说明 |
| 提交前小闭环 | 部分完成 | [交接](deliveries/precommit-smoke.md) | 真实 2 条 episode 转换、128 步训练、新 bundle 与推理链路实际运行；短训练模型首个动作块被既有跟随偏差门禁拒绝，未产生执行反馈 |

## 当前接口与保护范围

- 真实 binding：`shared/scenarios/panda_stack/binding.yaml`；模型引用为空，推理需显式提供可验证 bundle。binding 的 `/data` artifact root 在项目容器内解析。
- Teacher：`panda_stack_api_msgs.teacher_chunk.v1`；核心 ROS SDK 为单包，Panda 完整 SDK 为三包。阶段 07 SDK 校验已记录，阶段 08 因 `FollowerReference` 改名已重建核心与完整 SDK，Policy SDK 未改。
- 反馈：`robot_api_msgs` 0.3.0，能力 `execution_feedback.gripper_set.v1`；夹爪 0～8、每夹爪手指 1～8。Panda 双侧双指与物理接触规则仍在 simulation。导出与审计在解码前识别布局：当前布局正常读取，完整已知 0.1.0 固定数组布局交给专用 legacy 解码并用当前 DTO 校验；其他布局仍拒绝。审计仅在整份定义匹配该已知布局时免除旧 backend 不具备的反馈能力字符串。
- 轨迹：公共记录只校验三次曲线表示、采样与硬边界归一化约定；`algorithm_version` 保留生产者真实值，不参与公共版本门禁。生产者曲线总时长字段为 `curve_duration_ns`（原 `ruckig_duration_ns`），计划时长、最终段与采样网格关系不变。旧记录定义仅差该字段名，读取时用 v1 摘要载荷校验并按当前摘要重建。
- 部署：三端各维护 `docker/deployment.yaml`（`service`、`source`、`mounts`、`networks`、`gpu`、`commands`）；公共检查器不按服务名映射 GPU、挂载或命令。当前 simulation/learning 声明 GPU，runtime 声明无 GPU；simulation 源码仅绑定本端目录，runtime/learning 绑定仓库根只读。
- 方案模板：`shared/scenarios/_template/` 只有说明与 `binding.example.yaml`，没有 `binding.yaml`；占位值未填写时校验失败，`--scenario _template` 不会选中 Panda。更换 checkpoint、算法、机器人、后端、任务、真机与能力时，配置切换和新增实现的修改位置见模板说明。
- Profile/观测/动作语义在迁移中保持原值；关键前缀 `a95b0f81…`、`9e0b5b5f…`、`452d8a15…`、`93689bc5…`，完整值与先前验证见阶段 00～03 交接。
- 历史数据、模型、prepared、正式台账与第三方资产保持保护范围；正式采集目标已为 50/50，业务回归使用独立诊断台账。
- 已有测试结果见各阶段交接。阶段 07 runtime 全量记录为 925 passed / 25 skipped / 3 failed，三个失败为 runtime 缺学习包的控制台环境问题；所属 learning 环境同文件 7 passed。阶段 08 runtime 全量为 937 passed / 28 skipped / 3 failed，失败项相同。

## 已知问题与下一步

- 公共层与方案绑定迁移已完成。阶段 12 已验证双 worker Teacher 调用（各一次、domain 210/211）、新格式 episode 导出/审计、并行入口中断收尾、真实 bundle 的有界 Policy 闭环，以及已选历史 episode 的只读导出/审计。历史导出与既有结果在 summary/labels/raw_predictions/frame_index 上逐字节一致，quality 判定一致；quality 的样本计数因现有 backend 日志覆盖更完整而不同（阶段 12 交接已说明）。诊断配置与结果保留在 `var/diagnostics/phase-12-20261112/`。
- 真实 Policy 软件链路已完成，但本局任务为 `TASK_FAILED: DROPPED`，导出质量因既有遮挡问题不合格。按实际记录，未放宽门禁，也未修改模型、物理参数或质量门禁；一次闭环不能作为成功率或真机能力结论。
- 当前已实现并被入口接受的组合只有 MuJoCo 仿真后端、ROS 2 运行端与 LeRobot ACT；第二机器人、第二后端、第二学习算法和真实驱动均未实现。`panda_stack_diag` 仅用于诊断复现，不是第二个生产方案。
- 阶段 13 只修改文档、导航与方案说明，并检查活跃路径与命令参数；未运行三端测试或重复闭环。容器和进程状态以运行前检查为准；既有交接中的空闲状态不是持续保证。
- 本轮未自动提交或推送。之前阶段 10/11 说明不变：阶段 10 的替换验证只证明配方选择进入生成配置；阶段 11 的产物形状差异已记录。
- 提交前小闭环使用阶段 12 的两条真实 Teacher episode 完成转换与 128 步训练；新 bundle 的推理链路已实际运行，但短训练模型的首个动作块被既有 `FOLLOWER_DEVIATION_EXCEEDED` 门禁拒绝，未产生执行反馈。按原始口径闭环未完整，按本轮“不追求推理成功、只要求软件路径可运行”的口径，未发现未捕获异常、NaN、错误模型来源或资源泄漏。诊断配置与结果保留在 `var/diagnostics/precommit-smoke-20261112/`。
