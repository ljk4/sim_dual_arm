# 阶段 13 交接：文档与最终收口

- 日期：2026-11-12（Asia/Shanghai）。状态：完成。公共层与方案绑定迁移到此收口。
- 依据：总要求、阶段 13、progress、阶段 12 交接，以及阶段 07、08、11 的接口与历史读取说明。
- 本轮只修改文档、导航与方案说明，未修改源码、shell、Compose、binding、配方、Profile、模型、数据、台账或第三方资产；未 commit/push/stash/reset/clean。

## 活跃路径与过渡入口

- 用 `rg` 检查活跃源码、shell、Compose、`container.env`、构建元数据、测试入口与操作文档中的 `contracts/`、`libraries/`、`infrastructure/`、`/contracts`、`/profiles`、`/deps` 等旧根路径：未发现需要修复的调用。相关路径已指向 `shared/common/...` 或 `shared/scenarios/...`。
- 历史报告和旧实验文档保留原路径与原语义；`docs/experiments/` 中的旧 `profiles/` 链接不批量替换。`collect_artifacts.contracts_digest` 仍识别产物来源清单中的历史 `contracts/` 前缀，这是对既有产物的兼容读取，不改成当前根路径。
- 仓库内没有需要保留的旧根目录软链接或转发入口；`shared/scenarios/panda_stack_diag/` 是阶段 12 诊断复现方案，不是旧路径兼容层，按原用途保留。
- 修复失效导航：根 README 删除的架构执行记录改为[架构任务索引](../../README.md)；`docs/history/README.md`、`shared/common/python/trajectory/README.md` 的失效链接改为 Git 历史或当前契约说明；`panda_stack_api_msgs/README.md` 的上下级链接改为实际存在的方案与公共接口文档。

## 文档与操作入口

- 根 [README](../../../../README.md)：架构概览区分 `shared/common` 公共约束、`shared/scenarios/<id>` 方案绑定和四端实现；采集、转换、训练、bundle 与 `prepare-policy` 命令统一使用 `--scenario panda_stack`，并注明直接 `--template`/`--config` 模式仍可用；删除已失效的架构执行记录链接。
- 新增 [`shared/README.md`](../../../../shared/README.md)：公共层与方案层职责、当前实例、接口版本与历史读取索引；[`shared/common/README.md`](../../../../shared/common/README.md) 明确不选择方案。
- [`panda_stack` 方案说明](../../../../shared/scenarios/panda_stack/README.md)：补齐 SDK、采集（手动 Teacher 与并行）、数据转换/训练/bundle、模型准备、Policy 推理操作入口；明确其为当前 MuJoCo + ROS 2 + LeRobot ACT 实例，并链接直接配置模式与已知边界。
- [`_template`](../../../../shared/scenarios/_template/README.md) 的“更换内容时改哪里”已覆盖 checkpoint、学习算法、机器人、仿真后端、任务、真机与新能力；本轮核对内容与当前实现一致，未改其结论。
- 新增 [`panda_stack_diag/README.md`](../../../../shared/scenarios/panda_stack_diag/README.md)：说明仅用于诊断复现、复用正式档案与配方、独立台账/种子/输出根，配置与 `var/diagnostics/phase-12-20261112/` 结果保留，不进入默认流程。
- [文档索引](../../../README.md)、[项目规范](../../../standards.md) 与四端 README 同步：公共接口约束与当前 Panda/ACT/MuJoCo 实例配置分开；simulation、runtime、learning、hardware 各自说明实现归属并链接 binding。
- [项目状态](../../../project-status.md) 补充阶段 12 的实际结果与公共层/方案绑定迁移完成状态。

## 最终接口与历史读取核对

- `robot_api_msgs` 当前版本 **0.3.0**；`FollowerReference.curve_duration_ns` 仅由 `ruckig_duration_ns` 改名，CDR 类型、顺序和边界不变。详见 [`robot_api_msgs/README.md`](../../../../shared/common/interfaces/ros/robot_api_msgs/README.md)。
- 反馈能力为 `execution_feedback.gripper_set.v1`；夹爪 0～8、每夹爪手指 1～8。Panda 的左右双指规则仍在 simulation 实现侧。
- 旧反馈使用固定数组布局的专用 legacy 解码；旧轨迹按已知字段改名识别，用 v1 摘要载荷校验并按当前摘要重建。完整历史 episode 的导出/审计已在阶段 12 通过，详见 [阶段 07](../deliveries/phase-07.md)、[阶段 08](../deliveries/phase-08.md) 与 [阶段 12](../deliveries/phase-12.md) 交接。
- Episode manifest 继续使用 `episode_manifest.schema.json` 的 `schema_version=1`。阶段 11 精简后的准备/worker 清单 `selection` 记录为 `field/declared/path/sha256`（有语义摘要时附 `semantic_sha256`）；不再复制 recipe 的 `template` 键，worker 的 `selection.profiles` 是唯一来源。权威说明见 [`schemas/README.md`](../../../../shared/common/interfaces/schemas/README.md) 与 [阶段 11 交接](../deliveries/phase-11.md)。

## 阶段 12 结果收口

- 双 worker Teacher 各一次真实调用（domain 210/211）、新格式 episode 导出/审计、并行入口中断收尾，以及已选历史 episode 的只读导出/审计均已验证。历史导出与既有结果在 `summary.json`、`labels.json`、`raw_predictions.json`、`frame_index.json` 上逐字节一致；`quality.json` 的样本计数因 backend 日志覆盖更完整而不同，质量判定未变。
- 真实 Policy 软件链路完成（Policy 服务 45 次 `Predict` 全部成功、44 个动作块、审计通过），本局任务为 `TASK_FAILED: DROPPED`，导出质量因既有遮挡不合格。未放宽门禁，未修改模型、物理参数或质量门禁。
- 当前实现组合只有 MuJoCo 仿真后端、ROS 2 运行端与 LeRobot ACT；真机、第二机器人、第二后端和第二学习算法均未实现。`panda_stack_diag` 仅用于诊断复现。

## 验证

- 对活跃项目 Markdown（根 README、docs、shared 与四端 README）做相对链接检查：修改后全部可解析。`RULES.md` 中的 `cases/`、`hosts/` 是上游参考链接，不在本仓库内，保持原文。
- 用 `rg` 复查活跃文件中的旧根路径、旧文档名与软链接：无旧调用、无旧导航、无仓库内软链接；仅保留历史报告路径与 `collect_artifacts` 的历史产物兼容读取。
- 对照 CLI 定义核对新增文档命令参数：`record.py`/`record_parallel.py` 的 `--scenario`、`--shared-root`、`--target-total`、`--workers`，learning `cli.py` 的 `--scenario`/`--shared-root`，simulation `prepare_backend.py` 与 runtime `prepare_run.py` 的 `--role`/`--bundle`/`--policy-binding`/`--seed`/`--run-id`/`--output` 均存在且互斥关系一致。宿主机没有 `pydantic`，`scenario.py` 未在宿主机执行；方案说明已注明该命令需在容器内运行。
- 本阶段只改文档与导航，未修改代码或 shell，因此未运行三端测试，也未重复阶段 12 闭环。

## 已知问题与未执行项

- Policy 本局 `DROPPED` 与导出遮挡质量不合格是既有模型/物理业务问题，按实际记录保留；一次闭环不能作为成功率或真机能力结论。跨种子泛化、腕部相机遮挡与质量门禁优化仍需后续单独验证。
- 阶段 13 未运行测试、完整训练、正式补采或完整评价集；容器与进程状态以运行前检查为准。
- 未自动提交或推送。历史交接、诊断结果、模型、prepared、正式台账与第三方资产保持原状。
