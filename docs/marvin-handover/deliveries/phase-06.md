# P06 交付：最终验收与交付

日期：2026-11-13。**P06 完成。** 本轮按用户明确口径验收“Teacher 物理任务 + 数据到执行的诊断链路”，不要求 128 step 策略完成交接或达到成功率。复用 P01A～P05 已有证据，不新增训练、Policy rollout、采集或 evaluation 尝试（预算保持 3/3），不切换 test 种子，不扩大容差。工作树、历史产物与失败记录保留；未暂存、提交或推送。

## 验收结论

| 验收项 | 结论 | 实际证据 |
| --- | --- | --- |
| 资产与模型 | 通过 | simulation 内单份 Marvin 资产，宿主机/容器均由 `assets.py` 定位；P01A 编译模型、碰撞探针、联动与三路渲染检查 `succeeded` |
| 标准控制与运动 | 通过 | P02A 真实 Robot API/`SubmitFollower`、具名 I/O、反馈与停止；P02B 左右独立持稳、共持全过程、带物停止及连续空中交接入箱 |
| 完整物理任务 | 通过 | 同一最终模型与配方下 Teacher 连续 5 次复位成功，5/5 `TASK_SUCCEEDED`；P04 另取得 6 条同任务成功诊断数据 |
| 失败处理 | 通过 | 右手未接稳时左手不释放，`GATE_TIMEOUT: dual_hold/right_close`；SIGINT 中断完成停止、释放与录制关闭 |
| 数据与划分 | 通过 | 6 条诊断示教、1120 标签/条、4/2 SELECTED；按绝对时刻对照 6720/6720 标签可比；训练统计仅来自训练集 |
| 新模型执行冒烟 | 通过（诊断条件） | 新 bundle 加载、6 份有限 `60×16` Predict、1 个逻辑 command 的 4 个窗口被接受并实际执行、完整收尾；任务结果 `failed` |
| 架构修复 | 通过 | 跨端公共、端内机制、机器人/任务适配边界成立；本次静态导入检查无反向依赖，见[边界检查](../../../var/reports/p06/architecture-boundary-check.txt) |
| Panda 回归 | 通过（复用） | P03/P04 已有 Panda API、采集选择、导出质量、标签与真实导出结果；本轮无 Panda 路径代码改动 |
| 必要补充回归 | 通过 | 最后代码改动 `campaign_execution.py` 的 12 项 campaign 测试通过；runtime 缺依赖的终端进度测试在 learning 容器 7 项通过 |

连续 5 次成功是固定布局下的重复性验证，不是策略成功率。128 step 策略未完成整项交接，任务终态仍为 `failed`；本表“通过”仅指诊断链路跑通，不表示模型可用。

## 证据索引

| 阶段 | 关键产物 |
| --- | --- |
| P01A | [模型检查报告](../../../var/reports/sim-check/p01a-20261112/report.json)；资产 [marvin/](../../../simulation/mujoco/assets/marvin/README.md)、[assets.py](../../../simulation/mujoco/src/robot_backend_sim/physics/assets.py) |
| P02A | [P02A 交付](phase-02a.md)；`var/reports/p02a/p02a-022c0db8/evidence.json`、`var/prepared/p02a-marvin/` |
| P02B | [P02B 交付](phase-02b.md)；`var/reports/p02b/p02b-*`、`var/reports/handover-paths/p02b-verified-final-plan/plan.json` |
| P03 | [successes.json](../../../var/reports/p03/successes.json)；[右手未接稳](../../../var/raw/p03-miss-01/p03-miss-01/task_result.json)、[SIGINT 中断](../../../var/raw/p03-interrupt-01/p03-interrupt-01/task_result.json) |
| P04 | [SELECTED.json](../../../var/campaigns/marvin_handover_v1/SELECTED.json)；[绝对时刻边界审计](../../../var/reports/p04/boundary_audit_absolute_time.json)；各 episode `export/quality.json` |
| P05 | [执行事件复核](../../../var/reports/p05/execution_evidence_reconciled.json)；[接口审计](../../../var/reports/p05/interface_audit.json)；[bundle 摘要](../../../var/reports/p05/bundle-summary.json)；[训练摘要](../../../var/reports/p05/training-summary.json)；[服务摘要](../../../var/reports/p05/service-summary.json) |
| 回归 | `var/reports/p03/regression-*.log`、`var/reports/p04/regression-*.log`、`var/reports/p05/regression-*.log`、[P06 campaign](../../../var/reports/p06/regression-runtime-campaign.log)、[P06 learning](../../../var/reports/p06/regression-learning-console.log) |

## 数据、划分与训练统计

[SELECTED.json](../../../var/campaigns/marvin_handover_v1/SELECTED.json) 的 sha256 为 `f7f14f79…33af6d4`，6 条 `diagnostic: true`，训练 seed 4100–4103、验证 4104–4105，零基位置 `[4, 5]`。P05 的 [dataset manifest](../../../var/learning/marvin_handover_diag/dataset/manifest.json) sha256 为 `0790c6c0…c3f0c1e`，训练 4 条 4480 帧、验证 2 条 2240 帧，每条 1120 标签，输入仍为三路 RGB 与 16 维实测状态。

bundle 的 `processors/train_stats_summary.json` 标记 `source: training_split_only`，action count 4480，仅训练 episode 参与拟合；验证数据未进入统计。P05 的 [interface_audit.json](../../../var/reports/p05/interface_audit.json) 将 processor 统计与训练摘要逐项对照，最大差 `5.96e-8`，未发现重复归一化或轴顺序错配。

## P04 标签对照口径

边界对照以 [boundary_audit_absolute_time.json](../../../var/reports/p04/boundary_audit_absolute_time.json) 为准。它在 `label_sim_time_ns = raw.source_observation_time_ns + raw.times_from_observation_ns[index]` 的同一绝对时刻取 raw/applied 样本，不使用插值或最近邻近似。

| 项 | 结果 |
| --- | --- |
| 覆盖 | 6 条 episode、6720 标签全部可比，完整覆盖 |
| 同绝对时刻臂关节差 | 2736 条不同，全局最大 `0.032903 rad` |
| 同绝对时刻夹爪差 | 288 条不同，全局最大 `0.002870 m` |
| 每条 episode | 1120 标签中臂 456 条、夹爪 48 条不同 |
| 标签时刻 | 边界观测 + 50 ms；`reference_start` 与 `actual_start` 相等 |
| 原始块范围 | 标签对应 raw 偏移 0.30–2.15 s，位于原始块 0.05–3.00 s 覆盖内，时间残差为零 |

原 [boundary_audit.json](../../../var/reports/p04/boundary_audit.json) 与其脚本保留为历史：其中 965/1120、交接段 1.049 rad 是不同绝对时刻的比较，不再用于同刻偏差结论。实际执行标签、原始录制和 SELECTED 未改变。

## P05 执行复核与模型身份

[execution_evidence_reconciled.json](../../../var/reports/p05/execution_evidence_reconciled.json) 从后端控制事件补齐 chunk 生命周期。5 次提交属于同一逻辑 command `evaluation-05102-p05-eval-3-policy`；command 在 2.166 s 接受、2.37 s 开始、5.406 s 进入停止、5.622 s 取消。

| chunk | 观测 | 提交/接受 | 执行事实 | 结束 |
| --- | --- | --- | --- | --- |
| 1 | 42 | `STREAM_FIRST`，2.37 s 开始 | 21 条 `STARTED`/`FOLLOWING`，峰值位置误差 0.00325 | 3.35 s `RETIRED / HANDOFF` |
| 2 | 43 | `STREAM_NEXT`，接受 | 21 条反馈，0.00342 | 4.35 s `RETIRED / HANDOFF` |
| 3 | 67 | `STREAM_NEXT`，接受 | 21 条反馈，0.00180 | 5.35 s `RETIRED / HANDOFF` |
| 4 | 86 | `STREAM_NEXT`，接受并启动 | 3 条 `STARTED→STOPPING→CANCELED`，0.00009 | 5.622 s 随 command 停止取消 |
| 5 | 107 | 未接受 | 0 条 | `FOLLOWER_DEVIATION_EXCEEDED: right_joint2 0.10461851842063667 > 0.05` |

不能把 command 的 2.37 s 起点当作每个 chunk 的起点；流式正常交接块以 `RETIRED / HANDOFF` 结束，不要求 command 级 `COMPLETED`。原始 [execution_evidence.json](../../../var/reports/p05/execution_evidence.json) 的前三窗口 `finished=false` 是其缺少后端事件的摘要，不代表执行失败或仍在运行。

P05 记录 `/robot/state` 112 条，最大单轴增量 `1.369 rad`，各轴位移方向与模型首个目标一致；[interface_audit.json](../../../var/reports/p05/interface_audit.json) 核对 16 轴顺序、绝对位置语义与训练集统计，dataset/bundle/profile/schema 的顺序一致，processor 统计与训练摘要最大差 `5.96e-8`，6 份 Predict 均为有限 `60×16`，服务侧修正仅存在于夹爪且不超过 `0.003181 m`，臂修正为 0。模型身份由 Policy 服务就绪日志给出：bundle `7ddcdb62…d70a83`、processor `85f627f5…9895faf2`、model sha `57a4a92e…52fb3c2`、checkpoint smoke/128、Marvin profile `bc815504…7bd0947`。这证明新权重实际驱动物理机器人并正常收尾，不证明策略任务完成。

## 诊断条件与限制

inference 专用 follower 配方 [marvin_follower_inference_v1.yaml](../../../shared/scenarios/marvin_handover/profiles/control/marvin_follower_inference_v1.yaml) 的 `max_target_deviations` 与原配方对比如下：

| 轴组 | 原值 | 实际执行值 | 说明 |
| --- | --- | --- | --- |
| 左臂 7 轴 | `0.05 rad` | `[0.30, 1.30, 0.52, 0.98, 0.55, 0.44, 1.51] rad` | 最高 `left_joint7 1.51`，为原值 30.2 倍 |
| 右臂 1–4、6–7 | `0.05 rad` | `0.05 rad` | 未改 |
| `right_joint5` | `0.05 rad` | `0.10 rad` | 本方案唯一放宽的右臂轴 |
| 夹爪 | `0.01 m` | `0.01 m` | 未改 |

该预算比较拟合参考与锚点至逐时刻预测目标的分段线性轨迹，是实际触发 `FOLLOWER_DEVIATION_EXCEEDED` 的准入门槛；关节硬限位、速度/加速度与制动可行性未改。它不同于服务准入修正（只裁夹爪 `≤0.0032 m`）和机器人跟踪后端参考的实测误差（最大 `0.00342`）。因此本链路不声称原容差下可用，也只记录“短训练预测在 chunk 时间维上变化很小”，不据此认定模型直接输出训练均值。

## 职责修复与边界

| 归属 | 实际责任与修复 | 证据 |
| --- | --- | --- |
| `shared/common/` | 只保存跨端协议、schema、公共 Python 包与工具；未导入四端实现 | [边界检查](../../../var/reports/p06/architecture-boundary-check.txt) |
| simulation | `scene_loader.py` 按 `robot.assembly` 选择配置，`robot_adapters.py` 注册场景与 I/O；`physics/marvin/assembly.py`、`io.py` 负责机器人装配、I/O、TCP 与接触绑定；任务环境几何与布局仍由独立的 `physics/marvin/environment.py` 维护，经 `scene.py` 显式组合；`tasks/handover/` 负责门禁、评价与 Teacher。通用 controller/motion 只消费 `CompiledScene`/`RobotIO` | [scene_loader.py:29-53](../../../simulation/mujoco/src/robot_backend_sim/config/scene_loader.py#L29-L53)、[robot_adapters.py:34-46](../../../simulation/mujoco/src/robot_backend_sim/physics/robot_adapters.py#L34-L46)、[environment.py](../../../simulation/mujoco/src/robot_backend_sim/physics/marvin/environment.py)、[registry.py:11-29](../../../simulation/mujoco/src/robot_backend_sim/tasks/registry.py#L11-L29) |
| runtime | 通用采集/导出/审计不导入具体 Teacher 类型、不导入 MuJoCo；`teacher_client.py` 按 `<package>.teacher_chunk.v<N>` 解析生成类型并校验能力标识，`target_source.py` 只透传可选 `handoff_time_ns`；任务标识来自配方 `teacher.task_id` | [teacher_client.py:52-78](../../../runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/teacher_client.py#L52-L78)、`var/reports/p03/` 能力解析回归 |
| learning | 保持 ROS-free；未导入 ROS、MuJoCo、runtime 或方案专用消息；ACT 适配、处理器与 bundle 留在本端 | [边界检查](../../../var/reports/p06/architecture-boundary-check.txt)、[interface_audit.json](../../../var/reports/p05/interface_audit.json) |
| hardware | 仅保留边界说明；无真实驱动时不创建空模块或假驱动 | [hardware/README.md](../../../hardware/README.md) |
| 方案 | `shared/scenarios/marvin_handover/` 持有 binding、档案、控制/采集/学习/Policy 配方与专用 IDL，不持有机器人模型或业务实现 | [方案 README](../../../shared/scenarios/marvin_handover/README.md) |

其它已落地修复：运动配置不再强制 Panda 16 维名称；`kinematics/dual_arm.py` 使用机器人适配提供的具名关节/TCP/分组；`export_quality.py` 的阶段顺序、必需阶段与质量规则来自 campaign；campaign 停止宽限与中断终态补录为通用机制；`task_config.py` 仅在 Teacher 开启时校验长任务预算。未创建对称 `common/` 目录、第二算法或空硬件框架。

## 有效使用命令

路径基准：宿主机仓库根目录 `/home/ljk/work/sim_dual_arm`；simulation 容器工作目录 `/app`（宿主 `simulation/mujoco/`），runtime 容器 `/workspace`（宿主仓库根），learning 容器 `/workspace`。容器内 `/data` 与宿主 `var/` 是同一目录，`/shared` 为宿主 `shared/`。

```bash
# runtime 容器：构建本方案 SDK（robot_api_msgs + sim_api_msgs + marvin_handover_api_msgs）
runtime/ros2/scripts/run.sh build-ros-sdk --scenario marvin_handover --shared-root /shared
# 从 Panda 切换到本方案或其他包集前，先结束占用旧 SDK 的进程；构建后核对包集标记，再启动后端

# simulation 容器：离线模型检查与 Teacher 后端
scripts/run.sh marvin --run-id marvin-p01a
scripts/run.sh prepare --scenario marvin_handover --role teacher --seed 0 \
  --run-id sim-teacher --output /data/prepared/sim-teacher
scripts/run.sh backend --config /data/prepared/sim-teacher/backend.yaml --run-id backend-teacher

# runtime 容器：Teacher 准备与一次完整示教
runtime/ros2/scripts/run.sh prepare --scenario marvin_handover --role teacher --seed 0 \
  --run-id teacher --output /data/prepared/teacher
python3 -m dual_arm_runtime.runtime_node --project-root /workspace \
  --config /data/prepared/teacher/runtime.yaml \
  --output /data/raw/teacher --run-id teacher --episode-id teacher --once

# runtime 容器：pilot/formal 采集、台账与筛选
runtime/ros2/scripts/record.sh --scenario marvin_handover --purpose pilot \
  --count 1 --max-consecutive-failures 1 --backend-log /data/runs/backend/<run>/log
runtime/ros2/scripts/record.sh --scenario marvin_handover --purpose formal \
  --count 6 --max-consecutive-failures 3 --backend-log /data/runs/backend/<run>/log
runtime/ros2/scripts/run.sh campaign --scenario marvin_handover status
runtime/ros2/scripts/run.sh campaign --scenario marvin_handover select \
  --out /data/campaigns/marvin_handover_v1/SELECTED.json

# learning 容器：转换、短训练、bundle 与诊断 Policy 准入
learning/lerobot/scripts/run.sh convert --scenario marvin_handover
learning/lerobot/scripts/run.sh train --scenario marvin_handover --mode smoke
learning/lerobot/scripts/run.sh bundle --scenario marvin_handover --mode smoke \
  --run /data/learning/marvin_handover_diag/runs/<run> --checkpoint final.pt \
  --output /data/learning/marvin_handover_diag/bundles/<bundle>
learning/lerobot/scripts/run.sh prepare-policy --scenario marvin_handover \
  --bundle /data/learning/marvin_handover_diag/bundles/<bundle> --no-require-formal \
  --run-id <policy-run> --output /data/prepared/<policy-run>/policy

# runtime 容器：生成 Policy SDK（Service 需要的 protobuf/gRPC 代码）
runtime/ros2/scripts/run.sh build-policy-sdk

# learning 容器：启动 Policy 服务
learning/lerobot/scripts/run.sh policy \
  --config /data/prepared/<policy-run>/policy/policy.yaml \
  --log-directory /data/logs/act-policy/<policy-run> --run-id <policy-run>

# simulation / runtime 容器：一次 Policy evaluation 尝试（新预算；5100–5102 已消耗）
scripts/run.sh prepare --scenario marvin_handover --role policy \
  --bundle /data/learning/marvin_handover_diag/bundles/<bundle> \
  --seed <evaluation-seed> --run-id sim-policy --output /data/prepared/sim-policy
scripts/run.sh backend --config /data/prepared/sim-policy/backend.yaml --run-id backend-policy
runtime/ros2/scripts/run.sh prepare --scenario marvin_handover --role policy \
  --policy-binding /data/prepared/<policy-run>/policy/policy_binding.json \
  --bundle /data/learning/marvin_handover_diag/bundles/<bundle> \
  --seed <evaluation-seed> --run-id <policy-run> --output /data/prepared/<policy-run>/runtime
runtime/ros2/scripts/run.sh campaign \
  --config shared/scenarios/marvin_handover/configs/runtime/experiments/handover_campaign.yaml \
  run --purpose evaluation --mode act --seed <evaluation-seed> --run-id <attempt> \
  --runtime-config /data/prepared/<policy-run>/runtime/runtime.yaml -- \
  python3 -m dual_arm_runtime.runtime_node --project-root /workspace --once \
  --config /data/prepared/<policy-run>/runtime/runtime.yaml \
  --run-id {run_id} --episode-id {episode_id} \
  --output {attempt_dir}/stack/{run_id}/raw \
  --runs-output {attempt_dir}/stack/{run_id}/runs
```

以上命令已按当前 CLI 帮助与历史执行记录核对；方案 README 中原先放在 learning 容器的 `build-policy-sdk` 已更正为 runtime 容器的 `runtime/ros2/scripts/run.sh build-policy-sdk`，与脚本实现和 runtime 文档一致。P05 实际执行的 runtime 配置是 `var/prepared/p05-policy-2/runtime-follower/runtime.yaml`；当前 [policy_eval.yaml](../../../shared/scenarios/marvin_handover/configs/runtime/policy_eval.yaml) 已直接指向 `marvin_follower_inference_v1.yaml`，新的 prepare 会生成同一诊断跟随条件。已有产物位于 `var/prepared/`、`var/campaigns/marvin_handover_v1/`、`var/learning/marvin_handover_diag/`、`var/logs/act-policy/`、`var/runs/backend/`。本轮不重跑转换、训练、采集或 evaluation；后续效果实验必须使用新目录、新预算，并显式记录新的评价条件。

## 必要回归

P05 全量 runtime 单元记录（13:45）之后，代码只改动了 [campaign_execution.py](../../../runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/offline/campaign_execution.py)（13:51）：中断停止等待整个启动进程组完成，并携带实际 wall 时长。针对性补验结果：

| 验证 | 结果 | 日志 |
| --- | --- | --- |
| runtime 容器 `tests/unit/test_stack_campaign.py`（含中断等待、终态分类与 `backfill-terminal`） | **12 passed**，7.32 s | [regression-runtime-campaign.log](../../../var/reports/p06/regression-runtime-campaign.log) |
| learning 容器 `tests/unit/test_learning_console_progress.py` | **7 passed**，2.16 s | [regression-learning-console.log](../../../var/reports/p06/regression-learning-console.log) |

`test_learning_console_progress.py` 在 runtime 容器的 3 项失败是环境缺 `dual_arm_learning`：该测试的子进程直接导入学习端模块，runtime 镜像不安装该包；同一文件在 learning 容器通过。它不涉及本轮功能，不作为 Marvin 或 campaign 回归。

复用的既有回归：P03 runtime 全量（1052 passed）、Panda API（10 passed）、P02B 连续交接与持稳（5 passed）、Marvin 能力解析（2 passed）；P04 runtime 全量（949 passed）、Panda 标签/质量（109 passed）与真实 Panda 导出；P05 runtime 全量（964 passed、3 项上述环境失败）、learning/policy/Panda（114 passed）。P05 全量回归之后只改动了文档与该 campaign 代码，未触发全量重跑。

## 仍存在的限制

- 128 step 诊断模型未完成交接：任务以 `TASK_FAILED: CONTROL_FAILED: RUNTIME_SHUTDOWN` 结束，第 5 个窗口因 `right_joint2` 偏差被拒绝。链路跑通不等于策略效果。
- 实际执行使用推理专用轨迹保真容差，左臂最高为原值 30.2 倍、`right_joint5` 为 0.10 rad；原容差下可用性未验证，该配方不能作为正式执行参数。
- evaluation 预算已用尽 3/3；本轮未加训、未重采、未切换 test 种子。5K 量级效果训练与成功率验证是独立后续计划。
- 6 条诊断数据来自固定布局，只验证软件链路与数据格式，不证明泛化；训练统计仅来自 4 条训练 episode。
- P05 的原始/准入预测差异、同刻 raw/applied 差异和实测跟踪误差分别描述不同环节；本轮没有量化模型原始输出到后端拟合参考的完整差异。
- 底座网格仍仅用于渲染；本任务路径未进入未建模底座区域。hardware 目录没有真实驱动实现。

## 下一步

模型效果属于独立计划：使用新的训练预算和数据规模，在正式准入与保留原轨迹容差的条件下验证策略可执行性和任务成功率。它不改变本轮已完成的 Teacher 物理任务、数据/学习链路和职责修复结论。

目录导航：[任务书入口](../README.md) · [P05 交付](phase-05.md) · [P06 验收要求](../phase-06-acceptance.md)
