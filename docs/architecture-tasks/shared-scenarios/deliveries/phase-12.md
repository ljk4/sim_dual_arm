# 阶段 12 交接：有界集成回归与历史读取

- 日期：2026-11-12（Asia/Shanghai）。状态：完成。未进入阶段 13。
- 依据：总要求、阶段 12、progress、阶段 11 交接，以及阶段 06～08 的最终 Teacher 能力、反馈布局与轨迹字段说明。
- 本轮未修改真实 binding、业务配方、Profile、模型、prepared、正式台账、历史数据与第三方资产；未 commit/push/stash/reset/clean。

## 环境与资源

三个容器均在运行且业务进程空闲；`run.sh` 与预检的 ROS SDK 检查通过（`b79e8a7e…`，3 包），Policy SDK 检查通过（`f1f2f6ee…`）。运行前 GPU 空闲 5.3 GiB，磁盘剩余 53 GiB，无残留 backend/runtime/policy 进程。Teacher 并行使用 domain 210/211，中断补跑使用 213，Policy 使用默认 42。

诊断根为 `var/diagnostics/phase-12-20261112/`（容器 `/data/diagnostics/phase-12-20261112/`）。正式台账保持 156 条记录、formal 50/50；本轮全部运行使用独立台账与种子 900000/900001/900100，未占用正式预算。

临时诊断方案 [`shared/scenarios/panda_stack_diag/`](../../../../shared/scenarios/panda_stack_diag/) 只引用原 Panda Profile、Teacher 配方与扩展包，另给诊断 campaign（[`stack_campaign_diag.yaml`](../../../../shared/scenarios/panda_stack_diag/configs/stack_campaign_diag.yaml)、[`stack_campaign_interrupt.yaml`](../../../../shared/scenarios/panda_stack_diag/configs/stack_campaign_interrupt.yaml)）设置诊断输出根、台账、种子与两/一次尝试上限；业务配方与质量门禁逐项复制自正式 campaign，真实 `panda_stack/binding.yaml` 未改。

## 历史读取修复

完整历史 episode 导出/审计此前会在反馈布局检查处拒绝，缺少对已知 0.1.0 固定数组布局的读取路径。本轮只扩充该具体路径：

- [`feedback_definition.py`](../../../../runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/offline/feedback_definition.py)：`require_supported_feedback_definition` 在解码前识别布局，返回是否为已知 legacy 固定数组布局；当前布局返回 false，其他布局仍明确拒绝。
- [`export_episode.py`](../../../../runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/offline/export_episode.py) 与 [`mcap_reader.py`](../../../../runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/offline/mcap_reader.py)：legacy 反馈载荷交给既有 `decode_legacy_execution_feedback` 解码并校验为当前 DTO，绝不交给当前生成类型。审计仅在整份录制定义与已知 legacy 布局一致时，以该布局证据替代旧 backend 不具备的 `execution_feedback.gripper_set.v1` 能力字符串；其他缺失能力仍失败。
- 未实现通用迁移框架，未改写任何 MCAP。测试更新见下。

## 运行 1～2：两个隔离 worker 各一次 Teacher 尝试与一个新 episode

命令（宿主机，`timeout --signal=TERM 900` 包裹）：

```bash
runtime/ros2/scripts/record_parallel.sh --scenario panda_stack_diag \
  --purpose pilot --count 2 --workers 2 --domain-base 210 \
  --max-consecutive-failures 2
```

- 预检：`record_preflight --scenario panda_stack_diag --purpose pilot --parallel` exit 0；计划见 `preflight/teacher-preflight.json`。
- 池目录：`var/record-pools/pool-1794455130527176076`；台账：`teacher-pool/attempts.jsonl`；宿主机输出：`teacher-pool/host.log`。
- 两个 worker 身份：worker-01 / domain 210 / `/record_pool_1794455130527176076_1` / seed 900000；worker-02 / domain 211 / `/record_pool_1794455130527176076_2` / seed 900001。每个 worker 恰好一次尝试，共 2 次；连续失败上限 2 未触发。
- 实际 Teacher 服务：两份新录制 `BackendInfo.capabilities` 均含 `panda_stack_api_msgs.teacher_chunk.v1`；各自 backend 日志记录 49 次 `backend.task.teacher_chunk` 成功调用，无跨 worker 的 topic/命名空间/输出混用。启动初次的 `TASK_NOT_QUIESCENT` 与收尾的 `TASK_NOT_RUNNING` 单调警告后由正常链路继续/结束。
- 结果（两次均为真实任务成功）：任务 `succeeded/TASK_SUCCEEDED`，录制 `complete=true`，导出质量 `qualified=true`，`launch_result.json` 的 runtime/export 退出码均为 0。worker-01：labels 480、observations 505、raw_predictions 49；worker-02：labels 480、observations 503、raw_predictions 49。两次 raw_bytes 约 394/393 MB，wall 约 40 s。
- 从 worker-01 的新录制完成审计：反馈布局为当前 `GripperFeedback[<=8]`，轨迹字段为 `curve_duration_ns`（无 `ruckig_duration_ns`），两者均无需 legacy 解码；审计通过，`message_counts` 与 manifest 逐项一致，49 个 follower chunk、445 observations。审计同时校验了当前 `FollowerReference` 摘要、曲线与样点，未触发 `FOLLOWER_DIGEST_MISMATCH`。审计输出 `audit/pilot-900000-worker-01-audit.json`。
- 新 episode 的导出由采集入口自动完成（`--no-previews`），任务、录制、质量结果分别保存在 attempt 根的 `task_result.json`、`manifest.json` 与 `quality.json`；未另加单 worker 采集。

## 运行 3：受支持并行入口的中断清理

直接配置模式下补一次短尝试（独立台账 `interrupt/attempts.jsonl`、seed 900100、1 worker、domain 213）。在 `recording.started` 后约 3 s 向宿主机 `record_parallel` 进程发送 SIGTERM：

```bash
runtime/ros2/scripts/record_parallel.sh \
  --config shared/scenarios/panda_stack_diag/configs/stack_campaign_interrupt.yaml \
  --runtime-config shared/scenarios/panda_stack/configs/runtime/teacher_collect.yaml \
  --backend-template shared/scenarios/panda_stack/configs/simulation/backend/backend_stack_teacher.yaml \
  --purpose pilot --count 1 --workers 1 --domain-base 213 \
  --max-consecutive-failures 1
```

- 宿主机退出码 130，pool 输出 `reason=interrupted`、`cancelled=1`；池目录 `var/record-pools/pool-1794455291094948747`，日志 `interrupt/host.log`。
- 台账写入 RESERVED → RUNNING → INTERRUPTED（`exit_code=130`、`failure="batch stopped: interrupted"`、部分 raw_bytes），未伪造成 FINISHED/qualified。
- 收尾检查：worker 的 simulation 容器已删除；controller PID 568 与其 collector/runtime 子进程均不存在；`interrupt/.run.lock` 已删除；两份诊断台账的 advisory lock 均可立即获取；宿主机无残留 `record_parallel`/`collect_attempt` 进程。`.ledger.lock` 作为持久锁文件保留，但未被持有。

## 运行 4：现有真实 bundle 的有界 Policy 闭环

模型包 `/data/learning/bundles/stack-act`（bundle digest `952bd795…`，processor `dbaa26dd…`）未改。按真实 `panda_stack` binding 生成诊断运行：

```bash
# learning
learning/lerobot/scripts/run.sh prepare-policy --scenario panda_stack --shared-root /shared \
  --bundle /data/learning/bundles/stack-act --run-id phase12-policy \
  --output /data/diagnostics/phase-12-20261112/policy/prepared-policy
# simulation
scripts/run.sh prepare --scenario panda_stack --shared-root /shared --role policy \
  --bundle /data/learning/bundles/stack-act --seed 20001 --run-id phase12-policy \
  --output /data/diagnostics/phase-12-20261112/policy/simulation
# runtime
runtime/ros2/scripts/run.sh prepare --scenario panda_stack --shared-root /shared --role policy \
  --policy-binding /data/diagnostics/phase-12-20261112/policy/prepared-policy/policy_binding.json \
  --bundle /data/learning/bundles/stack-act --seed 20001 --run-id phase12-policy \
  --output /data/diagnostics/phase-12-20261112/policy/runtime
```

三次 prepare 均通过真实 bundle 校验，binding sha `9ee2fd32…`；runtime 有效模型摘要与 learning 生成的 `policy_binding.json` 一致。随后启动 Policy 服务与推理 backend，再执行一局：

```bash
# learning
learning/lerobot/scripts/run.sh policy \
  --config /data/diagnostics/phase-12-20261112/policy/prepared-policy/policy.yaml \
  --run-id phase12-policy \
  --log-directory /data/diagnostics/phase-12-20261112/policy/policy-logs-20261112T034918Z
# simulation
scripts/run.sh backend \
  --config /data/diagnostics/phase-12-20261112/policy/simulation/backend.yaml \
  --run-root /data/diagnostics/phase-12-20261112/policy/backend-runs \
  --run-id phase12-policy
# runtime（有时间上界，一局）
timeout --signal=INT 300 runtime/ros2/scripts/run.sh policy \
  --config /data/diagnostics/phase-12-20261112/policy/runtime/runtime.yaml \
  --output /data/diagnostics/phase-12-20261112/policy/raw \
  --run-id phase12-policy-ep1 --episode-id phase12-policy-ep1 --reset-seed 20001 \
  --runs-output /data/diagnostics/phase-12-20261112/policy/runs
```

- 链路：Policy 服务完成 45 次 `Predict` RPC（全部 `OK`），runtime 记录 44 个动作块、44 个 follower reference、487 条 feedback、483 个观测；会话以 `CloseSession OK` 收尾。backend 收到 44 次 `submit_follower`，无失败。
- 实际任务结果：`TASK_FAILED: DROPPED: a block was dropped or left the table`（`task_result.json` 与 manifest 一致）；录制仍 `complete=true`，runtime 以 `stopped/TASK_FAILED` 正常停止，进程退出码 0。未放宽任何门禁。
- 审计通过（`audit/phase12-policy-ep1-audit.json`）：44 chunks、483 observations、`message_counts` 与 manifest 一致。导出 `policy/export`：labels 430、observations 483、raw_predictions 44；质量 `qualified=false`，失败原因为 `right_wrist.target` 遮挡 1.550 s 与 `global.source` 遮挡 2.700 s（既有业务遮挡问题，非本轮迁移改动）。
- 停止链路：Policy 服务收到 SIGTERM 后写入 `policy.closed exit_code=0`，50051 端口释放；backend 写入 `backend.close.begin/end`，日志 summary 为 `complete`、2936/2936 写入、0 dropped；三容器恢复空闲。

## 运行 5：已选历史 episode 的只读导出与审计

历史 episode 沿用阶段 00 选择：`var/campaigns/panda_stack_v1/attempts/formal-10014-record-1794304035694470020-worker-03/`。只读导出到新目录：

```bash
runtime/ros2/scripts/run.sh export --scenario panda_stack \
  --episode-dir /data/campaigns/panda_stack_v1/attempts/formal-10014-record-1794304035694470020-worker-03/stack/record-1794304035694470020-worker-03/raw/formal-10014-record-1794304035694470020-worker-03 \
  --out /data/diagnostics/phase-12-20261112/exports/formal-10014-record-1794304035694470020-worker-03 \
  --runtime-log /data/campaigns/panda_stack_v1/attempts/formal-10014-record-1794304035694470020-worker-03/stack/record-1794304035694470020-worker-03/runs/record-1794304035694470020-worker-03/logs/runtime \
  --backend-log /data/record-pools/pool-1794303940600369712/worker-03/backend-runs/backend/log \
  --no-previews
```

- 导出 exit 0，`labels=470`、`quality_qualified=true`。
- 只读审计输出 `audit/formal-10014-audit.json`，通过；`message_counts` 与录制 manifest 完全一致（495 帧/路、529 feedback、48 references 等），48 个 applied chunks。旧轨迹字段 `ruckig_duration_ns` 按已知改名识别，使用 v1 摘要载荷校验并按当前摘要重建；旧反馈按固定数组布局解码，均未交给当前生成类型。
- 与既有导出比较：`summary.json`、`labels.json`、`raw_predictions.json`、`frame_index.json` 逐字节等价；`quality.json` 的 `qualified`、各阶段规则、每规则 `qualified_ratio/min_pixels/blackout` 全部一致。唯一差异是各相机/物体的 `samples` 计数 797 → 882：既有导出在运行期生成，当时 backend 日志尚未包含收尾写入的 `backend.render.quality` 事件；本轮读取完整日志。质量判定未变。
- 源 MCAP、既有导出与正式台账均未改动。

## 测试与检查

runtime 容器（`cd /workspace`，`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`，`-p no:cacheprovider`）：

```bash
python3 -m pytest -q tests/unit/test_mcap_feedback_guard.py tests/unit/test_legacy_feedback.py \
  tests/unit/test_offline_audit.py tests/unit/test_stack_export_build.py \
  tests/unit/test_stack_export_quality.py tests/unit/test_feedback_capability.py
# 31 passed
```

覆盖：legacy 布局识别、未知布局拒绝、legacy 导出不使用当前类型且调用专用解码、legacy 审计能力调整与 DTO 传递、既有导出/审计规则。ruff 对四个改动文件全部通过。阶段 06～11 已记录的单元测试/静态检查未受本轮改动影响，未重跑全量。

## 已知问题与未执行项

- Policy 本局失败与遮挡质量失败是既有模型/物理业务问题（阶段 00 已记录），本轮按实际结果保留，不通过放宽门禁或调整参数规避。一次闭环不能作为成功率或真机能力结论。
- runtime 全量单测中 `test_learning_console_progress.py` 的 3 个失败仍是 runtime 容器缺 `dual_arm_learning` 的环境问题；本轮未新增该类失败，也未混称回归。
- 历史导出的 `quality.json` 样本计数差异来自 backend 日志覆盖范围不同，分类/门禁与标签未变；未回写旧导出。
- 诊断方案目录 [`panda_stack_diag`](../../../../shared/scenarios/panda_stack_diag/) 保留用于复现，仅调整诊断输出、台账、种子与预算，不是第二个生产方案。
- 未运行完整训练、正式补采、完整评价集，未复制大型数据目录，未修改旧 MCAP/模型/prepared/正式台账/第三方资产。未 commit/push/stash/reset/clean。

## 资源收尾

三容器均为空闲运行状态，无 backend、runtime、collector、Policy 服务或 `50051` 监听；GPU 已释放，磁盘剩余 52 GiB。诊断结果（约 1.2 GB，含两次 Teacher 尝试、新 episode 导出/审计、中断现场、Policy prepared/raw/export/日志与历史导出/审计）保留在 `var/diagnostics/phase-12-20261112/`；两处 pool 目录与诊断台账作为运行证据保留。正式 `var/campaigns/panda_stack_v1`、`var/raw`、`var/learning` 与 `var/prepared` 未改动。

未进入阶段 13。
