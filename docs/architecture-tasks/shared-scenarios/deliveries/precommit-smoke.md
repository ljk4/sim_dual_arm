# 提交前小闭环：真实小样本转换、训练、bundle 与推理

- 日期：2026-11-12（Asia/Shanghai）。结果：部分完成。
- 依据：总要求、阶段 12/13 交接与本轮“真实小样本闭环 + 最终审计”要求。未 commit/push/stash/reset/clean。
- 验收口径：不追求短训练模型完成任务；软件链路、模型来源、门禁、错误处理和资源收尾按实际结果验收。原始“观测 → Predict → 后端请求 → 反馈 → 会话关闭”口径下，本轮未产生执行反馈，闭环未完整。

## 结果

真实阶段 12 Teacher episode → 独立训练/验证 LeRobot 数据集 → 128 步 ACT 优化 → checkpoint → 新 bundle → `prepare-policy` / simulation `prepare` / runtime `prepare` → 仿真后端与 Policy 服务 → 一局有界推理均已实际运行。

推理局完成了观测输入、`Predict`、`follower.submit_requested` 与 `CloseSession`。首个动作块被既有跟随偏差门禁以 `FOLLOWER_DEVIATION_EXCEEDED: axis 'right_joint2' deviation 0.3624944226264954 exceeds 0.3` 拒绝，因此没有执行反馈，任务是受控失败而非异常崩溃。Policy、backend 和 runtime 进程均正常收尾，无残留进程、端口或未释放锁。

## 数据与产物

诊断根：`var/diagnostics/precommit-smoke-20261112/`（容器 `/data/diagnostics/precommit-smoke-20261112/`）。正式数据、正式 50/50 台账、`var/learning/bundles/stack-act` 与正式训练预算未改。

使用的真实 episode（阶段 12 已采集并导出，种子与目录均不重叠）：

| 划分 | 种子 | 源 attempt | 帧数 |
| --- | --- | --- | --- |
| training | 900000 | `pilot-900000-pool-1794455130527176076-worker-01-1794455136621192877` | 480 |
| validation | 900001 | `pilot-900001-pool-1794455130527176076-worker-02-1794455136665468889` | 480 |

- 转换清单：`var/diagnostics/precommit-smoke-20261112/select.json`，`qualified=2`，训练/验证各 1 条，验证位置 `[1]`。
- 数据集：`dataset/`，`synthetic=false`、`diagnostic=true`；训练/验证各 480 帧，3 路 320×240 RGB，动作 `(60,16)`，观测/动作时钟严格相差 50 ms、周期 50 ms；归一化统计只来自训练集。
- 训练：`runs/smoke-20261112T045917Z-23281/`，128/128 step，`loss` 235.79 → 168.16，`final.pt`/`latest.pt` 可读取，无 NaN、无有限性错误。
- 最终 bundle：`bundle-128/`，digest `5d674979ba7c3e6d9af3c3d982ee5672854102e52ebe88a969289bf5b21c052b`，checkpoint step 128、mode `smoke`、seed 910002，dataset `1train/1validation`、`diagnostic=true`；模型 SHA `88d7ac2a…`，processor digest `cf1e67e5…`。
- 首轮 48 步 bundle：`bundle/`，digest `199c78a3…`，保留为训练步数对照证据，未用于最终推理。

## 实际命令

诊断配置新增在 [`shared/scenarios/panda_stack_diag/configs/`](../../../../shared/scenarios/panda_stack_diag/configs/)：

- `precommit_dataset.container.yaml`：`diagnostic: true`，`expected 2/1/1`，验证位置 `[1]`，输出与缓存写入诊断根。
- `precommit_training.container.yaml`：smoke 128 step、batch 8、2 workers；独立 budget ledger、checkpoint 与日志根；正式配方未改。

learning 容器：

```bash
learning/lerobot/scripts/run.sh convert \
  --config shared/scenarios/panda_stack_diag/configs/precommit_dataset.container.yaml
learning/lerobot/scripts/run.sh validate \
  --config shared/scenarios/panda_stack_diag/configs/precommit_training.container.yaml
learning/lerobot/scripts/run.sh train \
  --config shared/scenarios/panda_stack_diag/configs/precommit_training.container.yaml \
  --mode smoke --max-steps 128
learning/lerobot/scripts/run.sh bundle \
  --config shared/scenarios/panda_stack_diag/configs/precommit_training.container.yaml \
  --mode smoke \
  --run /data/diagnostics/precommit-smoke-20261112/runs/smoke-20261112T045917Z-23281 \
  --checkpoint final.pt \
  --output /data/diagnostics/precommit-smoke-20261112/bundle-128
learning/lerobot/scripts/run.sh verify-bundle \
  --bundle /data/diagnostics/precommit-smoke-20261112/bundle-128 --device cpu
learning/lerobot/scripts/run.sh prepare-policy --scenario panda_stack --shared-root /shared \
  --bundle /data/diagnostics/precommit-smoke-20261112/bundle-128 --run-id precommit-smoke-128 \
  --output /data/diagnostics/precommit-smoke-20261112/policy/prepared-128 --no-require-formal
```

simulation 容器：

```bash
scripts/run.sh prepare --scenario panda_stack --shared-root /shared --role policy \
  --bundle /data/diagnostics/precommit-smoke-20261112/bundle-128 --seed 20001 \
  --run-id precommit-smoke-128 \
  --output /data/diagnostics/precommit-smoke-20261112/simulation-128
```

runtime 容器：

```bash
runtime/ros2/scripts/run.sh prepare --scenario panda_stack --shared-root /shared --role policy \
  --policy-binding /data/diagnostics/precommit-smoke-20261112/policy/prepared-128/policy_binding.json \
  --bundle /data/diagnostics/precommit-smoke-20261112/bundle-128 --seed 20001 \
  --run-id precommit-smoke-128 \
  --output /data/diagnostics/precommit-smoke-20261112/runtime-128
```

服务与一局推理：

```bash
# learning
learning/lerobot/scripts/run.sh policy \
  --config /data/diagnostics/precommit-smoke-20261112/policy/prepared-128/policy.yaml \
  --run-id precommit-smoke-128 \
  --log-directory /data/diagnostics/precommit-smoke-20261112/policy/logs-128-<timestamp>

# simulation
scripts/run.sh backend \
  --config /data/diagnostics/precommit-smoke-20261112/simulation-128/backend.yaml \
  --run-root /data/diagnostics/precommit-smoke-20261112/backend-runs-128 \
  --run-id precommit-smoke-128

# runtime，墙钟上界 240 s
timeout --signal=INT 240 runtime/ros2/scripts/run.sh policy \
  --config /data/diagnostics/precommit-smoke-20261112/runtime-128/runtime.yaml \
  --output /data/diagnostics/precommit-smoke-20261112/raw-128 \
  --run-id precommit-smoke-128-ep1 --episode-id precommit-smoke-128-ep1 \
  --reset-seed 20001 \
  --runs-output /data/diagnostics/precommit-smoke-20261112/runs-128
```

## 推理实际加载与结果

- Policy 服务从 `prepared-128/policy.yaml` 加载 `bundle-128`，`resolved_contract.json` 记录的 bundle digest 为 `5d674979…`、checkpoint step 128、mode `smoke`、dataset `1/1`、`diagnostic=true`、device `cuda`。
- runtime 的 `policy.info` 校验 `model_bundle_digest=5d674979…`、`processor_digest=cf1e67e5…`，与 `policy_binding.json`、`runtime.yaml` 和 bundle manifest 一致。
- RPC：`GetPolicyInfo`、`OpenSession`、`Predict`、`CloseSession` 各 1 次，全部 `OK`；`Predict` 返回完整 60×16 动作块。
- 后端：收到 1 次 `follower.submit_requested`；`backend.submit_follower.rejected` 为 `FOLLOWER_DEVIATION_EXCEEDED`，`right_joint2` 偏差 0.3625 > 0.3。没有产生执行反馈。
- 录制：`raw-128/precommit-smoke-128-ep1/`，runtime 记录 `complete=true`；三路相机、state、task_state 与 command 均写入 MCAP，`message_counts` 与 manifest 一致。
- 进程：runtime 退出码 1、`process_outcome=failed`、日志正常关闭；Policy 与 backend `summary.json` 均为 `complete`、0 dropped；服务 SIGTERM 后 50051 端口释放。

## 正式准入仍拒绝诊断模型

同一 bundle 不传 `--no-require-formal` 时，`prepare-policy` 以 `formal ACT deployment requires a full-mode checkpoint` 拒绝，exit 1，未创建输出目录。数据配置的非 `diagnostic` 路径仍要求 50/45/5 与固定验证位置；bundle/dataset 的 `diagnostic` 证据在 `require_formal=true` 时被拒绝。

## 清理、测试与审计

- 新增/修改源码：`learning/lerobot/src/dual_arm_learning/{config,dataset,dataset_manifest,training_manifest,training_support,bundle,policy_config,cli}.py`。`diagnostic` 是显式配置字段；非诊断配置仍强制 50/45/5；bundle、训练报告与数据清单记录实际数量和诊断标记。
- 复核旧入口：仓库内无 `contracts/`、`libraries/`、`infrastructure/`、`profiles/` 旧根目录或兼容软链接；活跃文件无旧根调用。`collect_artifacts` 保留对历史 `contracts/` 产物前缀的兼容读取。
- 受影响测试（learning 容器）：`test_learning_config`、`test_learning_training_manifest`、`test_policy_config`、`test_policy_contract`、`test_learning_dataset`、`test_learning_validation`、`test_learning_scenario`、`test_learning_bundle_cli`，共 47 passed；最终文案调整后复测其中 19 passed。改动文件 ruff 全部通过。测试覆盖真实小样本清单、诊断 bundle 的非正式准入，以及正式模式对非 50/45/5 数据/模型和 `diagnostic`/`synthetic` 证据的拒绝。
- `var/`、`.build/`、模型权重、数据集和临时缓存均被忽略，未进入 git 状态；`shared/` 新增源码、配置与旧目录删除配套。未自动暂存或提交。

## 已知业务问题

- 128 步 smoke 模型仍未学会从当前观测输出可执行的首个动作块；其首块接近训练集均值，右侧 `right_joint2` 参考偏差 0.3625 超过既有 0.3 预算。48 步模型的首块更远，先被 Ruckig 3 s 时限拒绝。两者都是短训练/模型质量问题，不是本轮迁移或推理软件缺陷；未放宽跟随、Ruckig、控制或质量门禁。
- 本轮没有执行反馈，不能据此评价堆叠成功率、真机能力或跨种子泛化。Policy 服务与 runtime 的协议、模型来源校验、会话关闭和资源收尾已实际通过。
