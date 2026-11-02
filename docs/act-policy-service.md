# 具名 ACT Policy 服务与固定种子评估

本页是 DSH-036 合入后的公共入口。服务复用 DSH-032 的 `PolicyProvider` /
`PolicyService` / `PolicyServer` 和 DSH-035 的 `load_bundle` /
`predict_action_chunk`，不新增 gRPC 协议、不复制 fake provider、不动态导入
任意类。正式入口只有 035 离线 bundle；没有 constant/replay/teacher/脚本补救
fallback。**不要用空 campaign 或新工作树台账重跑 pilot/test**。

## 0. 固定路径与预算检查

以下变量在合入后的验收 ROOT 下设置；`CAMPAIGN_CONFIG` 必须指向已有正式台账，
不能用默认模板或新目录重新分配种子：

```bash
ROOT=/home/ljk/work/sim_dual_arm
CAMPAIGN_CONFIG=$ROOT/var/campaigns/panda_stack_v1/campaign.yaml
DATA_ROOT=$ROOT/var/learning/panda_stack_learning_v1
BUNDLE=/path/to/frozen/act-bundle          # 仓库外只读；训练完成并复核后
SDK_ROOT=$ROOT/.build/generated/policy-sdk
ASSETS_ROOT=$ROOT/third_party/src/mujoco_menagerie
ASSET_SUMMARY=$ROOT/.build/assets/panda.json
LEARNING_VENV=$ROOT/.build/learning-dev-venv
LEARNING_SOURCE=$ROOT/third_party/src/lerobot-act-v0.6.1
LEROBOT_SRC=$LEARNING_SOURCE/src
LEARNING_CACHE=$ROOT/.build/cache
PROJECT_PYTHON=$LEARNING_VENV/bin/python

test -f "$CAMPAIGN_CONFIG" || { echo "missing existing campaign ledger: $CAMPAIGN_CONFIG" >&2; exit 2; }
findmnt -T "$ROOT"
findmnt -T "$DATA_ROOT"
findmnt -T "$BUNDLE" 2>/dev/null || true
df -hT "$ROOT" "$DATA_ROOT" "$ROOT/.build"
```

数据盘/源码/SDK/bundle/权重 cache 只读；只有 `$ROOT/.build/generated` 和本次
attempt 输出目录可写。不要读取/修改主 campaign 的历史记录或模型。

## 1. 启动 ACT 服务（真实模型）

```bash
LEARNING_SOURCE="$LEROBOT_SRC" $PROJECT_PYTHON $ROOT/tools/run/act_policy.py \
  --config $ROOT/configs/learning/act_policy.yaml \
  --bundle "$BUNDLE" \
  --profile $ROOT/profiles/robots/panda_stack_learning_v1/profile.json \
  --observation-schema $ROOT/profiles/observations/panda_rgb_joint_v1.json \
  --action-schema $ROOT/profiles/actions/panda_joint_position_v1.json \
  --device cuda \
  --log-directory $ROOT/var/logs/act-policy --run-id act-frozen-001
```

启动时从文件派生并校验：

1. `manifest.bundle_digest` 与完整文件清单（拒绝 symlink、pickle、增删改）；
2. `model.safetensors` / `config.json` / `checkpoint.config_sha256`；
3. bundle 内 `metadata/profile.json`、外部 shipped profile 的 semantic hash；
4. 外部 observation/action schema 的 canonical hash 与联合顺序；
5. `processors.digest`、`policy_preprocessor/postprocessor`、
   `train_stats_summary.json` 的 train-only 来源；
6. `metadata/dataset_manifest.json` 的 45/5、固定验证位置、逐 episode
   `source_facts`、`chunk_size=60`、`action_lead_ns=50ms`、`fps=20`；
7. ACT config 的 `chunk_size=60`、`n_obs_steps=1`、temporal ensemble 关闭。

`require_formal: true` 时 synthetic、profile.test_only、非 full checkpoint 直接
拒绝。检查完成后写 `resolved_contract.json`；`configuration.json` 和
`events.*.jsonl` 由共享 logger 写入。加载/暖启动失败时写
`startup_failure.json`（stage/error/bundle/contract 摘要），并返回非零，不会
监听端口。

服务在 `policy.ready` 之前完成 bundle 加载和一次 warm-up。warm-up 使用 profile
初始关节位置（不是全零），并严格检查 `(60,16)` 和 finite；失败不会 listen。
每个新 observation 都完整调用 `predict_action_chunk` 60×16，时间 50 ms …
3000 ms；不调用 `select_action`、不保留内部动作队列、不裁剪/投影/覆盖左臂。
RGB 解码为 HWC→CHW `float32 [0,1]`，train processor 只应用一次。输出非有限、
shape/名字/顺序/越界按现有 `PolicyService` 契约拒绝并记录。

## 2. Compose

`deploy/compose/native_act_policy.yaml` 只叠加
`native_backend.yaml + native_runtime.yaml`，不启动 fake-policy。固定挂载：

- `/workspace`：ROOT 源码 RO
- `/opt/policy-sdk`：生成 Policy SDK RO
- `/opt/act-bundle`：真实 bundle RO
- `/opt/host-learning-venv`：本机学习 venv RO
- `/opt/lerobot`：pinned LeRobot source RO
- `/opt/learning-cache`：权重 cache RO（只读、不写）
- `/data`：本次 attempt 日志/输出 RW；`LEARNING_CACHE_ROOT`/`HF_HOME` 指向
  `/data/act-cache`，不写只读权重盘

`act-policy` 使用 `sim-dual-arm/learning:act-dev` 入口（清除 ENTRYPOINT），健康
检查通过后才允许 runtime 启动。runtime 的 endpoint/model/processor digest 由
`stack_once.py` 从真实 bundle 写入生成配置，不手工散落假 digest。只启动/停止
本次 Compose project；外来资源保留并报告。

## 3. 模型冻结与 pilot

在完整训练结束前不消耗 ACT pilot/test 种子。真实模型导出后先离线严格复核
（bundle/profile/processor/stats/source-facts/60×16/有限与硬限位），再由监督者
按**已有** `$CAMPAIGN_CONFIG` 台账分配 pilot。不要硬猜 1008/1009：台账可能已
分配；命令只传 purpose，由 `stack_campaign.py` 选择未用预留 seed。

```bash
$PROJECT_PYTHON $ROOT/tools/run/stack_campaign.py \
  --project-root "$ROOT" --config "$CAMPAIGN_CONFIG" \
  run --purpose pilot --mode act --kind rollout \
  --runtime-config configs/runtime/native_task_act.yaml \
  -- $PROJECT_PYTHON $ROOT/tools/run/stack_once.py \
     --runtime-config configs/runtime/native_task_act.yaml \
     --policy-mode act --bundle "$BUNDLE" \
     --learning-venv "$LEARNING_VENV" \
     --learning-source "$LEARNING_SOURCE" \
     --learning-cache "$LEARNING_CACHE" \
     --policy-sdk "$SDK_ROOT" --assets-root "$ASSETS_ROOT" \
     --asset-summary "$ASSET_SUMMARY"
```

`stack_once.py` 会核对 `STACK_CAMPAIGN_MODE` 与实际来源/模式：ACT 预留必须
`--policy-mode act` + `--bundle`，不能通过 `auto` 无 bundle 或显式 `fake` 静默
变成 fake rollout。pilot 仍计入原总≤10 预算。生成配置在
`$ROOT/.build/generated/stack-inputs/<run-id>/`（容器内 RO）；attempt 输出写入
本次 campaign attempt 目录。任何完整 rollout 先有 RESERVED/RUNNING 记录，ACT
latency/显存和失败原因进入 attempt/ledger。

## 4. 冻结后固定 20 次 test

pilot 复核通过后**只冻结一个模型**，再按已有台账分配固定 `20000..20019` 的
未用 seed。`evaluation` 与 `test` 共用同一 seed 池和 20 次总上限，不能替换、
重跑挑最好或按 test 集选择 checkpoint。

```bash
$PROJECT_PYTHON $ROOT/tools/run/stack_campaign.py \
  --project-root "$ROOT" --config "$CAMPAIGN_CONFIG" \
  run --purpose evaluation --mode act --kind rollout \
  --runtime-config configs/runtime/native_task_act.yaml \
  -- $PROJECT_PYTHON $ROOT/tools/run/stack_once.py \
     --runtime-config configs/runtime/native_task_act.yaml \
     --policy-mode act --bundle "$BUNDLE" \
     --learning-venv "$LEARNING_VENV" \
     --learning-source "$LEARNING_SOURCE" \
     --learning-cache "$LEARNING_CACHE" \
     --policy-sdk "$SDK_ROOT" --assets-root "$ASSETS_ROOT" \
     --asset-summary "$ASSET_SUMMARY"
```

只读汇总（不改 ledger/attempt）：

```bash
$PROJECT_PYTHON $ROOT/tools/run/stack_evaluate.py \
  --project-root "$ROOT" --config "$CAMPAIGN_CONFIG" --purpose all \
  --output $ROOT/.build/reports/dsh-036/evaluation
```

报告分开列出 `task_success`（独立物理判定事实）、`act_executed`、`closed_loop`、
runtime/recording/launch/quality 和 `run_success`；固定 20 物理成功率
`success` 只由可信 ACT 证据 + 物理任务事实计算，质量另列。`task_result` 没有
`evaluated=true/test_only=false/action_source=policy/task` 冻结事实时不算物理
成功。时间分别报告 `first_started`（有确切 block 开始事实）、
`first_reference_applied`（有 raw prediction 目标事实时）和
`first_exported_label`（20Hz 标签采样，注明分辨率），以及来自
`task_start/completed/sample` 的仿真任务时长；执行片段和 wall 时间另列。
拒绝/超时/失败原样计入 20 分母，不把 fake/replay/无预测日志的 manifest 冒充
ACT 执行。

## 5. 组件验证命令

```bash
ROOT=/home/ljk/work/sim_dual_arm
LEROBOT_SRC=$ROOT/third_party/src/lerobot-act-v0.6.1/src
PYTHONPATH=$ROOT/.build/learning-test-tools:$ROOT/components/learning/src:\
$ROOT/libraries/run_logging/src:$ROOT/tests/unit:\
$ROOT/.build/generated/policy-sdk:$LEROBOT_SRC \
$ROOT/.build/learning-dev-venv/bin/python -m pytest -q \
  $ROOT/tests/unit/test_act_contract.py \
  $ROOT/tests/unit/test_act_provider.py \
  $ROOT/tests/unit/test_stack_evaluation.py \
  $ROOT/tests/unit/test_stack_once_act.py
```

测试使用临时 synthetic bundle/typed model；synthetic 永远不能通过
`require_formal: true`，不能作为正式服务或 20 次评估证据。

## 最终模型的夹爪准入（本次明确授权）

最终5000步模型在64个真实观察的原始60×16输出中仍有夹爪越界；用户随后批准左右夹爪按profile的0～0.08m范围投影，单次修正最多0.03m，超过拒绝，机械臂仍严格拒绝。配置为model.gripper_max_correction_m；0表示旧严格模式。此步骤不读取物体真值，不改机械臂预测，不改变模型权重/训练标签或follower的限速限加速度。

每次预测在act-policy-log/predictions保存独立NPZ，含raw_act/admitted/correction/accepted/reason；普通事件仅记录路径、数量与最大修正。MCAP及export/raw_predictions.json里的Policy targets指准入后、尚未插值的传输目标；原始神经网络输出以NPZ为准，执行参考与实测仍各自记录。模型digest保持原权重身份，准入参数由生成ACT配置hash和运行快照追溯。原始输出/准入值/执行参考/实测四层不能混用。
