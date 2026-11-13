# P06 交付：`var/<scenario-id>/` 四类产物目录规范化

任务书见 [phase-06-var-directory-standardization.md](../phase-06-var-directory-standardization.md)，总规范见 [`RULES.md`](../../../../RULES.md) 第二节。

## 结果

全仓运行期产物统一收敛到方案目录下的四个平级子目录（容器 `/data/<scenario-id>/`，宿主机 `var/<scenario-id>/`）：

```text
var/<scenario-id>/
├── datasets/      # LeRobot 数据集、manifest、filter 掩码、formal-select.json
├── models/        # 训练 runs/checkpoints、训练预算台账、bundles/<name>
├── logs/          # 三端事件日志、采集台账、录制内容、record-pools
└── diagnostics/   # 场景/运动学/交接检查、离线评估与测试产物
```

无方案选择的直接配置模式使用保留段 `var/_direct/`；与方案无关的环境探针使用 `var/_system/diagnostics/environment/`。`var/campaigns/`、`var/learning/`、`var/runs/`、`var/raw/`、`var/reports/`、`var/record-pools/` 等野生顶层目录不再由任何代码、脚本或配方创建。

## 路径映射

| 旧路径 | 新路径 |
| --- | --- |
| `var/campaigns/<campaign>/{attempts,attempts.jsonl,lock}` | `var/<scenario-id>/logs/campaign/...` |
| `var/campaigns/<campaign>/formal-select.json` | `var/<scenario-id>/datasets/formal-select.json` |
| `var/learning/<dataset>` | `var/<scenario-id>/datasets/<dataset>` |
| `var/learning/cache` | `var/<scenario-id>/datasets/.cache` |
| `var/learning/{runs,training-budget.jsonl}` | `var/<scenario-id>/models/{runs,training-budget.jsonl}` |
| `var/learning/bundles/<name>`、`/data/learning/.../bundles/<name>` | `var/<scenario-id>/models/bundles/<name>`、`/data/<scenario-id>/models/bundles/<name>` |
| `/data/runs/backend/<run-id>` | `/data/<scenario-id>/logs/backend/<run-id>` |
| `/data/raw/<run-id>`、runtime `/data/runs/<run-id>` | `/data/<scenario-id>/logs/{recordings,runs}/<run-id>` |
| `var/record-pools/pool-*` | `var/<scenario-id>/logs/record-pools/pool-*` |
| `/data/reports/sim-check/<run-id>` | `/data/<scenario-id>/diagnostics/<tool>/<run-id>` |
| `/data/logs/act-policy` | `/data/<scenario-id>/logs/policy-service` |
| `build/reports/evaluation` | `var/<scenario-id>/diagnostics/evaluation` |

## 实现要点

- 新增 runtime 内部路径单一信源 `dual_arm_runtime.artifact_paths`：`scenario_segment`、`logs_root`、`diagnostics_root`、`runs_root`、`recordings_root`、`default_backend_log`；所有 runtime CLI 默认值由它派生。
- `campaign.yaml` 新增 `paths.formal_select`，`campaign.py` 直接读取该字段，自动写出与 `campaign select` 使用同一位置，与 `act_dataset.yaml` 的 `select` 对齐。
- `runtime_cli`、`record.py`、`collect_attempt.py`、`evaluate.py`、`record_parallel.py`、`backend_cli.py`、`policy_service_cli.py` 的默认输出全部改为方案目录；`run.sh` 入口在缺少 `--output` 时注入 `logs/recordings` 与 `logs/runs`。
- 仿真 `run.sh` 的诊断子命令分别写入 `panda_stack/diagnostics/{scene,kinematics}` 与 `marvin_handover/diagnostics/{marvin,handover}`；`environment` 写入 `_system/diagnostics/environment`。
- 学习端 `convert` 的详细日志改写到 `<scenario>/logs/convert/`；训练 run 日志保留在 `models/runs` 内，与训练产物同属模型目录。
- `container.env` 不再用全局 `LEARNING_CACHE_ROOT` 覆盖配方缓存路径，缓存统一由 `datasets/.cache` 管理。
- 两个方案的 `binding.yaml`、`act_dataset.yaml`、`act_training.yaml`、`*_campaign.yaml` 与根/方案/三端 README 同步更新；测试夹具与集成测试证据目录一并迁移。

## 验证

| 检查 | 结果 |
| --- | --- |
| `ruff check --no-cache .`（runtime 容器） | All checks passed |
| 全仓单元测试（runtime 容器，`tests/unit`） | 959 passed, 26 skipped |
| 全仓集成测试（simulation 容器，`tests/integration`） | 93 passed, 2 skipped |
| Markdown 相对链接扫描（排除 `third_party/`、`build/`、`var/`） | 338 links, 0 dead link(s) |
| `scenario.py validate --binding shared/scenarios/panda_stack/binding.yaml --stage collection` | 通过 |
| 三端 `run.sh` shell 语法检查 | 通过 |
