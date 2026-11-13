# P06：按 `RULES.md` 规范统一 `var/<scenario-id>/` 四类产物目录

总要求见 [README.md](README.md) 与仓库最高规范 [`RULES.md`](../../../RULES.md)，进度跟踪见 [progress.md](progress.md)。

## 背景与目标

根据最新写入 [`RULES.md`](../../../RULES.md) 第二节的强制规定：
- 所有运行期产物必须归入 `var/<scenario-id>/`（容器内 `/data/<scenario-id>/`）对应的任务目录下；
- 任务目录内部**严格划分为四个平级目录，绝对禁止混放**，也禁止在 `var/` 根下散落 `var/campaigns/`、`var/learning/`、`var/runs/`、`var/reports/`、`var/raw/`、`var/record-pools/` 等脱离任务目录的旧顶级目录：

```text
var/<scenario-id>/                  # 容器内为 /data/<scenario-id>/
├── datasets/                       # 1. 数据集目录（转换后的 LeRobot 数据集、数据集 manifest、filter 掩码、formal-select.json）
├── models/                         # 2. 模型目录（训练 runs/checkpoints、训练 budget 台账、导出的 bundles/）
├── logs/                           # 3. 运行日志目录（三端运行事件日志、采集台账 ledger.jsonl、以及运行时录制内容 attempts/<id>/{raw,logs,export} 或单局录制）
└── diagnostics/                    # 4. 诊断和测试产物目录（场景/运动学/交接检查报告、evaluate 评估报告与媒体、测试临时产物）
```
*(注：与具体任务无关的纯系统环境探针如 `environment` 可置于 `var/_system/diagnostics/`。)*

## 实施任务

### 1. 对齐方案配置文件（`shared/scenarios/{panda_stack,marvin_handover}/`）中的产物路径
检查并更新两个方案下的配方与 `binding.yaml`，将所有指向旧 `var/campaigns/...`、`var/learning/...`、`var/runs/...`、`/data/campaigns/...`、`/data/learning/...` 的路径统一收敛到 `var/<scenario-id>/{datasets,models,logs,diagnostics}/`（容器内 `/data/<scenario-id>/...`）：

1. **`configs/runtime/experiments/*_campaign.yaml`**：
   - 采集台账 `ledger` 与采集尝试根目录统一归入 `/data/<scenario-id>/logs/campaign/`（或 `/data/<scenario-id>/logs/` 下对应的采集子目录，包含 `ledger.jsonl` 与 `attempts/<attempt_id>/`）；
   - 正式数据筛选清单 `formal-select.json` 归入 `/data/<scenario-id>/datasets/formal-select.json`（或由 `campaign select` / 自动写出直接写到 `datasets/formal-select.json`）。
2. **`configs/learning/act_dataset.yaml`**：
   - `select`：`var/<scenario-id>/datasets/formal-select.json`
   - `output_root`：`var/<scenario-id>/datasets/act_dataset`（或 `var/<scenario-id>/datasets/<dataset_name>`）
   - `cache_root`：`var/<scenario-id>/datasets/.cache`
3. **`configs/learning/act_training.yaml`**：
   - `dataset_root`：指向 `var/<scenario-id>/datasets/...`
   - 训练输出 `output_root`（runs/checkpoints）与训练 `budget` 台账：统一归入 `var/<scenario-id>/models/`（如 `var/<scenario-id>/models/runs`、`var/<scenario-id>/models/budget.jsonl`）
4. **`binding.yaml`**：
   - `models.bundle`（若配置了路径）归入 `/data/<scenario-id>/models/bundles/<bundle-name>`。

### 2. 对齐三端脚本与 CLI 的默认输出路径
消除代码中硬编码的 `/data/runs`、`/data/raw`、`/data/logs`、`var/record-pools`、`build/reports/evaluation` 等旧野生路径：
1. **Simulation 端（`simulation/mujoco/scripts/run.sh`、`backend_cli.py`、`tools/`）**：
   - `backend`：当指定 `--scenario <id>` 时，运行快照与事件日志默认写入 `/data/<scenario-id>/logs/backend/<run-id>/`；
   - 诊断子命令（`scene`、`kinematics`、`marvin`、`handover`）：报告统一写入 `/data/<scenario-id>/diagnostics/<tool>/<run-id>/`；无方案的 `environment` 探针写入 `/data/_system/diagnostics/environment/<run-id>/`。
2. **Runtime 端（`runtime/ros2/scripts/run.sh`、`runtime_cli.py`、`offline/`）**：
   - `runtime_node` / `run.sh policy` / `run.sh interactive`：运行事件日志与运行时录制内容统一落入 `/data/<scenario-id>/logs/`（例如 `/data/<scenario-id>/logs/runs/<run-id>/` 与录制子目录），不再默认写到顶层 `/data/raw` 或 `/data/runs`；
   - `record.py` / `record_pool.py` / `record_parallel.py`：
     * 单路试采默认查找的 backend 日志路径改为 `/data/<scenario-id>/logs/backend/teacher-backend/log`；
     * 并行采集池的运行日志与 worker 录制统一放在 `var/<scenario-id>/logs/record-pools/pool-*/`（容器 `/data/<scenario-id>/logs/...`），不再写到顶层 `var/record-pools/`；
     * `formal` 达标自动生成的 `formal-select.json` 写出到 `<scenario-root>/datasets/formal-select.json`（与 `act_dataset.yaml` 的 `select` 对齐）；
   - `evaluate.py`：默认输出路径改为 `var/<scenario-id>/diagnostics/evaluation`（容器 `/data/<scenario-id>/diagnostics/evaluation`）。
3. **Learning 端（`learning/lerobot/scripts/run.sh`、`policy_service_cli.py`、`cli.py`）**：
   - `policy` 服务日志默认写入 `/data/<scenario-id>/logs/policy-service/`；
   - `bundle` 导出推荐/默认归入 `/data/<scenario-id>/models/bundles/<name>`。

### 3. 同步更新文档与测试
- 更新根 [`README.md`](../../../README.md)、[`shared/scenarios/panda_stack/README.md`](../../../shared/scenarios/panda_stack/README.md)、[`shared/scenarios/marvin_handover/README.md`](../../../shared/scenarios/marvin_handover/README.md)、[`shared/scenarios/_template/README.md`](../../../shared/scenarios/_template/README.md) 及三端 `scripts/README.md` 中的所有 `/data/...` 与 `var/...` 路径示例，使其严格体现 `var/<scenario-id>/{datasets,models,logs,diagnostics}/` 四分法。
- 清理或重置 `var/` 下残留的旧野生目录结构，运行双方案冒烟与全套测试验证新目录结构。

## 完成条件与交付

- 全仓代码、脚本、配方与文档中的所有运行期产物路径均严格落在 `var/<scenario-id>/{datasets,models,logs,diagnostics}/`（及纯环境探针 `var/_system/diagnostics/`），零混放、零野生顶层目录。
- 全仓 Markdown 链接扫描 `0 dead link(s)`，`ruff check` 零警告，三端单元测试与集成测试全部通过。
- 更新 [progress.md](progress.md) 中 P06 状态为“已完成”，并在 `deliveries/phase-06.md` 记录路径映射与验证结果后停止。

目录导航：[总要求](README.md) · [P00～P05 进度](progress.md) · [进度索引](progress.md)
