# runtime 离线采集、导出、评估与审计

`dual_arm_runtime.offline` 按子包组织，与在线控制解耦：

- **`collection/`**（`python3 -m dual_arm_runtime.offline.collection.<模块名>`）：
  - `record`、`record_preflight`、`record_pool`、`record_workers`：单路串行录制 CLI、前置检查、并行采集池调度与隔离 worker 配置生成。
  - `campaign`、`campaign_ledger`、`campaign_execution`、`collect_attempt`：预算/种子与台账管理，达到正式目标时原子写出 `formal-select.json`，并按每局 `runtime_node --robot ... --task ...` 启动采样与清理。
  - `scenario_selection`：从 `<robot>/<task>` binding 解析并校验采集配方。
- **`export/`**（`python3 -m dual_arm_runtime.offline.export.<模块名>`）：
  - `export_episode`、`export_media`、`export_quality`：MCAP 导出、RGB 帧写入及画面/轨迹质量判定。
  - `mcap_reader`：ROS/MCAP 解码为纯数据并按流分发。
  - `follower_definition`：解码前校验记录内的 `FollowerReference` CDR 布局与当前定义一致。
- **`evaluation/`**（`python3 -m dual_arm_runtime.offline.evaluation.<模块名>`）：
  - `evaluate`、`evaluation_metrics`、`evaluation_media`：只读评估与结构化评估报告生成。
  - `inference_preflight`、`inference_worker`、`inference_workers`：并行推理前置检查、单局推理 worker 与隔离配置生成。
- **[`audit/`](audit/README.md)**：纯 Python（ROS-free）审计规则、标签生成与任务结果规则。
- **`parallel_common.py`**：采集与评估 worker 配置共享的公共路径映射与校验原语。

每次尝试的产物写入 `var/<robot>/<task>/datasets/raw/<purpose>-<seed>-<run_id>/`（`raw/` 与 `export/` 含 `labels.json`、`raw_predictions.json`、`quality.json`、`rgb/<frame>/<camera>.png`）；运行日志与 `console.log` 写入 `logs/collection/attempts/<attempt>/`。不合格或失败的 attempt 在写入台账后自动清理，`datasets/raw/` 仅保留合格 episode。

宿主机并行入口（`record_parallel.sh` / `inference_parallel.sh`）由 [`shared/common/tools/host/`](../../../../../../shared/common/tools/README.md) 维护。在线清理与录制收尾由上级 `episode_finalization.py` 编排；配置根优先使用 `PROJECT_ROOT`。

目录导航：[上级目录](../README.md)
