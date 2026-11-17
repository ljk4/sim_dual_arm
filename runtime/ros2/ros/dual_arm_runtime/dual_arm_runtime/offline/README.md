# runtime 命令与离线处理

所有 Python 业务入口均支持 `python3 -m dual_arm_runtime.offline.<模块名>`。各端独立入口脚本（`record.sh`、`policy.sh`、`batch_policy.sh`、`interactive.sh` 及宿主机 `record_parallel.sh`）均直接接受 `--robot <robot> --task <task>` 或显式 `--config`，不经过离线 `prepare`。

- `campaign*`：预算、台账与筛选；达到正式目标时自动原子写出 `formal-select.json`；`collect_*`：单次采集及产物收口。
- `record*`：串行录制、宿主机 Docker 编排、并行池及 worker 配置。
- `evaluate`、`evaluation_*`：只读评估。
- `export_episode`、`export_media`、`export_quality`：MCAP 导出、图像及质量规则。
- `scenario_selection`：从 `<robot>/<task>` binding 选择并校验采集配方；`collect_attempt` 直接以 `runtime_node --robot ... --task ...` 启动每局。
- `mcap_reader`：ROS/MCAP 解码为纯数据，按流分发；按需加载 ROS。
- `follower_definition`：解码前校验记录内的 `FollowerReference` 布局与当前已安装定义一致。
- [`audit/`](audit/README.md)：纯 Python 审计、标签与任务结果规则。
- `episode_finalization`：在线 episode 的清理和录制收尾编排；在审计规则包之外调用控制客户端和 recorder。

每次尝试的产物按 `datasets/raw/<purpose>-<seed>-<run_id>/` 保存：录制落在 `raw/`，导出落在 `export/`（`labels.json`、`raw_predictions.json`、`quality.json`、`rgb/<frame>/<camera>.png`），不再有 `stack/<run_id>` 子层、attempt 顶层重复 JSON，也不生成 `preview/` 与 `video/`。运行日志与 `console.log` 位于 `logs/collection/attempts/<attempt>/`；不合格或失败的 attempt 在写入台账后自动删除，`datasets/raw/` 只保留合格 episode。

宿主机 `record_parallel` 仅使用标准库与 Docker；导入本包不会加载 ROS。配置根优先使用 `PROJECT_ROOT`，各 CLI 也保留已有的 `--project-root` 参数；源码运行默认定位仓库根。
