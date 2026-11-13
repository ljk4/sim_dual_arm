# runtime 命令与离线处理

所有 Python 业务入口均支持 `python3 -m dual_arm_runtime.offline.<模块名>`。`scripts/run.sh`、`record.sh`、宿主机 `record_parallel.sh` 保留原参数与操作入口；它们都直接接受 `--scenario <id>` 或显式 `--config`，不经过离线 `prepare`。

- `campaign*`：预算、台账与筛选；达到正式目标时自动原子写出 `formal-select.json`；`collect_*`：单次采集及产物收口。
- `record*`：串行录制、宿主机 Docker 编排、并行池及 worker 配置。
- `evaluate`、`evaluation_*`：只读评估。
- `export_episode`、`export_media`、`export_quality`：MCAP 导出、图像及质量规则。
- `scenario_selection`：从 binding 选择并校验采集配方；`collect_attempt` 直接以 `runtime_node --scenario ...` 启动每局。
- `mcap_reader`：ROS/MCAP 解码为纯数据，按流分发；按需加载 ROS。
- `follower_definition`：解码前校验记录内的 `FollowerReference` 布局与当前已安装定义一致。
- [`audit/`](audit/README.md)：纯 Python 审计、标签与任务结果规则。
- `episode_finalization`：在线 episode 的清理和录制收尾编排；在审计规则包之外调用控制客户端和 recorder。

每次尝试的产物按 `attempts/<purpose>-<seed>-<run_id>/` 拍平保存：录制落在 `raw/`，运行日志落在 `logs/`，导出落在 `export/`（`labels.json`、`raw_predictions.json`、`quality.json`、`rgb/<frame>/<camera>.png`，可选 `preview/` 与 `video/`），不再有 `stack/<run_id>` 子层或 attempt 顶层重复 JSON。

宿主机 `record_parallel` 仅使用标准库与 Docker；导入本包不会加载 ROS。配置根优先使用 `PROJECT_ROOT`，各 CLI 也保留已有的 `--project-root` 参数；源码运行默认定位仓库根。
