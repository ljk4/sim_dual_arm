# runtime 命令与离线处理

所有 Python 业务入口均支持 `python3 -m dual_arm_runtime.offline.<模块名>`。`scripts/run.sh`、`record.sh`、宿主机 `record_parallel.sh` 保留原参数与操作入口。

- `campaign*`：预算、台账与筛选；`collect_*`：单次采集及产物收口。
- `record*`：串行录制、宿主机 Docker 编排、并行池及 worker 配置。
- `prepare_run`：运行配置准备；`evaluate`、`evaluation_*`：只读评估。
- `export_episode`、`export_media`、`export_quality`：MCAP 导出、图像及质量规则。
- `feedback_definition`：解码前识别记录内的 `ExecutionFeedback` 布局。导出与审计共用；安装布局放行，完整 0.1.0 旧布局指向 `legacy_feedback`，其他定义明确拒绝。
- `mcap_reader`：ROS/MCAP 解码为纯数据，按流分发；按需加载 ROS。
- `legacy_feedback`：0.1.0 固定 `[2]` 夹爪/手指数组的显式 CDR 读取器；要求 bag 内嵌定义与完整已知旧布局一致，用当前 `ExecutionFeedback` DTO 校验数值，不导入旧 SDK。
- [`audit/`](audit/README.md)：纯 Python 审计、标签与任务结果规则。
- `episode_finalization`：在线 episode 的清理和录制收尾编排；在审计规则包之外调用控制客户端和 recorder。

宿主机 `record_parallel` 仅使用标准库与 Docker；导入本包不会加载 ROS。配置根优先使用 `PROJECT_ROOT`，各 CLI 也保留已有的 `--project-root` 参数；源码运行默认定位仓库根。
