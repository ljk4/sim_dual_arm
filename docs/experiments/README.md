# 重要实践与经验

本目录按技术领域归档项目研发过程中经实测验证的故障排查方法、关键根因分析与工程修复经验，仅保留对后续开发、调优与问题定位有长期参考价值的核心结论与边界条件。

## 领域文档导航

| 领域文档 | 覆盖模块 | 核心内容摘要 |
| --- | --- | --- |
| [全链路闭环排查方法与验收总结](closed-loop-triage.md) | 全栈（`learning` / `runtime` / `simulation`） | 自底向上四步排查法（数据回放 → 模型离线输出 → 推理执行追踪 → 物理与评价器对齐）、跨领域故障根因速查表、双任务 80%（Panda 5/5、Marvin 3/5）闭环验收结果与后续优化方向 |
| [数据采集、并行录制与模型离线诊断](data-and-model-diagnostics.md) | `runtime`（录制）· `learning`（数据与模型） | 多容器隔离并行示教录制（4 路 3.2 倍吞吐）、Parquet/MCAP/Oracle Follower 100% 物理回放验证、阶段末等待静止帧（32%–47%）导致的单帧模型“近端等待、远端运动”分布特征、按轴有界输出准入（`max_prediction_corrections`）与绝对时间戳凸包融合 |
| [推理窗口调度、过渡重定时与轨迹跟随控制](inference-trajectory-execution.md) | `runtime` · `shared/trajectory` · `simulation/motion` | `target_count=20`（1.0 s）执行窗口与等待态尾部推进、仅限 Policy 的全 16 轴显式过渡重定时与非最终块可刹停边界（`_stop_bounds`）、左右臂跟随偏差预算对称化、Ruckig 零纳秒折叠与非零纳秒制动段导出、复位后 2.0 s 稳态等待与首次物理停止原因保留 |
| [仿真接触物理、夹持跟踪与任务评价器](simulation-physics-and-evaluators.md) | `simulation/physics` · `tasks` · `runtime/offline` | 自由物体 `condim=4` 扭转摩擦修复（消除搬运 90° 翻转）与轻物体禁用 `noslip_iterations`、`efc_address >= 0` 有效接触过滤、双指有效夹持预载豁免与零速度容差统一、堆叠局部系 `R.T @ delta` 投影、交接任务箱底 geom 支撑与四阶段空中交接状态机（含离桌参考位姿锁死与左手释放移交修复） |

## 维护准则

- **按领域收敛**：新增实验结论或故障定位经验时，直接合入上述对应领域的文档中，保持知识体系无重复、无冲突。
- **高信噪比**：只记录问题现象、支撑证据、真实根因、已排除假说与最终生效的修复方案；不堆砌日常调试流水、临时备份路径或已废弃的中间过程提示词。
- **可追溯性**：如需查阅早期单次实验原始记录或阶段任务书全文，可通过 `git log -- docs/experiments/` 在 Git 历史中检索。

目录导航：[文档索引](../README.md) · [功能说明](../features/README.md) · [项目历史](../history/README.md) · [Agent 指导与规范](../../AGENTS.md) · [项目说明](../../README.md)
