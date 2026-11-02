# DSH-016：三相机静态可见性已验收

2026-10-31。对应 [任务书](dsh-016-scene-visibility.md)。15 个允许文件已整合；主暂存区未变，未提交 Git。S2 的 IK、关键姿态和路径碰撞仍待下一任务。

## 实现与职责

- `rendering.py` 持有私有 MjData/EGL，通过不可变 full-physics 快照生成同一时刻三路 RGB/分割，不修改物理状态；校验原生 TIME，支持明确关闭。
- `visibility.py` 解析冻结的检查参数、生成初始布局、按 geom ID/类型统计像素；`check_scene.py` 编排日志、配置/源码/资产摘要、PNG、吞吐及最终报告。
- `scene_check.yaml` 定义 3 个工程种子及 64 个 XY/yaw 联合边界布局。固定腕相机外参调整后，每个布局两块都达到原定门槛；未缩小随机域。
- `tools/run/sim_check.py --check scene` 复用现有有界 Compose 工具容器，配置/源码/资产只读挂载，报告可写；默认 environment 入口保留。

## 实施、返工与独立验收

本地 headless dsh 首轮 00:30–00:46，86 步/97 次工具调用；唯一一次返工 00:49–00:52，24 步/24 次工具调用。session 为 `session-218c4e3e-bb64-4c44-b8b0-5887bf58d1fb`。dsh 当前已退出，不依赖用户打开 web。

首轮真实入口暴露逐相机重复创建 layout 目录的问题；独立原生探针另发现快照时间元数据不一致可被接受。返工修复目录创建和错误分类、原生时间校验、close 前事件措辞，并补真实 PNG/原生入口回归。主助手随后补充实际挂载 Python 源码清单与摘要，没有第二次 dsh 返工。

| 验证层次 | 最终结果 |
| --- | --- |
| 候选纯 Python | 411 项通过；Ruff check/format 通过 |
| 固定 MuJoCo 3.17/NVIDIA EGL | 50 项通过，无 skip |
| 实际 scene 产品入口 | exit 0；67/67 布局通过；201 张 320×240 PNG 和概览；日志排空、无遗留容器 |
| 高门槛、错误外参 | 均 exit 1，`visibility_threshold_not_met`，失败图像/原因保留 |
| logger close、renderer close 故障 | 均 exit 1，明确 close 原因，无提前完成事件 |
| 最终报告写入故障 | exit 1，stderr 明确 `report_write_failed`，无成功报告 |
| 原 environment 产品入口 | exit 0，无回归 |

独立使用原生 MuJoCo 重建全部种子/边界布局，402 项分割像素统计完全一致，配置与源码摘要匹配。201 张 RGB 中 200 张逐字节一致；一张的 4 个通道值相差 1/255，其他通道一致。严格逐字节审计原始结果保留为 false，不把它改写成全图完全相等；该微小跨渲染差异不影响分割或可见性判定。

人工已查看三视角概览。global、left_wrist、right_wrist 的 source 最小像素分别 162/50/43，target 为 332/123/99；门槛仍为 global 各 64、腕各 32。RGB-only 三视图一组 p50 约 1.84 ms、p95 约 2.18 ms，只表示纯渲染耗时，不表示物理/ROS/策略并发性能。

## 证据与限制

本地证据根 `var/runs/dsh/dsh-016/20261030T163020Z/`：`implementation/`、`rework/`、三次候选源码归档、`acceptance/product-final/`、`native-final/`、各故障场景及 `native-oracle-final/result.json`。早期失败未清除。首轮 usage 有缺失字段，完整 token 成本未知，不宣称具体节省比例。

本阶段没有运动、抓取或数据采集；固定初始相机可见性不代表运动全过程合格。下一任务通过 mink 检查关键位姿、路径采样净空及关键视角；接触抓取、搬运和稳定成功另在 S4 做真实物理验收。用户 ComfyUI 保留运行。
