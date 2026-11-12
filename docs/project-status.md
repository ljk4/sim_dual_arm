# 项目状态与里程碑

更新日期：2026-11-12。本文件记录项目总体目标、核心技术约束、已完成阶段、关键产物路径与下一步计划。操作指南见[根 README](../README.md)，工程准则见[项目规范](standards.md)。

---

## 1. 总体目标

构建结构清晰、解耦可测、可复现的双臂机器人“数据采集—模型训练—策略推理”闭环系统：

- **统一接口解耦**：通过 ROS 2 Robot API 隔离仿真与真机硬件，通过 gRPC Policy API 隔离运行端与深度学习框架，避免 ROS、物理引擎与 PyTorch 环境互相耦合。
- **基准验证场景**：采用 MuJoCo 双 Panda 机械臂堆叠场景（双臂安装于约 1 m 高立柱两侧，前方桌面放置一大一小两个自由方块，配置立柱全局相机与左右腕部相机）。右臂将小方块抓取并堆叠至大方块上方，左臂保持待机，ACT 策略同时输出双臂 16 维动作。
- **阶段验收标准**：在纯视觉与关节状态输入下（不读取物体真值、不借助示教或 IK 补救），实现真实 ACT 策略闭环稳定堆叠，并完整保留所有成功与失败的审计证据。

---

## 2. 核心技术基线

| 维度 | 当前基线约定 |
| --- | --- |
| 软件与环境 | ROS 2 Jazzy；MuJoCo + Franka Emika Panda（Menagerie 固定资产）；LeRobot ACT；三端独立 Docker 容器 |
| 策略输入与输出 | 三路 320×240 RGB 图像 + 16 维实测关节/夹爪位置 → 60×16 位置动作块（步长 50 ms，总时域 3.0 s） |
| 示教与评估隔离 | 仅示教采集阶段可使用物体真值；ACT 评估阶段严禁读取真值或替换模型双臂输出 |
| 数据集规模 | 50 条合格正式 episode，按 45 训练 / 5 验证划分（训练集 19,676 帧，验证集 2,160 帧） |
| 运动控制与跟随 | Ruckig 有界跟随规划（`SubmitFollower`），0.5 s（10 点）执行窗口，绝对仿真时间指数加权融合（系数 0.01） |
| 夹爪与接触处理 | 准入范围 0～0.08 m（单次最大修正 0.03 m）；支持双指接触完成判定与 2.0 s 有时限接触卸载过渡 |
| 任务成功判定 | 小方块释放且机械臂退开后，在大方块上稳定支撑持续 2.0 s（任务总时限 30.0 s） |

---

## 3. 已完成阶段总览

完整历史演进见[项目历史](history/README.md)，当前架构任务与迁移验收见[架构任务书](architecture-tasks/README.md)。

| 阶段 | 核心进展 | 验证结果与边界 |
| --- | --- | --- |
| **S1**（DSH-001～013） | 建立配置校验、控制会话、结构化日志、有界 MCAP 录制与流式审计框架 | 打通基础软件控制与审计闭环 |
| **S2**（DSH-014～017） | 固定 MuJoCo 依赖与 Panda 资产，构建双臂堆叠场景、三路相机可见性与运动学净空检查 | 独立完成静态视野与关键位姿净空验证 |
| **S3**（DSH-018～032） | 实现 16 维 SI 执行器映射、Ruckig 有界跟随规划、连续动作块接续与 gRPC Policy 闭环 | 打通 `Policy -> Runtime -> FollowerReference -> MuJoCo` 全链路 |
| **S4～S5**（DSH-033～035） | 实现堆叠任务状态机、程序示教、质量门禁、45/5 LeRobot 数据集转换与模型包导出 | 52 次正式尝试采集 50 条合格 episode，完成离线数据集构建 |
| **S6～S7**（DSH-036～051） | 完成 20K 步 ACT 训练，排查并修复短执行窗口停滞、预测切换跳变、启动非稳态及浮点/静止容差问题 | 在种子 30600 同场景修复回归中完成真实 ACT 物理堆叠（耗时 25.85 s，54 次预测） |
| **S8**（DSH-052～060） | 拆分为 `simulation`/`runtime`/`learning`/`hardware` 四端目录，统一容器空闲启动与 `scripts/run.sh` 入口 | 三端独立构建、SDK 显式生成、挂载边界与硬件渲染全部验收通过 |
| **架构重构**（Phase 1～6） | 清除早期死代码与废弃接口、统一单层 Pydantic v2 模型、下沉轨迹数学、解耦任务插件、提速训练并拆分大文件 | 较重构前净减约 3.4 万行代码；全仓 `.py` 均 ≤ 720 行，核心函数均 ≤ 85 行，三端单测全部通过 |

---

## 4. 当前状态说明

1. **物理闭环与推理状态**：
   - 通过引入 **10 点（0.5 s）执行窗口**、**绝对仿真时间预测融合**、**任务前 2.0 s 稳定等待** 以及 **Ruckig 零纳秒前缀折叠与 `1e-9 m/s` 静止速度容差统一**，20K 步模型已在种子 `30600` 的同场景修复回归中完成真实物理堆叠（详细分析见 [ACT 推理修复经验](experiments/2026-11-03-act-inference-execution-repair.md)）。
   - 该次成功中物理任务判定 `task_success=true`，但因右腕相机视角下底块存在 1.85 s 遮挡（超出 0.35 s 阈值），图像质量门禁记为 `quality_qualified=false`。
   - 当前工作区保留用于快速验证链路的 5K 步模型包；5K 模型可跑通完整推理闭环，但在部分种子下仍会出现跟随偏差超限（`FOLLOWER_DEVIATION_EXCEEDED`）或夹持阶段跟踪超时（`TRACKING_TIMEOUT`）。
   - 2026-11-11 复核最近 8 局并修复制动前缀导出、夹持接触误判和停止原因掩盖。初次两局完整回归均录制完整、未复现这些工程错误，但均因方块掉落失败。证据与边界见[本次排查记录](experiments/2026-11-11-inference-execution-failures.md)。
   - 后续同种子 `20001` 的用户运行出现一次真实 `TRACKING_TIMEOUT` 和一次任务成功（成功条件持续 2 s；未做导出图像质量门禁）。数据逐行检查未发现标签转换错误；新增诊断发现融合关闭延迟、跟随参考桌面净空不足及模型过渡误差，仍需分开验证，不能视为稳定成功。
   - 2026-11-12 阶段 12 有界回归使用现有真实 bundle 完成 Policy 软件链路（45 次 `Predict`、44 个动作块、审计通过）；本局任务结果为 `TASK_FAILED: DROPPED`，导出质量因既有遮挡不合格。结果按实际记录，未调整模型、物理参数或质量门禁。
2. **代码与工程架构状态**：
   - 公共层与方案绑定迁移已完成：公共协议、Python 包与工具在 `shared/common/`；当前 Panda/MuJoCo/ROS 2/ACT 实例的档案、配方和专用接口由 `shared/scenarios/panda_stack/binding.yaml` 选择；四端持有实现与容器入口。替换步骤见 [方案模板](../shared/scenarios/_template/README.md)，诊断复现见 `shared/scenarios/panda_stack_diag/`。
   - 阶段 12 有界回归同时验证了双 worker Teacher（各一次、domain 210/211）、新 episode 导出/审计、并行入口中断收尾，以及已选历史 episode 的只读导出/审计；历史导出与既有结果在 summary/labels/raw_predictions/frame_index 上逐字节一致，quality 样本计数差异来自日志覆盖范围，判定未变。
   - 六阶段全量重构已完成：废弃的 S1 Mock 后端、非受限 SciPy 样条接口、旧版 Schema（v2～v4）及 `Native*` 命名前缀已彻底清理。
   - 训练端已默认切换为 `PyAV` 视频解码、`default_collate` 批张量直出、多进程 `num_workers`/`pin_memory` 与 CUDA `torch.amp`（bfloat16）混合精度训练。
   - 运行端支持 `interactive` 交互式单局评估，仿真端支持独立进程只读 `viewer` 实时可视化。

---

## 5. 关键产物路径索引

以下路径均相对于仓库根目录（位于 `var/` 下，不纳入 Git 版本控制）：

| 路径 | 内容说明 |
| --- | --- |
| `var/campaigns/panda_stack_v1/` | 正式示教采集台账（`attempts.jsonl`）、划分清单（`formal-select.json`）与原始录制 |
| `var/learning/panda_stack_learning_v1/` | 转换并校验通过的 45/5 LeRobot 训练与验证数据集 |
| `var/learning/bundles/stack-act/` | 当前工作区导出的可部署 ACT 模型包（safetensors 格式） |
| `var/learning/experiments/act-static-b16-20k-20261103/` | 20K 步训练实验配置、起点筛选掩码与 `inference-repair-20261103/success-30600/` 成功堆叠证据 |
| `var/learning/training-budget.jsonl` | 追加式训练时间预算台账 |
| `var/prepared/<run-id>/` | 各端 `prepare` 生成的运行配置快照（`simulation/`、`policy/`、`runtime/`） |
| `var/raw/<run-id>/` | 单次运行或 `interactive` 评估生成的 MCAP 原始录制与 `episode_manifest.json` |
| `var/runs/`、`var/logs/` | 运行端、仿真后端与 Policy 服务的结构化事件日志与汇总报告 |

---

## 6. 下一步计划

1. **跨种子泛化评估**：固定当前验证通过的流式执行与时间融合配方，在独立的新随机种子集合上评估完整训练模型的泛化成功率。
2. **腕部相机视野与质量门禁优化**：针对右腕相机在接近与堆叠阶段的底块遮挡问题，评估阶段化可见性规则或相机位姿调整方案。
3. **双臂协作与真机接口扩展**：在单臂堆叠稳定后扩展双臂协同任务，并在确认真机硬件与安全边界后于 `hardware/` 下实现对应的 Robot API 驱动。
