# 总目标与当前阶段

更新：2026-11-10。本文件统一维护目标、约束、阶段与下一步。
运行方法见[根 README](../README.md)，维护规则见[项目规范](standards.md)。
本页不充当训练监控面板，运行事实以带时间的日志、台账和最终报告为准。

## 总目标

建立可理解、可修改、可复现的双臂机器人采集—训练—推理系统。通过统一 Robot
API 切换真机与仿真，通过独立 Policy API 切换模型；机器人、相机、场景和训练
机器可替换，避免 ROS、物理引擎与机器学习依赖互相绑定。

首个场景使用 MuJoCo 和双 Panda：两臂装在约 1 m 高的圆柱两侧，前方有桌子、
两个参与物理的方块、柱上全局相机和左右腕相机。右臂把小方块放到大方块上，
左臂暂时待机，但 ACT 同时输出双臂动作，后续再扩展协作。

当前要求在真实ACT推理下至少稳定堆叠成功一次，保留全部失败并定位原因；
本轮不以较高成功率为验收条件。可靠堆叠、双臂协作和真机迁移仍是后续目标。

## 已确认的边界

| 项目 | 当前约定 |
| --- | --- |
| 技术基线 | ROS 2 Jazzy；MuJoCo + Panda；固定的 LeRobot ACT；精确依赖由锁文件定义 |
| 学习输入/输出 | 三路 320×240 RGB + 双臂 16 维测量位置 → 60×16 位置动作块，间隔 50 ms |
| 示教与推理 | 示教可用物体真值；ACT 不读真值、不用脚本补救，左臂输出不被替换 |
| 数据集 | 50 条合格 episode，45 训练 / 5 验证；独立新种子评估，失败计入分母 |
| 本地计算 | RTX 4050 6 GB，训练与仿真分时；必要时换训练机器 |
| 原执行预算 | 最多 10 次 pilot、100 次正式尝试、30 GiB 原始记录、累计训练 4 小时；不清空台账重计 |
| 执行规则 | 连续接续、限速/限加速度跟随、明确 ACK 与实测反馈；原预测、参考和测量分开记录 |
| 夹爪 | 已批准接触完成与有时限卸载过渡；ACT 开度准入 0～0.08 m、单次修正最多 0.03 m；机械臂严格拒绝越界 |
| 成功判断 | 释放、退开后稳定支撑持续 2 秒；几何、速度和 30 秒任务期限由配置定义 |

## 已完成部分简记

| 阶段 | 已完成内容 | 结果边界 |
| --- | --- | --- |
| S1，DSH-001～013 | 配置、生命周期、日志、MCAP 审计、Compose 与操作入口 | CPU mock 验证软件链路 |
| S2，DSH-014～017 | 固定依赖/资产、双 Panda、三相机、可达性和关键位姿视野 | 场景检查与物理任务成功分开 |
| S3，DSH-018～032 | 16 维语义、轨迹接续、受限跟随、控制会话与公共接口 | 已接通预测、ACK、执行、反馈和新观测 |
| S4～S5，DSH-033～035 | 52 次正式尝试取得 50 条合格数据，独立 45/5 转换 | 训练 19676 帧、验证 2160 帧；失败保留 |
| S6～S7，DSH-035～036 | batch8 / 5000 步训练，20 个固定种子实际 ACT 闭环 | 堆叠 0/20；链路接通不等于学会任务 |
| DSH-037～039 | 重规划窗口、ACK 唤醒与离线运动诊断 | 单种子连续执行 289 块仍超时；完整预测块并非普遍静止 |
| DSH-040～041 | 筛选起点、batch16 / 20K 新训练完成 | 排除 6/19676 个起点；完整验证 L1=0.04347，新模型首次仿真见 DSH-044 |
| DSH-042 | 当前入口收敛、按编号归档、目录说明和链接迁移 | 历史内容保留；未改程序、配置或操作训练 |
| DSH-043 | 移除业务 COPY、提取 Dockerfile 程序、SDK 挂载校验 | 实际构建、源码修改对照和 CPU mock 采集/审计通过；未新增 GPU 推理 |
| DSH-044 | 20K 模型包、验证观察对照与单次真实仿真 | 显存足够，83 次预测、82 块执行；CHUNK_LATE 停车，堆叠 0/1 |
| DSH-045～051 | 推理窗口、预测融合、启动稳定与数值误判修复 | 30600同场景回归物理堆叠成功；腕部遮挡质量项仍保留 |
| DSH-052 | 按仿真、真机边界、运行和学习拆分目录，迁移配置/Docker/Compose/脚本 | 包名、IDL、profile/schema 和物理/控制参数不变；完整入口与回归见 DSH-052 |
| DSH-053 | 环境与业务执行分离、容器内具名脚本、bundle 驱动的准备流程 | 三端真实容器、SDK、NVIDIA 相机与模型加载验收通过；已由 054 收敛入口 |
| DSH-054 | 每端唯一 Dockerfile/docker-compose.yml/run.sh；三份独立依赖镜像与统一功能入口 | 实际构建通过，运行 98／仿真 122／学习 61 项通过；三路 NVIDIA 相机与原 20K 模型预测通过，旧 31 标签已清理 |
| DSH-055 | 精简仿真依赖、四个挂载、版本锁唯一来源、显式 SDK 与容器内入口 | 214 项目标回归及 33 项无 SDK 测试通过；公开场景/运动学、SIGINT 收尾与 init 停止验证通过 |
| DSH-056 | 仿真 Dockerfile 单阶段，直接安装固定 Ruckig wheel，清除编译路线 | 实际构建及 285 项目标回归通过，Ruckig 动态库字节不变；场景/相机/收尾正常 |
| DSH-057 | 仿真项目依赖统一为一份锁，沿用现成官方 ROS 环境 | 实际构建、65 项测试、相机场景与后端收尾通过；官方镜像自带工具保留 |
| DSH-058 | 删除自定义镜像版本报告层及三处检查入口的报告依赖 | 实际构建、105 项测试及环境/场景/运动学/后端收尾检查通过 |
| DSH-059 | Docker 内使用通用依赖文件名和本地构建上下文 | 实际构建、57 项测试及真实环境/三相机场景通过；不同宿主 cwd 解析一致 |
| DSH-060 | 容器资产变量通用化、删除四个冗余标记，运行 ENV 放在依赖安装后 | 实际构建、138 项测试及环境/场景/运动学/后端启停通过 |
| 阶段6重构 | 训练默认 PyAV + 默认张量批处理 + 多 worker/`pin_memory`/AMP；拆分学习与运行端千行模块 | 全仓 `.py` 均 ≤720 行、函数均 ≤85 行；学习 733 项、运行/仿真 691/784 项单测通过；全仓非测试总行数仍高于任务书的 35k 目标，见下文 |

阶段 6 重构（2026-11-10）把 ACT 训练切换为 PyAV 解码、DataLoader 默认
`default_collate` 张量批处理、多 worker、`pin_memory` 和 CUDA
`torch.amp.autocast`/`GradScaler`；训练配置新增 `num_workers`、`pin_memory`、
`prefetch_factor`、`use_amp`、`amp_dtype`。学习端拆分为
`dataset_common/dataset_export/dataset_manifest`、`sample_filter_rules/mask`、
`training_manifest/training_engine/training_support`；运行端拆分为
`episode_runner_io/episode_streams/episode_finalization`、`audit_stream/follower_audit`、
`runtime_callbacks/identity/limits`、`control_cli_support`，脚本拆分为
`evaluation_metrics/media`、`campaign_ledger/execution`、`export_quality/media`。
拆分后全仓 `.py` 均 ≤720 行、函数均 ≤85 行；但任务书 35,000 行总行数目标在
阶段 5 基线（43,540 行）时已不可达，本阶段拆分又增加了必要的模块脚手架，
当前约 46,200 行。未通过删除功能或压缩格式来伪造该指标。

详细计划、执行和结果已压缩为[历史总览](history/README.md)，
每个 DSH 编号保留一条关键结论；需要原文时通过 Git 历史追溯。

## 当前阶段

**已完成一次真实ACT稳定堆叠：同场景修复回归成功，泛化与图像质量仍有验收项。**

新训练于北京时间 2026-11-03 16:36 启动，batch16、20000 步、workers2、LR=1e-5，
从头训练；18:39 正常完成，完整 2160 帧验证 L1=0.0434682。新 best 模型包已独立
导出并通过真实观察往返重载和 20 个验证观察的输出/准入检查，旧 bundle 保留。

用户通知恢复后，[DSH-044](history/README.md#dsh-044) 按 1 次独立新种子 30300 验证：
83 次预测、82 块实际执行后，约 8.5 秒仿真时间触发 CHUNK_LATE 停车，堆叠 0/1。
本例右臂最大单关节活动范围约 6.1°，夹爪仍张开，动作推进不足仍未解决。
模型不是完整动作块普遍静止，近端目标、短窗口推进和调度余量仍需定位。
显存足够、无 OOM；录制、审计、图像质量和资源收尾正常，视频与失败证据已保存。
本轮预算已用完，未追加仿真或训练；一次失败不能代表跨种子成功率。

用户随后授权继续分析修复至至少一次成功。推理改为0.5s执行窗口、按绝对仿真时间
融合历史预测、任务前2s稳定等待，并修复夹爪8cm平均舍入、22飞秒Ruckig前缀
误拒绝和数值静止接触分类不一致。[DSH-048](history/README.md#dsh-048)记录全部探索。

最终30600同场景修复回归实际完成堆叠：任务25.85s、释放退开后稳定2s、54次真实
ACT预测、53块执行；无真值/teacher/回放补救。模型仍为原batch16/20K best，未新增训练。
这不是独立新种子的成功率评估；原失败保留，当前停止新增尝试。

用户批准的实验参考偏差预算为关节0.30rad/夹爪0.04m；硬限位、速度/加速度、实测
跟踪、接触条件、卸载2s和成功判定保持原值。通过的源配方为runtime/backend的
`*_act_chunked.yaml`与Policy的`policy_temporal.yaml`，已与成功输入做语义一致性核对。

录制、MCAP审计、数组/文件哈希与资源收尾通过；**图像质量门禁未全过**：右腕底块
曾遮挡1.85s。因此保留task_success=true、quality_qualified=false、run_success=false。
全局相机目标始终可见。所有本轮原始campaign约26.13GB，仍低于30GiB；未删除旧数据、
日志或缓存，未自动提交。

总览正文统一为根 README、本文件与规范文档；历史目录只保留压缩后的阶段结论和
DSH 索引，逐阶段原文通过 Git 历史追溯。功能目录当前只有组织说明；重要实践目录
已收录 ACT 推理执行修复经验，新条目继续遵守用户提出或同意收录的规则。
迁移范围、API 故障/恢复、审查修正与独立验收见[DSH-042](history/README.md#dsh-042)。

当前环境按 [DSH-060](history/README.md#dsh-060) 整理，文件组织保持每端唯一入口：

| 端 | 实现 | 唯一环境文件 | 唯一容器入口 |
| --- | --- | --- | --- |
| 仿真 | `simulation/mujoco/` | `docker/Dockerfile`、`docker/docker-compose.yml`、`configs/container.env` | 手动执行 `scripts/run.sh` |
| 运行 | `runtime/ros2/` | `docker/Dockerfile`、`docker/docker-compose.yml`、`configs/container.env` | 手动执行 `scripts/run.sh` |
| 学习 | `learning/lerobot/` | `docker/Dockerfile`、`docker/docker-compose.yml`、`configs/container.env` | 手动执行 `scripts/run.sh` |
| 真机 | `hardware/` | 尚未实现 | 尚未实现 |

三端在各自 `docker/` 目录直接用 `docker compose build/up -d/exec` 管理；业务在容器内由本端 run.sh
启动。SDK 不随 build 自动生成；ROS SDK 在 runtime 容器内显式生成，再只读挂载到仿真端。
三端镜像仍为 `sim-dual-arm/{simulation,runtime,learning}:current`。

仿真将 MuJoCo 项目挂到 `/app`，必要共享依赖与配置单独只读挂载，不带入运行端
和学习端源码；容器内执行 `scripts/run.sh`。Panda 模型放在
`simulation/mujoco/assets/franka_emika_panda/`，随 `/app:ro` 一起挂入，
不再下载到 `var/` 或生成资产摘要。Python 安装只有一份
`requirements.lock`：16 个固定版本及哈希包，含 Ruckig；Dockerfile 单 FROM、
单 COPY、一次 pip 安装。本目录 `context: .` 配合 `dockerfile: Dockerfile`，
COPY 使用本地相对路径，镜像内安装输入统一放 `/opt/dependencies`。
Ruckig 的约 1 MB wheel 随源码提供，来源和 MIT 许可保留，
适用于当前 Ubuntu 24.04/CPython 3.12/x86_64，不在日常构建中源码编译。

额外工具清单与重复安装步骤已移除；官方 ROS 镜像自带的工具与系统依赖原样保留，
不为包数量单独卸载。仿真入口不再维护测试或 SDK 构建命令，SDK 由运行端负责。
环境、三相机、运动学检查仍是直接可运行的功能。
构建直接由 pip 按唯一清单安装，不再生成镜像版本报告或进行运行时锁摘要比对。
环境检查直接验证模块导入、挂载源码及实际渲染/场景功能；数据录制仍由运行端负责。
Panda 模型现随仿真目录只读挂入，不使用资产环境变量或摘要文件。
Compose 不再维护无用组件标签、SDK 镜像/工作区标记或重复 NVIDIA 检查开关；直接
检查实际 GPU 配置。服务名和真实仓库路径仍负责多端编排与挂载定位。
DSH-060 实际构建、138 项相关测试及环境/场景/运动学/后端启停通过；新镜像不含
版本报告，Ruckig 二进制保持不变。未追加训练或完整任务 rollout。

旧 31 个本项目镜像标签和已退出的旧训练容器已清理，数据、模型、日志、缓存保留。
当前仅保留本地 `main`，用户成功阶段提交 `8ec25b7` 保留，重构未自动提交或推送。
旧 20K bundle 已删除，工作区当前只有新训练的 5K 模型；5K 模型仍可能出现动作偏差和
夹持跟踪超时，失败日志与录像必须保留。
目录迁移与执行方式的历史记录见 [052](history/README.md#dsh-052)、[053](history/README.md#dsh-053)、
[054](history/README.md#dsh-054)、[055](history/README.md#dsh-055)、[056](history/README.md#dsh-056)、
[057](history/README.md#dsh-057)、[058](history/README.md#dsh-058)、[059](history/README.md#dsh-059)、[060](history/README.md#dsh-060)，
操作步骤只按根 README 的现行入口执行。

## 主要产物与定位

以下路径相对于仓库根。运行产物不随 Git 提供，缺失时不能仅按文档数字补建。

| 路径 | 用途 |
| --- | --- |
| `var/campaigns/panda_stack_v1/` | 正式台账、原始记录、审计、RGB 与参考标签 |
| `var/learning/panda_stack_learning_v1/` | 已接受的 45/5 LeRobot 数据集 |
| `var/learning/runs/full-20261102T072749Z-1/` | 原 batch8 / 5000 步训练 |
| `var/learning/bundles/full-best/` | 原基线模型包，未被新作业替换 |
| `var/learning/evaluation-final/` | 原固定 20 次评估与录制预览 |
| `var/learning/stall-analysis-20261103/` | 离线策略运动诊断 |
| `var/learning/experiments/act-static-b16-20k-20261103/` | 新配置、filter、来源、预检、run、独立 `bundle-best-20k/` 与 `verification/` |
| `var/campaigns/panda_stack_act20k_20261103/` | 20K 首次仿真的单次追加式台账、原始录制与导出 |
| `var/campaigns/panda_stack_act20k_contact_regression_20261103/` | 30600成功修复回归的独立台账与完整原始证据 |
| `var/learning/experiments/act-static-b16-20k-20261103/inference-repair-20261103/success-30600/` | 成功视频、独立评估、数组与文件核验及质量限制 |
| `var/learning/training-budget.jsonl` | 新旧作业共用的追加式训练预算台账 |
| `var/runs/dsh/` | 原始调用、错误、任务输入与独立审查证据 |

新训练实际输入为实验目录的 `training-workers2.yaml`、`training-workers2.env`
和 `training-filter.json`，run 为 `runs/full-20261103T083633Z-1`。
`status.json` 是观察快照，实时进度读取 `metrics.jsonl`，结束读取
`training_report.json`。冻结源码 `.build/worktrees/dsh-041` 不能在整理文档或
清缓存时修改/删除。原数据、模型、失败记录与缓存均未自行删除。

## 下一步（尚未执行）

1. 固定已保留的推理配方，另定新种子与次数做独立评估，不能将同场景修复回归当泛化成功率。
2. 分析腕部目标遮挡及阶段化质量规则，保持本次质量失败记录；需要改变相机/数据分布时另作决定。
3. 日常环境使用三端 `:current` 镜像；进入所需实现的 `docker/` 目录执行
   `docker compose build`。build 只构建环境，SDK 通过运行容器的 `run.sh build-ros-sdk`
   / `build-policy-sdk` 显式生成。旧 review 标签只作为历史证据，不作为启动路线。
4. 真机、其他仿真器和其他学习框架待实际选型后接入；当前仅验证 MuJoCo / ROS 2 /
   LeRobot ACT。不得把目录分层当作其他适配器已实现。

本轮物理目标已达到，未继续采集或训练。后续修改与预算另行确定。
