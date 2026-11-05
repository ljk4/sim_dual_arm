# 历史材料索引

每个 DSH 编号一个文件，合并该阶段的计划、执行、返工、结果和归属专题；原始正文按来源分块保存并记录 SHA256，块首锚点可用于旧链接跳转。历史命令、路径、预算、版本和外部来源只反映当时状态，不保证今天可直接执行；当前入口见 [文档索引](../README.md) 和 [项目状态](../project-status.md)。

<a id="timeline"></a>
## 时间线

原 `docs/simulation-loop-progress.md` 共 531 行，已按清单给定的 41 个区间拆入 000～041 各阶段；区间首尾相接、互不重叠、无遗漏。旧文档中指向整份时间线的链接统一转到本节。每个阶段内的原文见该文档“原始时间线”。

| 原行区间 | 归档阶段 |
| --- | --- |
| 1～15 | [DSH-000](dsh-000.md#timeline-0001-0015) |
| 16～22 | [DSH-004](dsh-004.md#timeline-0016-0022) |
| 23～35 | [DSH-005](dsh-005.md#timeline-0023-0035) |
| 36～47 | [DSH-006](dsh-006.md#timeline-0036-0047) |
| 48～59 | [DSH-007](dsh-007.md#timeline-0048-0059) |
| 60～71 | [DSH-008](dsh-008.md#timeline-0060-0071) |
| 72～84 | [DSH-009](dsh-009.md#timeline-0072-0084) |
| 85～97 | [DSH-010](dsh-010.md#timeline-0085-0097) |
| 98～110 | [DSH-011](dsh-011.md#timeline-0098-0110) |
| 111～122 | [DSH-012](dsh-012.md#timeline-0111-0122) |
| 123～136 | [DSH-013](dsh-013.md#timeline-0123-0136) |
| 137～153 | [DSH-014](dsh-014.md#timeline-0137-0153) |
| 154～172 | [DSH-015](dsh-015.md#timeline-0154-0172) |
| 173～187 | [DSH-016](dsh-016.md#timeline-0173-0187) |
| 188～207 | [DSH-017](dsh-017.md#timeline-0188-0207) |
| 208～226 | [DSH-018](dsh-018.md#timeline-0208-0226) |
| 227～238 | [DSH-019](dsh-019.md#timeline-0227-0238) |
| 239～249 | [DSH-020](dsh-020.md#timeline-0239-0249) |
| 250～266 | [DSH-021](dsh-021.md#timeline-0250-0266) |
| 267～275 | [DSH-022](dsh-022.md#timeline-0267-0275) |
| 276～288 | [DSH-023](dsh-023.md#timeline-0276-0288) |
| 289～300 | [DSH-024](dsh-024.md#timeline-0289-0300) |
| 301～314 | [DSH-025](dsh-025.md#timeline-0301-0314) |
| 315～327 | [DSH-026](dsh-026.md#timeline-0315-0327) |
| 328～343 | [DSH-027](dsh-027.md#timeline-0328-0343) |
| 344～350 | [DSH-028](dsh-028.md#timeline-0344-0350) |
| 351～356 | [DSH-029](dsh-029.md#timeline-0351-0356) |
| 357～365 | [DSH-028](dsh-028.md#timeline-0357-0365) |
| 366～371 | [DSH-029](dsh-029.md#timeline-0366-0371) |
| 372～377 | [DSH-030](dsh-030.md#timeline-0372-0377) |
| 378～387 | [DSH-031](dsh-031.md#timeline-0378-0387) |
| 388～410 | [DSH-032](dsh-032.md#timeline-0388-0410) |
| 411～416 | [DSH-033](dsh-033.md#timeline-0411-0416) |
| 417～448 | [DSH-034](dsh-034.md#timeline-0417-0448) |
| 449～467 | [DSH-035](dsh-035.md#timeline-0449-0467) |
| 468～473 | [DSH-034](dsh-034.md#timeline-0468-0473) |
| 474～481 | [DSH-035](dsh-035.md#timeline-0474-0481) |
| 482～509 | [DSH-036](dsh-036.md#timeline-0482-0509) |
| 510～518 | [DSH-037](dsh-037.md#timeline-0510-0518) |
| 519～524 | [DSH-039](dsh-039.md#timeline-0519-0524) |
| 525～531 | [DSH-041](dsh-041.md#timeline-0525-0531) |

## 设计基线

| 编号 | 文档 | 摘要 |
| --- | --- | --- |
| DSH-000 | [dsh-000.md](dsh-000.md) | 汇总早期环境边界与 Robot/Policy/Runtime 接口、仓库目录规范、逐步实现路线图、仿真环境与最小闭环方案、容器依赖分析，以及当时的文档索引和旧路径迁移提示。 |

## S1（DSH-001～013）

| 编号 | 文档 | 摘要 |
| --- | --- | --- |
| DSH-001 | [dsh-001.md](dsh-001.md) | 阶段 2R 的首个委派试验，修复 ObservationBuffer 在同一 observation ID 下样本引用漂移：首次成功组装后固定 state 与相机引用，迟到合法帧只能供后续 state 使用。 |
| DSH-002 | [dsh-002.md](dsh-002.md) | R3 控制清理修复：关闭续租后依次尝试在途续租、停止和释放，前一步失败不跳过后继，错误保留步骤名；清理消耗本地 lease，重复调用不重复发 RPC。 |
| DSH-003 | [dsh-003.md](dsh-003.md) | R1 中断收尾修复：信号处理器只记录首个 SIGINT/SIGTERM，主线程停止接收新 episode，worker 停止动作、释放控制权并关闭记录后才销毁 ROS 资源；普通完成、运行失败、两种信号和收尾超时使用不同退出码。 |
| DSH-004 | [dsh-004.md](dsh-004.md) | S1a 类型配置与启动前校验：不可变 typed config、来源路径和边界校验，避免错误值被接受或配置快照被污染。 |
| DSH-005 | [dsh-005.md](dsh-005.md) | S1b 控制客户端提取：`ControlClient` 独占控制 RPC/action、lease、续租和清理状态，node 不再重复持有；故障回调在锁外执行，清理步骤独立尝试并汇总。 |
| DSH-006 | [dsh-006.md](dsh-006.md) | S1b episode 生命周期提取：`EpisodeRunner` 统一拥有 worker、停止/关闭请求、context、recorder、记录路径和失败汇总，node 保留 ROS 注册、消息转换、观测同步和进程入口。 |
| DSH-007 | [dsh-007.md](dsh-007.md) | S1c 公共事件日志库：新增 `robot_run_logging` 标准库包与事件 schema，提供有界队列、JSONL 轮转、关联 ID、严格载荷、健康检查和有界关闭。 |
| DSH-008 | [dsh-008.md](dsh-008.md) | S1c runtime 日志接线：不可变 `LogSettings`、run 目录独占、配置/版本来源快照、episode/命令/阶段/停止事件关联，日志故障阻止继续执行，stop/release 仍独立尝试；关闭前记录 `run.finalizing`，关闭后输出进程结果。 |
| DSH-009 | [dsh-009.md](dsh-009.md) | S1c 有界 MCAP 写入与流式审计：单 writer 线程、条数与字节双重有界队列、慢盘不阻塞控制、故障持久化；审计按条读取，只保留引用元数据并按 1 MiB 分块哈希。 |
| DSH-010 | [dsh-010.md](dsh-010.md) | S1c 结果分类与人工结束：新 episode 写 v2 manifest，分开表达记录完整性、runtime 结果和任务标签；人工 End 返回 ENDING 确认，保留首次原因并有界等待已知命令终态，历史 v1 严格读取且不原地改写。 |
| DSH-011 | [dsh-011.md](dsh-011.md) | S1d Compose 开发挂载：runtime 与 mock 后端共用稳定 ROS 依赖镜像，源码/配置/生成契约只读挂载，原始数据与日志直接写宿主，容器按调用者 UID/GID 运行；契约从 `contracts/ros` 单一来源生成并校验来源摘要。 |
| DSH-012 | [dsh-012.md](dsh-012.md) | S1d 人工操作命令：`up/status/start/end/down` 可在无宿主机 ROS 时操作 Compose 开发栈；主机入口核对项目、容器、run id、镜像和挂载归属，分离 PREPARING/ENDING 服务 ACK 与最终 manifest，按 episode id 等待并保留 recording/runtime/task 三维结果。 |
| DSH-013 | [dsh-013.md](dsh-013.md) | S1d 自动采集与整次运行监督：统一 Compose 入口，JSON 启动器配方集中管理工作期限、停止/清理预算、诊断和日志容量；一次采集由真实进程退出码、独立 MCAP 审计、关键日志排空和清理结果共同判定，并原子写 `status.json`。 |

## S2（DSH-014～017）

| 编号 | 文档 | 摘要 |
| --- | --- | --- |
| DSH-014 | [dsh-014.md](dsh-014.md) | S2a 仿真环境门槛：固定 15 个官方 wheel 版本与 SHA-256，基于 Python 3.12/Jazzy/NumPy 1.26.4 新增 `mujoco-dev` target；Panda 资产按固定 Menagerie 提交 fetch/verify，一次性 `sim_check.py` 检查依赖、资产、单 Panda step、QP 和 NVIDIA EGL。 |
| DSH-015 | [dsh-015.md](dsh-015.md) | S2b 双 Panda 场景模型：场景参数集中到 `configs/simulation/panda_stack.yaml`，原生 MjSpec 挂接固定 Menagerie Panda，包含柱、两侧 1 m 基座、0.75 m 桌面、两个自由方块和三个固定相机；保留上游网格、动力学、tendon 和执行器，具名索引避免依赖数组相邻，并提供确定性复位。 |
| DSH-016 | [dsh-016.md](dsh-016.md) | S2 三相机静态可见性：增加场景快照、可见性检查和一次性检查入口，覆盖 67 个初始布局、PNG、原生像素审计和关闭故障验证。 |
| DSH-017 | [dsh-017.md](dsh-017.md) | S2 关键位姿、IK、路径净空与三相机视野检查：数值求解只影响 IK 搜索，实际路径仍从 home 开始；冻结关键位姿、离散净空并逐阶段核对三相机视野。 |

## S3（DSH-018～032）

| 编号 | 文档 | 摘要 |
| --- | --- | --- |
| DSH-018 | [dsh-018.md](dsh-018.md) | S3 三相机 profile、runtime 接线与原始记录：v2 具名相机集合、16 维关节顺序、三相机 mock/MCAP 与 audit，19 个允许文件整合。 |
| DSH-019 | [dsh-019.md](dsh-019.md) | 命令状态查询、执行时间与反馈记录：新增 `CommandStatus`、`QueryCommand` 和独立 `command_feedback` 流，分离采样/执行时钟，记录接受、开始、反馈和终态；mock 测量仍等于参考，物理执行尚未接入。 |
| DSH-020 | [dsh-020.md](dsh-020.md) | 双 Panda 16 维 SI 执行器与测量映射：具名 qpos/dof/actuator 绑定、夹爪两指总开度转换、线程约束、声明标定与有效姿态区分，以及真实测量边界。 |
| DSH-021 | [dsh-021.md](dsh-021.md) | 可验证的分段三次参考计划：标准库共享不可变计划、采样、连续极值与约束检查，仿真侧用固定 SciPy 1.17.1 一次性编译；保留原始路点位置和时间，拒绝区间内部过冲及速度/加速度超限。 |
| DSH-022 | [dsh-022.md](dsh-022.md) | 无 ROS 物理运动、双指接触完成与有界停止：严格控制配置、真实步进参考/测量快照、常减速刹车和单运动状态机；双指接触分别判定，停止边界和原生重放可审计。 |
| DSH-023 | [dsh-023.md](dsh-023.md) | 连续参考边界与观测锚定动作窗口：编译/验证接受显式起止参考速度（默认仍为零端点），连续块使用严格区间采样；共享 `ActionWindow` 按观测捕获时间选择未来窗口，保留完整源、索引和重叠计数，并加入刹车余量。 |
| DSH-024 | [dsh-024.md](dsh-024.md) | 有界连续动作块的原生执行：真实跨块衔接、事件身份、受控停止、接触与期限故障，以及独立物理重放。 |
| DSH-025 | [dsh-025.md](dsh-025.md) | 原生控制会话：纯标准库 `ControlCommand` 与共享 generation 比较，`NativeController` 组合既有 `NativeMotion`，提供租约、整体命令/块幂等、有界查询、真实停止和 episode 边界 reset；期限用墙钟，参考与测量用仿真时间。 |
| DSH-026 | [dsh-026.md](dsh-026.md) | 原生参考与反馈公共类型：DTO 保存完整原请求、显式分段系数、来源观测/选中窗口、接触证据和真实反馈；原生侧只做预编译与转换，ROS 侧显式编解码。 |
| DSH-027 | [dsh-027.md](dsh-027.md) | 可运行的 ROS 原生 MuJoCo 后端：真实 ROS 服务可执行运动和连续块，三图与测量使用同一快照。 |
| DSH-028 | [dsh-028.md](dsh-028.md) | 原生诊断运行时与可审计 v3 MCAP：精确捕获、租约/ACK、v3 manifest、独立审计和故障处理；旧回归中把未知版本硬编码为 3 的测试夹具修正为 99，不放宽拒绝规则。 |
| DSH-029 | [dsh-029.md](dsh-029.md) | 受限目标跟随纯数学层：`follower.py` 保存不可变计划与验证，`follower_math.py` 提供通用端速约束和连续偏差极值，均只依赖标准库；单窗口 Ruckig 计算与显式分段系数导出分开。 |
| DSH-030 | [dsh-030.md](dsh-030.md) | 受限跟随接入同一原生物理执行器：三块独立运动仿真、参考跟踪误差、公开 `NativeExecution` DTO、原系数与应用参考、测量及应用/采样时钟逐步核对；旧 MOTION 路径保留。 |
| DSH-031 | [dsh-031.md](dsh-031.md) | 受限跟随公共协议与 ROS 后端：新增独立 `FollowerReference`/`SubmitFollower`，复用同一物理 owner、规划器、真实反馈和块历史；原 31 个 msg/srv/action 逐字不变，严格 `SubmitNative` 保留，配置显式启用才声明能力。 |
| DSH-032 | [dsh-032.md](dsh-032.md) | 连续 Policy RPC、受限执行与 MCAP v4：真实 gRPC→ROS→MuJoCo 连续 8 块→v4→独立审计，3 类语义篡改被拒绝；修正整体/每块身份混用、停止时活动块漏终态和跨 topic 收尾竞态。 |

## S4～S5（DSH-033～035）

| 编号 | 文档 | 摘要 |
| --- | --- | --- |
| DSH-033 | [dsh-033.md](dsh-033.md) | 独立堆叠判定与可选程序示教：任务状态机、双指接触聚合、真实 Stop/冻结事实、任务失败日志和官方 SDK 提升；30 个文件整合，1030 项业务回归及真实 ROS/CDR 两模式停止/重置通过。 |
| DSH-034 | [dsh-034.md](dsh-034.md) | 示教运行时、任务收尾与有预算试采：teacher 与 policy 共用 stream、连续 follower/ACK/插值/反馈/受控停止链，导出逐帧 RGB/执行参考和质量标签。 |
| DSH-035 | [dsh-035.md](dsh-035.md) | 数据转换、ACT 训练与离线模型包：45/5 独立数据集、train-only processor、追加式预算台账、safetensors bundle、CPU 合成检查、容器与本机 venv 入口。 |

## S6～S7（DSH-036）

| 编号 | 文档 | 摘要 |
| --- | --- | --- |
| DSH-036 | [dsh-036.md](dsh-036.md) | 真实 ACT PolicyProvider 与固定种子评估：具名 ACT adapter/service、真实 bundle 校验、独立 Compose、pilot 1008/1009 与 test 20000..20019；20/20 可信执行闭环，物理堆叠 0/20。 |

## S6～S7 后续修复（DSH-037～041）

| 编号 | 文档 | 摘要 |
| --- | --- | --- |
| DSH-037 | [dsh-037.md](dsh-037.md) | ACT 重规划周期修复：从 20 例原始 MCAP 重建被拒绝请求，独立实现 2 点 ACT 配方、拒绝证据和公共 backend 参数；原速 3 次、半速 1 次实测仍在 2 块后迟到/断流，失败保留在独立台账。 |
| DSH-038 | [dsh-038.md](dsh-038.md) | ROS ACK 等待立即唤醒：两个客户端用 Future 完成回调设置 Event，保留原超时、停止、取消、异常、ACK 拒绝和清理规则。 |
| DSH-039 | [dsh-039.md](dsh-039.md) | ACT 原始输出静止倾向独立诊断：关联 289 条原始预测，287 条有精确源观察；完整 60 步接近静止的块为 0，但实际常选 200/250 ms 低位移段。 |
| DSH-040 | [dsh-040.md](dsh-040.md) | 静止冗余训练起点筛选：只过滤当前观察起点，不删除原始帧、不改变 50 ms 时间槽或未来 60 步动作；19676 个训练起点保留 19670、排除 6 个退开阶段起点，验证 5 条 2160 帧不变，并接入 sampler/checkpoint/bundle 来源绑定。 |
| DSH-041 | [dsh-041.md](dsh-041.md) | batch16 / 20K 新训练配方与独立训练作业：完成部署文件、真实 GPU 预检和后台启动；首轮 600 秒限时退出 124，集中返工修正文档墙钟估时和无效线程环境变量。 |

## 文档收敛（DSH-042）

| 编号 | 文档 | 摘要 |
| --- | --- | --- |
| DSH-042 | [文档收敛与历史归档](dsh-042.md) | 文档职责收敛、历史归档与迁移验收；本文件由主助手维护。 |

## 部署整理（DSH-043）

| 编号 | 文档 | 摘要 |
| --- | --- | --- |
| DSH-043 | [源码挂载与 Docker 构建边界](dsh-043.md) | 移除业务 COPY、提取内嵌代码、SDK 来源检查与实际容器验收。 |

## 新模型验收（DSH-044）

| 编号 | 文档 | 摘要 |
| --- | --- | --- |
| DSH-044 | [20K 模型导出与首次仿真验证](dsh-044.md) | 模型包与真实验证观察检查通过；新种子单次执行 82 块后因 CHUNK_LATE 停车，未堆叠成功，显存足够；保留视频与失败证据。 |
| DSH-045 | [推理窗口与动作推进修复](dsh-045.md) | 1s窗口恢复接近动作；新旧预测切换仍触发偏差拒绝，继续分析。 |
| DSH-046 | [任务准备的有界静止等待](dsh-046.md) | 保持后端静止判定，按原总预算等待reset稳定，20项真实ROS回归通过。 |
| DSH-047 | [ACT按时间对齐的预测融合](dsh-047.md) | 模型与会话边界测试通过；真实融合与数值凸包修复已验收。 |
| DSH-048 | [融合窗口对照与真实成功验收](dsh-048.md) | 完整保留探索失败；30600同场景回归稳定堆叠成功，图像质量项未全过。 |
| DSH-049 | [首次预测前的稳定等待](dsh-049.md) | 显式等待2s仿真稳定，避免reset极早期测量离开训练分布；29项目标回归通过。 |
| DSH-050 | [亚纳秒制动前缀导出](dsh-050.md) | 修复22飞秒前缀误拒绝，41项真实Ruckig回归与16DOF失败重建通过。 |
| DSH-051 | [静止夹持速度的数值分类](dsh-051.md) | 与原卸载逻辑统一1e-9m/s数值容差，17项回归通过；夹持误停修复后完成堆叠。 |

## 目录迁移（DSH-052）

| 编号 | 文档 | 摘要 |
| --- | --- | --- |
| DSH-052 | [按端与实现拆分目录](dsh-052.md) | 迁移仿真/运行/学习实现、配置、Docker、Compose 与脚本；路径与只读回归通过，容器和集成验证留待 053。 |

## 环境与业务执行分离（DSH-053）

| 编号 | 文档 | 摘要 |
| --- | --- | --- |
| DSH-053 | [环境与业务执行分离](dsh-053.md) | 宿主唯一 `scripts/env.sh`；三端 idle Compose 与容器内 `run.sh`；learning 生成真实 bundle 绑定、runtime 消费通用 JSON；旧宿主启动器/overlay 清理，静态与有限纯回归通过。 |

## 每端唯一 Docker 环境与入口（DSH-054）

| 编号 | 文档 | 摘要 |
| --- | --- | --- |
| DSH-054 | [每端唯一 Docker 环境与入口](dsh-054.md) | 每实现一份 Dockerfile/docker-compose.yml/run.sh；根 Compose 只 include；env.sh 转发 run.sh 功能；旧入口、多 target 和宿主 venv 路线清理；候选静态与纯回归通过，实际构建/容器验收由主助手独立执行。 |

## 旧路径迁移表

原路径均来自 `migration-input.json`；逐项哈希和正文覆盖见 `.build/reports/dsh-042/migration-report.json`。`docs/README.md`、各目录 `README.md` 和根 `README.md` 的重写/保留状态在现位置说明。

| 原路径 | 归档位置 |
| --- | --- |
| `README.md` | [docs/history/dsh-036.md#src-readme-md](dsh-036.md#src-readme-md) |
| `contracts/README.md` | [docs/history/dsh-018.md#src-contracts-readme-md](dsh-018.md#src-contracts-readme-md) |
| `deploy/README.md` | [docs/history/dsh-011.md#src-deploy-readme-md](dsh-011.md#src-deploy-readme-md) |
| `deploy/docker/learning/README.md` | [docs/history/dsh-035.md#src-deploy-docker-learning-readme-md](dsh-035.md#src-deploy-docker-learning-readme-md) |
| `docs/README.md` | [docs/history/dsh-000.md#src-docs-readme-md](dsh-000.md#src-docs-readme-md) |
| `docs/act-b16-20k-training.md` | [docs/history/dsh-041.md#src-docs-act-b16-20k-training-md](dsh-041.md#src-docs-act-b16-20k-training-md) |
| `docs/act-cadence-repair.md` | [docs/history/dsh-037.md#src-docs-act-cadence-repair-md](dsh-037.md#src-docs-act-cadence-repair-md) |
| `docs/act-execution-alignment-review.md` | [docs/history/dsh-029.md#src-docs-act-execution-alignment-review-md](dsh-029.md#src-docs-act-execution-alignment-review-md) |
| `docs/act-motion-diagnosis.md` | [docs/history/dsh-039.md#src-docs-act-motion-diagnosis-md](dsh-039.md#src-docs-act-motion-diagnosis-md) |
| `docs/act-policy-service.md` | [docs/history/dsh-036.md#src-docs-act-policy-service-md](dsh-036.md#src-docs-act-policy-service-md) |
| `docs/architecture-and-interfaces.md` | [docs/history/dsh-000.md#src-docs-architecture-and-interfaces-md](dsh-000.md#src-docs-architecture-and-interfaces-md) |
| `docs/archive/humble-py310-analysis.md` | [docs/history/dsh-000.md#src-docs-archive-humble-py310-analysis-md](dsh-000.md#src-docs-archive-humble-py310-analysis-md) |
| `docs/bounded-target-follower.md` | [docs/history/dsh-029.md#src-docs-bounded-target-follower-md](dsh-029.md#src-docs-bounded-target-follower-md) |
| `docs/closure-plan.md` | [docs/history/dsh-036.md#src-docs-closure-plan-md](dsh-036.md#src-docs-closure-plan-md) |
| `docs/contact-unload-validation.md` | [docs/history/dsh-034.md#src-docs-contact-unload-validation-md](dsh-034.md#src-docs-contact-unload-validation-md) |
| `docs/continuous-action-design.md` | [docs/history/dsh-023.md#src-docs-continuous-action-design-md](dsh-023.md#src-docs-continuous-action-design-md) |
| `docs/contracts/README.md` | [docs/history/dsh-000.md#src-docs-contracts-readme-md](dsh-000.md#src-docs-contracts-readme-md) |
| `docs/contracts/deployment.example.yaml` | [docs/history/dsh-000.md#src-docs-contracts-deployment-example-yaml](dsh-000.md#src-docs-contracts-deployment-example-yaml) |
| `docs/contracts/policy.proto` | [docs/history/dsh-000.md#src-docs-contracts-policy-proto](dsh-000.md#src-docs-contracts-policy-proto) |
| `docs/dependencies/README.md` | [docs/history/dsh-000.md#src-docs-dependencies-readme-md](dsh-000.md#src-docs-dependencies-readme-md) |
| `docs/dependencies/lerobot-py310.candidate.in` | [docs/history/dsh-000.md#src-docs-dependencies-lerobot-py310-candidate-in](dsh-000.md#src-docs-dependencies-lerobot-py310-candidate-in) |
| `docs/dependencies/lerobot-py310.resolved.txt` | [docs/history/dsh-000.md#src-docs-dependencies-lerobot-py310-resolved-txt](dsh-000.md#src-docs-dependencies-lerobot-py310-resolved-txt) |
| `docs/docker-dependency-analysis.md` | [docs/history/dsh-000.md#src-docs-docker-dependency-analysis-md](dsh-000.md#src-docs-docker-dependency-analysis-md) |
| `docs/dsh-collaboration-plan.md` | [docs/history/dsh-001.md#src-docs-dsh-collaboration-plan-md](dsh-001.md#src-docs-dsh-collaboration-plan-md) |
| `docs/episode-recording-format.md` | [docs/history/dsh-010.md#src-docs-episode-recording-format-md](dsh-010.md#src-docs-episode-recording-format-md) |
| `docs/implementation-roadmap.md` | [docs/history/dsh-000.md#src-docs-implementation-roadmap-md](dsh-000.md#src-docs-implementation-roadmap-md) |
| `docs/kinematic-scene-check.md` | [docs/history/dsh-017.md#src-docs-kinematic-scene-check-md](dsh-017.md#src-docs-kinematic-scene-check-md) |
| `docs/learning-dataset-act.md` | [docs/history/dsh-035.md#src-docs-learning-dataset-act-md](dsh-035.md#src-docs-learning-dataset-act-md) |
| `docs/learning-environment-preflight.md` | [docs/history/dsh-035.md#src-docs-learning-environment-preflight-md](dsh-035.md#src-docs-learning-environment-preflight-md) |
| `docs/learning-sample-filter.md` | [docs/history/dsh-040.md#src-docs-learning-sample-filter-md](dsh-040.md#src-docs-learning-sample-filter-md) |
| `docs/learning-training.md` | [docs/history/dsh-035.md#src-docs-learning-training-md](dsh-035.md#src-docs-learning-training-md) |
| `docs/logging-and-recording.md` | [docs/history/dsh-009.md#src-docs-logging-and-recording-md](dsh-009.md#src-docs-logging-and-recording-md) |
| `docs/maintainability-review.md` | [docs/history/dsh-001.md#src-docs-maintainability-review-md](dsh-001.md#src-docs-maintainability-review-md) |
| `docs/minimal-simulation-loop-plan.md` | [docs/history/dsh-000.md#src-docs-minimal-simulation-loop-plan-md](dsh-000.md#src-docs-minimal-simulation-loop-plan-md) |
| `docs/minimal-simulation-loop.md` | [docs/history/dsh-036.md#src-docs-minimal-simulation-loop-md](dsh-036.md#src-docs-minimal-simulation-loop-md) |
| `docs/native-backend.md` | [docs/history/dsh-027.md#src-docs-native-backend-md](dsh-027.md#src-docs-native-backend-md) |
| `docs/native-control.md` | [docs/history/dsh-025.md#src-docs-native-control-md](dsh-025.md#src-docs-native-control-md) |
| `docs/native-follower-protocol.md` | [docs/history/dsh-031.md#src-docs-native-follower-protocol-md](dsh-031.md#src-docs-native-follower-protocol-md) |
| `docs/native-follower.md` | [docs/history/dsh-030.md#src-docs-native-follower-md](dsh-030.md#src-docs-native-follower-md) |
| `docs/native-motion.md` | [docs/history/dsh-022.md#src-docs-native-motion-md](dsh-022.md#src-docs-native-motion-md) |
| `docs/native-policy.md` | [docs/history/dsh-032.md#src-docs-native-policy-md](dsh-032.md#src-docs-native-policy-md) |
| `docs/native-protocol.md` | [docs/history/dsh-026.md#src-docs-native-protocol-md](dsh-026.md#src-docs-native-protocol-md) |
| `docs/native-recording.md` | [docs/history/dsh-028.md#src-docs-native-recording-md](dsh-028.md#src-docs-native-recording-md) |
| `docs/native-teacher-runtime.md` | [docs/history/dsh-034.md#src-docs-native-teacher-runtime-md](dsh-034.md#src-docs-native-teacher-runtime-md) |
| `docs/panda-io.md` | [docs/history/dsh-020.md#src-docs-panda-io-md](dsh-020.md#src-docs-panda-io-md) |
| `docs/panda-stack-scene.md` | [docs/history/dsh-015.md#src-docs-panda-stack-scene-md](dsh-015.md#src-docs-panda-stack-scene-md) |
| `docs/reference-trajectory-design.md` | [docs/history/dsh-021.md#src-docs-reference-trajectory-design-md](dsh-021.md#src-docs-reference-trajectory-design-md) |
| `docs/repository-layout.md` | [docs/history/dsh-000.md#src-docs-repository-layout-md](dsh-000.md#src-docs-repository-layout-md) |
| `docs/simulation-control-design.md` | [docs/history/dsh-019.md#src-docs-simulation-control-design-md](dsh-019.md#src-docs-simulation-control-design-md) |
| `docs/simulation-design.md` | [docs/history/dsh-000.md#src-docs-simulation-design-md](dsh-000.md#src-docs-simulation-design-md) |
| `docs/simulation-environment.md` | [docs/history/dsh-014.md#src-docs-simulation-environment-md](dsh-014.md#src-docs-simulation-environment-md) |
| `docs/simulation-loop-discussion.md` | [docs/history/dsh-000.md#src-docs-simulation-loop-discussion-md](dsh-000.md#src-docs-simulation-loop-discussion-md) |
| `docs/simulation-loop-result.md` | [docs/history/dsh-036.md#src-docs-simulation-loop-result-md](dsh-036.md#src-docs-simulation-loop-result-md) |
| `docs/stack-task-teacher.md` | [docs/history/dsh-033.md#src-docs-stack-task-teacher-md](dsh-033.md#src-docs-stack-task-teacher-md) |
| `docs/stacking-reference-review.md` | [docs/history/dsh-033.md#src-docs-stacking-reference-review-md](dsh-033.md#src-docs-stacking-reference-review-md) |
| `docs/stage2-quickstart.md` | [docs/history/dsh-013.md#src-docs-stage2-quickstart-md](dsh-013.md#src-docs-stage2-quickstart-md) |
| `docs/tasks/dsh-001-observation-identity.md` | [docs/history/dsh-001.md#src-docs-tasks-dsh-001-observation-identity-md](dsh-001.md#src-docs-tasks-dsh-001-observation-identity-md) |
| `docs/tasks/dsh-001-result.md` | [docs/history/dsh-001.md#src-docs-tasks-dsh-001-result-md](dsh-001.md#src-docs-tasks-dsh-001-result-md) |
| `docs/tasks/dsh-002-control-cleanup.md` | [docs/history/dsh-002.md#src-docs-tasks-dsh-002-control-cleanup-md](dsh-002.md#src-docs-tasks-dsh-002-control-cleanup-md) |
| `docs/tasks/dsh-002-result.md` | [docs/history/dsh-002.md#src-docs-tasks-dsh-002-result-md](dsh-002.md#src-docs-tasks-dsh-002-result-md) |
| `docs/tasks/dsh-003-result.md` | [docs/history/dsh-003.md#src-docs-tasks-dsh-003-result-md](dsh-003.md#src-docs-tasks-dsh-003-result-md) |
| `docs/tasks/dsh-003-runtime-shutdown.md` | [docs/history/dsh-003.md#src-docs-tasks-dsh-003-runtime-shutdown-md](dsh-003.md#src-docs-tasks-dsh-003-runtime-shutdown-md) |
| `docs/tasks/dsh-004-result.md` | [docs/history/dsh-004.md#src-docs-tasks-dsh-004-result-md](dsh-004.md#src-docs-tasks-dsh-004-result-md) |
| `docs/tasks/dsh-004-typed-configuration.md` | [docs/history/dsh-004.md#src-docs-tasks-dsh-004-typed-configuration-md](dsh-004.md#src-docs-tasks-dsh-004-typed-configuration-md) |
| `docs/tasks/dsh-005-control-client.md` | [docs/history/dsh-005.md#src-docs-tasks-dsh-005-control-client-md](dsh-005.md#src-docs-tasks-dsh-005-control-client-md) |
| `docs/tasks/dsh-005-result.md` | [docs/history/dsh-005.md#src-docs-tasks-dsh-005-result-md](dsh-005.md#src-docs-tasks-dsh-005-result-md) |
| `docs/tasks/dsh-006-episode-lifecycle.md` | [docs/history/dsh-006.md#src-docs-tasks-dsh-006-episode-lifecycle-md](dsh-006.md#src-docs-tasks-dsh-006-episode-lifecycle-md) |
| `docs/tasks/dsh-006-result.md` | [docs/history/dsh-006.md#src-docs-tasks-dsh-006-result-md](dsh-006.md#src-docs-tasks-dsh-006-result-md) |
| `docs/tasks/dsh-007-event-logging.md` | [docs/history/dsh-007.md#src-docs-tasks-dsh-007-event-logging-md](dsh-007.md#src-docs-tasks-dsh-007-event-logging-md) |
| `docs/tasks/dsh-007-result.md` | [docs/history/dsh-007.md#src-docs-tasks-dsh-007-result-md](dsh-007.md#src-docs-tasks-dsh-007-result-md) |
| `docs/tasks/dsh-008-result.md` | [docs/history/dsh-008.md#src-docs-tasks-dsh-008-result-md](dsh-008.md#src-docs-tasks-dsh-008-result-md) |
| `docs/tasks/dsh-008-runtime-logging.md` | [docs/history/dsh-008.md#src-docs-tasks-dsh-008-runtime-logging-md](dsh-008.md#src-docs-tasks-dsh-008-runtime-logging-md) |
| `docs/tasks/dsh-009-recording-queue.md` | [docs/history/dsh-009.md#src-docs-tasks-dsh-009-recording-queue-md](dsh-009.md#src-docs-tasks-dsh-009-recording-queue-md) |
| `docs/tasks/dsh-009-result.md` | [docs/history/dsh-009.md#src-docs-tasks-dsh-009-result-md](dsh-009.md#src-docs-tasks-dsh-009-result-md) |
| `docs/tasks/dsh-010-episode-outcomes.md` | [docs/history/dsh-010.md#src-docs-tasks-dsh-010-episode-outcomes-md](dsh-010.md#src-docs-tasks-dsh-010-episode-outcomes-md) |
| `docs/tasks/dsh-010-result.md` | [docs/history/dsh-010.md#src-docs-tasks-dsh-010-result-md](dsh-010.md#src-docs-tasks-dsh-010-result-md) |
| `docs/tasks/dsh-011-compose-development.md` | [docs/history/dsh-011.md#src-docs-tasks-dsh-011-compose-development-md](dsh-011.md#src-docs-tasks-dsh-011-compose-development-md) |
| `docs/tasks/dsh-011-result.md` | [docs/history/dsh-011.md#src-docs-tasks-dsh-011-result-md](dsh-011.md#src-docs-tasks-dsh-011-result-md) |
| `docs/tasks/dsh-012-manual-cli.md` | [docs/history/dsh-012.md#src-docs-tasks-dsh-012-manual-cli-md](dsh-012.md#src-docs-tasks-dsh-012-manual-cli-md) |
| `docs/tasks/dsh-012-result.md` | [docs/history/dsh-012.md#src-docs-tasks-dsh-012-result-md](dsh-012.md#src-docs-tasks-dsh-012-result-md) |
| `docs/tasks/dsh-013-result.md` | [docs/history/dsh-013.md#src-docs-tasks-dsh-013-result-md](dsh-013.md#src-docs-tasks-dsh-013-result-md) |
| `docs/tasks/dsh-013-run-supervision.md` | [docs/history/dsh-013.md#src-docs-tasks-dsh-013-run-supervision-md](dsh-013.md#src-docs-tasks-dsh-013-run-supervision-md) |
| `docs/tasks/dsh-014-result.md` | [docs/history/dsh-014.md#src-docs-tasks-dsh-014-result-md](dsh-014.md#src-docs-tasks-dsh-014-result-md) |
| `docs/tasks/dsh-014-simulation-environment.md` | [docs/history/dsh-014.md#src-docs-tasks-dsh-014-simulation-environment-md](dsh-014.md#src-docs-tasks-dsh-014-simulation-environment-md) |
| `docs/tasks/dsh-015-result.md` | [docs/history/dsh-015.md#src-docs-tasks-dsh-015-result-md](dsh-015.md#src-docs-tasks-dsh-015-result-md) |
| `docs/tasks/dsh-015-scene-model.md` | [docs/history/dsh-015.md#src-docs-tasks-dsh-015-scene-model-md](dsh-015.md#src-docs-tasks-dsh-015-scene-model-md) |
| `docs/tasks/dsh-016-result.md` | [docs/history/dsh-016.md#src-docs-tasks-dsh-016-result-md](dsh-016.md#src-docs-tasks-dsh-016-result-md) |
| `docs/tasks/dsh-016-scene-visibility.md` | [docs/history/dsh-016.md#src-docs-tasks-dsh-016-scene-visibility-md](dsh-016.md#src-docs-tasks-dsh-016-scene-visibility-md) |
| `docs/tasks/dsh-017-kinematic-check.md` | [docs/history/dsh-017.md#src-docs-tasks-dsh-017-kinematic-check-md](dsh-017.md#src-docs-tasks-dsh-017-kinematic-check-md) |
| `docs/tasks/dsh-017-result.md` | [docs/history/dsh-017.md#src-docs-tasks-dsh-017-result-md](dsh-017.md#src-docs-tasks-dsh-017-result-md) |
| `docs/tasks/dsh-018-multicamera-profiles.md` | [docs/history/dsh-018.md#src-docs-tasks-dsh-018-multicamera-profiles-md](dsh-018.md#src-docs-tasks-dsh-018-multicamera-profiles-md) |
| `docs/tasks/dsh-018-result.md` | [docs/history/dsh-018.md#src-docs-tasks-dsh-018-result-md](dsh-018.md#src-docs-tasks-dsh-018-result-md) |
| `docs/tasks/dsh-019-command-status.md` | [docs/history/dsh-019.md#src-docs-tasks-dsh-019-command-status-md](dsh-019.md#src-docs-tasks-dsh-019-command-status-md) |
| `docs/tasks/dsh-019-result.md` | [docs/history/dsh-019.md#src-docs-tasks-dsh-019-result-md](dsh-019.md#src-docs-tasks-dsh-019-result-md) |
| `docs/tasks/dsh-020-panda-io.md` | [docs/history/dsh-020.md#src-docs-tasks-dsh-020-panda-io-md](dsh-020.md#src-docs-tasks-dsh-020-panda-io-md) |
| `docs/tasks/dsh-020-result.md` | [docs/history/dsh-020.md#src-docs-tasks-dsh-020-result-md](dsh-020.md#src-docs-tasks-dsh-020-result-md) |
| `docs/tasks/dsh-021-reference-plan.md` | [docs/history/dsh-021.md#src-docs-tasks-dsh-021-reference-plan-md](dsh-021.md#src-docs-tasks-dsh-021-reference-plan-md) |
| `docs/tasks/dsh-021-result.md` | [docs/history/dsh-021.md#src-docs-tasks-dsh-021-result-md](dsh-021.md#src-docs-tasks-dsh-021-result-md) |
| `docs/tasks/dsh-022-native-motion.md` | [docs/history/dsh-022.md#src-docs-tasks-dsh-022-native-motion-md](dsh-022.md#src-docs-tasks-dsh-022-native-motion-md) |
| `docs/tasks/dsh-022-result.md` | [docs/history/dsh-022.md#src-docs-tasks-dsh-022-result-md](dsh-022.md#src-docs-tasks-dsh-022-result-md) |
| `docs/tasks/dsh-023-continuous-plans.md` | [docs/history/dsh-023.md#src-docs-tasks-dsh-023-continuous-plans-md](dsh-023.md#src-docs-tasks-dsh-023-continuous-plans-md) |
| `docs/tasks/dsh-023-result.md` | [docs/history/dsh-023.md#src-docs-tasks-dsh-023-result-md](dsh-023.md#src-docs-tasks-dsh-023-result-md) |
| `docs/tasks/dsh-024-native-stream.md` | [docs/history/dsh-024.md#src-docs-tasks-dsh-024-native-stream-md](dsh-024.md#src-docs-tasks-dsh-024-native-stream-md) |
| `docs/tasks/dsh-024-result.md` | [docs/history/dsh-024.md#src-docs-tasks-dsh-024-result-md](dsh-024.md#src-docs-tasks-dsh-024-result-md) |
| `docs/tasks/dsh-025-native-control.md` | [docs/history/dsh-025.md#src-docs-tasks-dsh-025-native-control-md](dsh-025.md#src-docs-tasks-dsh-025-native-control-md) |
| `docs/tasks/dsh-025-result.md` | [docs/history/dsh-025.md#src-docs-tasks-dsh-025-result-md](dsh-025.md#src-docs-tasks-dsh-025-result-md) |
| `docs/tasks/dsh-026-native-protocol.md` | [docs/history/dsh-026.md#src-docs-tasks-dsh-026-native-protocol-md](dsh-026.md#src-docs-tasks-dsh-026-native-protocol-md) |
| `docs/tasks/dsh-026-result.md` | [docs/history/dsh-026.md#src-docs-tasks-dsh-026-result-md](dsh-026.md#src-docs-tasks-dsh-026-result-md) |
| `docs/tasks/dsh-027-native-backend.md` | [docs/history/dsh-027.md#src-docs-tasks-dsh-027-native-backend-md](dsh-027.md#src-docs-tasks-dsh-027-native-backend-md) |
| `docs/tasks/dsh-027-result.md` | [docs/history/dsh-027.md#src-docs-tasks-dsh-027-result-md](dsh-027.md#src-docs-tasks-dsh-027-result-md) |
| `docs/tasks/dsh-028-native-runtime.md` | [docs/history/dsh-028.md#src-docs-tasks-dsh-028-native-runtime-md](dsh-028.md#src-docs-tasks-dsh-028-native-runtime-md) |
| `docs/tasks/dsh-028-result.md` | [docs/history/dsh-028.md#src-docs-tasks-dsh-028-result-md](dsh-028.md#src-docs-tasks-dsh-028-result-md) |
| `docs/tasks/dsh-029-bounded-follower-math.md` | [docs/history/dsh-029.md#src-docs-tasks-dsh-029-bounded-follower-math-md](dsh-029.md#src-docs-tasks-dsh-029-bounded-follower-math-md) |
| `docs/tasks/dsh-029-result.md` | [docs/history/dsh-029.md#src-docs-tasks-dsh-029-result-md](dsh-029.md#src-docs-tasks-dsh-029-result-md) |
| `docs/tasks/dsh-030-physical-follower.md` | [docs/history/dsh-030.md#src-docs-tasks-dsh-030-physical-follower-md](dsh-030.md#src-docs-tasks-dsh-030-physical-follower-md) |
| `docs/tasks/dsh-030-result.md` | [docs/history/dsh-030.md#src-docs-tasks-dsh-030-result-md](dsh-030.md#src-docs-tasks-dsh-030-result-md) |
| `docs/tasks/dsh-031-native-follower-protocol.md` | [docs/history/dsh-031.md#src-docs-tasks-dsh-031-native-follower-protocol-md](dsh-031.md#src-docs-tasks-dsh-031-native-follower-protocol-md) |
| `docs/tasks/dsh-031-result.md` | [docs/history/dsh-031.md#src-docs-tasks-dsh-031-result-md](dsh-031.md#src-docs-tasks-dsh-031-result-md) |
| `docs/tasks/dsh-032-policy-runtime.md` | [docs/history/dsh-032.md#src-docs-tasks-dsh-032-policy-runtime-md](dsh-032.md#src-docs-tasks-dsh-032-policy-runtime-md) |
| `docs/tasks/dsh-032-result.md` | [docs/history/dsh-032.md#src-docs-tasks-dsh-032-result-md](dsh-032.md#src-docs-tasks-dsh-032-result-md) |
| `docs/tasks/dsh-033-result.md` | [docs/history/dsh-033.md#src-docs-tasks-dsh-033-result-md](dsh-033.md#src-docs-tasks-dsh-033-result-md) |
| `docs/tasks/dsh-033-teacher-task.md` | [docs/history/dsh-033.md#src-docs-tasks-dsh-033-teacher-task-md](dsh-033.md#src-docs-tasks-dsh-033-teacher-task-md) |
| `docs/tasks/dsh-034-result.md` | [docs/history/dsh-034.md#src-docs-tasks-dsh-034-result-md](dsh-034.md#src-docs-tasks-dsh-034-result-md) |
| `docs/tasks/dsh-034-teacher-runtime-pilot.md` | [docs/history/dsh-034.md#src-docs-tasks-dsh-034-teacher-runtime-pilot-md](dsh-034.md#src-docs-tasks-dsh-034-teacher-runtime-pilot-md) |
| `docs/tasks/dsh-035-dataset-act-training.md` | [docs/history/dsh-035.md#src-docs-tasks-dsh-035-dataset-act-training-md](dsh-035.md#src-docs-tasks-dsh-035-dataset-act-training-md) |
| `docs/tasks/dsh-035-result.md` | [docs/history/dsh-035.md#src-docs-tasks-dsh-035-result-md](dsh-035.md#src-docs-tasks-dsh-035-result-md) |
| `docs/tasks/dsh-035-rework.md` | [docs/history/dsh-035.md#src-docs-tasks-dsh-035-rework-md](dsh-035.md#src-docs-tasks-dsh-035-rework-md) |
| `docs/tasks/dsh-036-act-policy-evaluation.md` | [docs/history/dsh-036.md#src-docs-tasks-dsh-036-act-policy-evaluation-md](dsh-036.md#src-docs-tasks-dsh-036-act-policy-evaluation-md) |
| `docs/tasks/dsh-036-result.md` | [docs/history/dsh-036.md#src-docs-tasks-dsh-036-result-md](dsh-036.md#src-docs-tasks-dsh-036-result-md) |
| `docs/tasks/dsh-036-rework.md` | [docs/history/dsh-036.md#src-docs-tasks-dsh-036-rework-md](dsh-036.md#src-docs-tasks-dsh-036-rework-md) |
| `docs/tasks/dsh-037-act-cadence-repair.md` | [docs/history/dsh-037.md#src-docs-tasks-dsh-037-act-cadence-repair-md](dsh-037.md#src-docs-tasks-dsh-037-act-cadence-repair-md) |
| `docs/tasks/dsh-037-result.md` | [docs/history/dsh-037.md#src-docs-tasks-dsh-037-result-md](dsh-037.md#src-docs-tasks-dsh-037-result-md) |
| `docs/tasks/dsh-038-ack-wakeup.md` | [docs/history/dsh-038.md#src-docs-tasks-dsh-038-ack-wakeup-md](dsh-038.md#src-docs-tasks-dsh-038-ack-wakeup-md) |
| `docs/tasks/dsh-038-result.md` | [docs/history/dsh-038.md#src-docs-tasks-dsh-038-result-md](dsh-038.md#src-docs-tasks-dsh-038-result-md) |
| `docs/tasks/dsh-039-act-motion-diagnosis.md` | [docs/history/dsh-039.md#src-docs-tasks-dsh-039-act-motion-diagnosis-md](dsh-039.md#src-docs-tasks-dsh-039-act-motion-diagnosis-md) |
| `docs/tasks/dsh-039-result.md` | [docs/history/dsh-039.md#src-docs-tasks-dsh-039-result-md](dsh-039.md#src-docs-tasks-dsh-039-result-md) |
| `docs/tasks/dsh-039-rework.md` | [docs/history/dsh-039.md#src-docs-tasks-dsh-039-rework-md](dsh-039.md#src-docs-tasks-dsh-039-rework-md) |
| `docs/tasks/dsh-040-filter-training-anchors.md` | [docs/history/dsh-040.md#src-docs-tasks-dsh-040-filter-training-anchors-md](dsh-040.md#src-docs-tasks-dsh-040-filter-training-anchors-md) |
| `docs/tasks/dsh-040-result.md` | [docs/history/dsh-040.md#src-docs-tasks-dsh-040-result-md](dsh-040.md#src-docs-tasks-dsh-040-result-md) |
| `docs/tasks/dsh-041-b16-20k-training-job.md` | [docs/history/dsh-041.md#src-docs-tasks-dsh-041-b16-20k-training-job-md](dsh-041.md#src-docs-tasks-dsh-041-b16-20k-training-job-md) |
| `docs/tasks/dsh-041-result.md` | [docs/history/dsh-041.md#src-docs-tasks-dsh-041-result-md](dsh-041.md#src-docs-tasks-dsh-041-result-md) |
| `docs/tasks/dsh-041-rework.md` | [docs/history/dsh-041.md#src-docs-tasks-dsh-041-rework-md](dsh-041.md#src-docs-tasks-dsh-041-rework-md) |
| `profiles/README.md` | [docs/history/dsh-018.md#src-profiles-readme-md](dsh-018.md#src-profiles-readme-md) |
| `profiles/robots/panda_stack_learning_v1/README.md` | [docs/history/dsh-034.md#src-profiles-robots-panda-stack-learning-v1-readme-md](dsh-034.md#src-profiles-robots-panda-stack-learning-v1-readme-md) |
| `third_party/README.md` | [docs/history/dsh-014.md#src-third-party-readme-md](dsh-014.md#src-third-party-readme-md) |
| `docs/simulation-loop-progress.md` | 按 531 行区间拆入 000～041，见[时间线](#timeline) |

旧 `docs/tasks/`、`docs/archive/`、`docs/dependencies/`、`docs/contracts/` 目录在内容归档并校验后移除；不再保留空壳跳转文档。

## 仿真 Docker 职责与准备流程

| 阶段 | 文档 | 内容 |
| --- | --- | --- |
| DSH-055 | [仿真 Docker 职责与操作流程简化](dsh-055.md) | 四个挂载、独立 SDK、依赖锁唯一来源、容器内业务入口；真实渲染、目标回归与 SIGINT/init 收尾验收。 |

## 固定预编译依赖

| 阶段 | 文档 | 内容 |
| --- | --- | --- |
| DSH-056 | [仿真直接安装固定 Ruckig wheel](dsh-056.md) | 移除编译 stage，单 FROM；固定产物/哈希/来源/许可证；285 项回归与实际场景、收尾验收通过。 |
| DSH-057 | [仿真项目依赖统一清单](dsh-057.md) | 沿用官方 ROS 环境，移除重复清单与安装步骤；65 项测试、实际构建/相机/后端收尾验收通过。 |
| DSH-058 | [删除仿真自定义镜像版本报告](dsh-058.md) | 删除生成器与三处入口的报告依赖；105 项测试和真实环境/场景/运动学/后端验收通过。 |
| DSH-059 | [Docker 通用命名与本地上下文](dsh-059.md) | requirements.lock、/opt/dependencies、本地 COPY；57 项测试、实际构建/场景及跨 cwd 解析通过。 |
| DSH-060 | [容器资产变量与冗余标记清理](dsh-060.md) | 通用资产变量、四个标记移除、运行 ENV 后置；138 项测试和真实功能验收通过。 |
