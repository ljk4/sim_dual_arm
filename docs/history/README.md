# 项目历史（压缩版）

本目录只保留项目演进中仍然有决策价值的内容。旧版逐阶段 DSH 原文、迁移期的
SHA256 分块、逐字时间线和临时命令已移出工作区；需要追溯原文时，可通过 Git
历史查看压缩前的文件，例如：

```bash
git log -- docs/history/dsh-048.md
git show <commit>:docs/history/dsh-048.md
```

当前使用入口、配置说明和故障排查以根 [README](../../README.md) 和
[项目状态](../project-status.md) 为准。历史文档中的旧路径、旧预算和旧命令只反映
当时状态，不代表现在可直接执行。

## 当前状态

- 已打通 MuJoCo 双 Panda、runtime、Policy 服务、ACT 推理、录制和审计链路。
- 数据侧固定为 45 条训练、5 条验证 episode；训练使用 ACT，Policy 通过 gRPC 提供
  60×16、50 ms 的动作块。
- 历史上 20K 模型在种子 30600 的同场景修复回归中完成过一次稳定堆叠：25.85 s
  完成释放、退开并稳定支撑 2 s；该结果不是独立泛化成功率，右腕遮挡质量门禁也未全过。
- 旧 20K bundle 已删除；当前工作区只有新训练的 5K 模型。5K 模型已能跑通链路，
  但仍会出现 `FOLLOWER_DEVIATION_EXCEEDED` 和夹持阶段 `TRACKING_TIMEOUT`。
- 运行端现在支持 `interactive`：每次运行生成唯一身份，输入 `y` 后通过
  `StartEpisode` 复位环境并执行一局，任务结束后按 `Ctrl-C` 退出。
- 仿真端增加了独立实时 MuJoCo viewer，通过只读完整物理快照显示机器人、物体和任务状态。

## 阶段主线

### S1：先把运行身份、日志和录制做可靠

早期工作集中在不可变 typed config、控制客户端与 episode 生命周期拆分、结构化
事件日志、有界 MCAP 写入、结果分类、Compose 挂载和自动采集监督。关键思想是：
运行身份、记录完整性、runtime 结果和任务结果必须分开表达；慢盘、日志故障和
人工停止都不能悄悄破坏控制链。

### S2：固定仿真环境与场景

固定依赖 wheel、Panda 资产和 MuJoCo 版本，建立双 Panda 场景、三路相机、确定性
reset、IK/净空检查与可见性门禁。场景、控制和任务参数从单一 YAML 源读取，避免
数组相邻假设和隐式资产下载。

### S3：原生控制与动作块执行

逐步引入 16 维 SI 执行器映射、分段三次参考、真实物理步进、接触完成、动作窗口、
连续动作块、租约/幂等命令、受限目标跟随、ROS 后端和 Policy RPC。最终链路为：

```text
Policy/gRPC -> runtime -> ROS FollowerReference -> MuJoCo owner -> 实测反馈
```

所有预测、准入目标、受约束参考和实测反馈分开保存，不以离线反事实代替物理成功。

### S4～S5：示教采集与训练数据

增加独立任务判定、程序示教、双指接触聚合、真实 Stop/冻结事实、逐帧 RGB/参考导出、
质量门禁、45/5 数据集、train-only 统计、追加式训练预算和 safetensors bundle。
这一阶段确立了“采集导出与学习格式转换分离”的边界，便于审计和重新转换。

### S6～S7：真实 ACT 推理与执行修复

先完成真实 ACT Provider 和固定种子闭环，随后发现多类执行问题：

- 短执行窗口会误判模型“静止”，需要结合完整 60 点预测判断；
- 新旧预测在绝对仿真时间上的不一致会触发偏差拒绝，加入时间融合；
- reset 后立即推理偏离训练分布，加入有界稳定等待；
- 动作权重平均可能出现浮点越界，融合结果必须保持在参与值凸包内；
- Ruckig 极小制动前缀、夹持数值静止和低力接触分类需要针对性修复。

最重要的实测结果是：原固定 20 种子在早期配方下物理成功率为 0/20；改进配方后，
种子 30600 的同场景修复回归完成了一次真实堆叠，但质量门禁与跨种子泛化仍未通过。

### S8：环境与入口收敛

DSH-052～060 将仓库拆成 simulation/runtime/learning/hardware 四端，收敛 Dockerfile、
docker-compose.yml、run.sh、SDK 生成、依赖锁和挂载关系。默认运行方式变为：

- 三端各自构建和启动 idle 容器；
- runtime 显式生成 ROS/Policy SDK；
- simulation 负责场景、物理、渲染和 viewer；
- learning 负责转换、训练、bundle 和 Policy 服务；
- runtime 负责观测、控制、录制和交互式运行。

## DSH 索引与关键结论

| 编号 | 压缩后的关键结论 |
| --- | --- |

### 设计基线

| 编号 | 压缩后的关键结论 |
| --- | --- |
| <a id="dsh-000"></a>DSH-000 | 汇总早期环境边界与 Robot/Policy/Runtime 接口、仓库目录规范、逐步实现路线图、仿真环境与最小闭环方案、容器依赖分析，以及当时的文档索引和旧路径迁移提示。 |

### S1：基础架构与可审计运行

| 编号 | 压缩后的关键结论 |
| --- | --- |
| <a id="dsh-001"></a>DSH-001 | 阶段 2R 的首个委派试验，修复 ObservationBuffer 在同一 observation ID 下样本引用漂移：首次成功组装后固定 state 与相机引用，迟到合法帧只能供后续 state 使用。 |
| <a id="dsh-002"></a>DSH-002 | R3 控制清理修复：关闭续租后依次尝试在途续租、停止和释放，前一步失败不跳过后继，错误保留步骤名；清理消耗本地 lease，重复调用不重复发 RPC。 |
| <a id="dsh-003"></a>DSH-003 | R1 中断收尾修复：信号处理器只记录首个 SIGINT/SIGTERM，主线程停止接收新 episode，worker 停止动作、释放控制权并关闭记录后才销毁 ROS 资源；普通完成、运行失败、两种信号和收尾超时使用不同退出码。 |
| <a id="dsh-004"></a>DSH-004 | S1a 类型配置与启动前校验：不可变 typed config、来源路径和边界校验，避免错误值被接受或配置快照被污染。 |
| <a id="dsh-005"></a>DSH-005 | S1b 控制客户端提取：`ControlClient` 独占控制 RPC/action、lease、续租和清理状态，node 不再重复持有；故障回调在锁外执行，清理步骤独立尝试并汇总。 |
| <a id="dsh-006"></a>DSH-006 | S1b episode 生命周期提取：`EpisodeRunner` 统一拥有 worker、停止/关闭请求、context、recorder、记录路径和失败汇总，node 保留 ROS 注册、消息转换、观测同步和进程入口。 |
| <a id="dsh-007"></a>DSH-007 | S1c 公共事件日志库：新增 `robot_run_logging` 标准库包与事件 schema，提供有界队列、JSONL 轮转、关联 ID、严格载荷、健康检查和有界关闭。 |
| <a id="dsh-008"></a>DSH-008 | S1c runtime 日志接线：不可变 `LogSettings`、run 目录独占、配置/版本来源快照、episode/命令/阶段/停止事件关联，日志故障阻止继续执行，stop/release 仍独立尝试；关闭前记录 `run.finalizing`，关闭后输出进程结果。 |
| <a id="dsh-009"></a>DSH-009 | S1c 有界 MCAP 写入与流式审计：单 writer 线程、条数与字节双重有界队列、慢盘不阻塞控制、故障持久化；审计按条读取，只保留引用元数据并按 1 MiB 分块哈希。 |
| <a id="dsh-010"></a>DSH-010 | S1c 结果分类与人工结束：新 episode 写 v2 manifest，分开表达记录完整性、runtime 结果和任务标签；人工 End 返回 ENDING 确认，保留首次原因并有界等待已知命令终态，历史 v1 严格读取且不原地改写。 |
| <a id="dsh-011"></a>DSH-011 | S1d Compose 开发挂载：runtime 与 mock 后端共用稳定 ROS 依赖镜像，源码/配置/生成契约只读挂载，原始数据与日志直接写宿主，容器按调用者 UID/GID 运行；契约从 `contracts/ros` 单一来源生成并校验来源摘要。 |
| <a id="dsh-012"></a>DSH-012 | S1d 人工操作命令：`up/status/start/end/down` 可在无宿主机 ROS 时操作 Compose 开发栈；主机入口核对项目、容器、run id、镜像和挂载归属，分离 PREPARING/ENDING 服务 ACK 与最终 manifest，按 episode id 等待并保留 recording/runtime/task 三维结果。 |
| <a id="dsh-013"></a>DSH-013 | S1d 自动采集与整次运行监督：统一 Compose 入口，JSON 启动器配方集中管理工作期限、停止/清理预算、诊断和日志容量；一次采集由真实进程退出码、独立 MCAP 审计、关键日志排空和清理结果共同判定，并原子写 `status.json`。 |

### S2：MuJoCo 双 Panda 场景

| 编号 | 压缩后的关键结论 |
| --- | --- |
| <a id="dsh-014"></a>DSH-014 | S2a 仿真环境门槛：固定 15 个官方 wheel 版本与 SHA-256，基于 Python 3.12/Jazzy/NumPy 1.26.4 新增 `mujoco-dev` target；Panda 资产按固定 Menagerie 提交 fetch/verify，一次性 `sim_check.py` 检查依赖、资产、单 Panda step、QP 和 NVIDIA EGL。 |
| <a id="dsh-015"></a>DSH-015 | S2b 双 Panda 场景模型：场景参数集中到 `configs/simulation/panda_stack.yaml`，原生 MjSpec 挂接固定 Menagerie Panda，包含柱、两侧 1 m 基座、0.75 m 桌面、两个自由方块和三个固定相机；保留上游网格、动力学、tendon 和执行器，具名索引避免依赖数组相邻，并提供确定性复位。 |
| <a id="dsh-016"></a>DSH-016 | S2 三相机静态可见性：增加场景快照、可见性检查和一次性检查入口，覆盖 67 个初始布局、PNG、原生像素审计和关闭故障验证。 |
| <a id="dsh-017"></a>DSH-017 | S2 关键位姿、IK、路径净空与三相机视野检查：数值求解只影响 IK 搜索，实际路径仍从 home 开始；冻结关键位姿、离散净空并逐阶段核对三相机视野。 |

### S3：原生控制、动作块与 Policy 闭环

| 编号 | 压缩后的关键结论 |
| --- | --- |
| <a id="dsh-018"></a>DSH-018 | S3 三相机 profile、runtime 接线与原始记录：v2 具名相机集合、16 维关节顺序、三相机 mock/MCAP 与 audit，19 个允许文件整合。 |
| <a id="dsh-019"></a>DSH-019 | 命令状态查询、执行时间与反馈记录：新增 `CommandStatus`、`QueryCommand` 和独立 `command_feedback` 流，分离采样/执行时钟，记录接受、开始、反馈和终态；mock 测量仍等于参考，物理执行尚未接入。 |
| <a id="dsh-020"></a>DSH-020 | 双 Panda 16 维 SI 执行器与测量映射：具名 qpos/dof/actuator 绑定、夹爪两指总开度转换、线程约束、声明标定与有效姿态区分，以及真实测量边界。 |
| <a id="dsh-021"></a>DSH-021 | 可验证的分段三次参考计划：标准库共享不可变计划、采样、连续极值与约束检查，仿真侧用固定 SciPy 1.17.1 一次性编译；保留原始路点位置和时间，拒绝区间内部过冲及速度/加速度超限。 |
| <a id="dsh-022"></a>DSH-022 | 无 ROS 物理运动、双指接触完成与有界停止：严格控制配置、真实步进参考/测量快照、常减速刹车和单运动状态机；双指接触分别判定，停止边界和原生重放可审计。 |
| <a id="dsh-023"></a>DSH-023 | 连续参考边界与观测锚定动作窗口：编译/验证接受显式起止参考速度（默认仍为零端点），连续块使用严格区间采样；共享 `ActionWindow` 按观测捕获时间选择未来窗口，保留完整源、索引和重叠计数，并加入刹车余量。 |
| <a id="dsh-024"></a>DSH-024 | 有界连续动作块的原生执行：真实跨块衔接、事件身份、受控停止、接触与期限故障，以及独立物理重放。 |
| <a id="dsh-025"></a>DSH-025 | 原生控制会话：纯标准库 `ControlCommand` 与共享 generation 比较，`NativeController` 组合既有 `NativeMotion`，提供租约、整体命令/块幂等、有界查询、真实停止和 episode 边界 reset；期限用墙钟，参考与测量用仿真时间。 |
| <a id="dsh-026"></a>DSH-026 | 原生参考与反馈公共类型：DTO 保存完整原请求、显式分段系数、来源观测/选中窗口、接触证据和真实反馈；原生侧只做预编译与转换，ROS 侧显式编解码。 |
| <a id="dsh-027"></a>DSH-027 | 可运行的 ROS 原生 MuJoCo 后端：真实 ROS 服务可执行运动和连续块，三图与测量使用同一快照。 |
| <a id="dsh-028"></a>DSH-028 | 原生诊断运行时与可审计 v3 MCAP：精确捕获、租约/ACK、v3 manifest、独立审计和故障处理；旧回归中把未知版本硬编码为 3 的测试夹具修正为 99，不放宽拒绝规则。 |
| <a id="dsh-029"></a>DSH-029 | 受限目标跟随纯数学层：`follower.py` 保存不可变计划与验证，`follower_math.py` 提供通用端速约束和连续偏差极值，均只依赖标准库；单窗口 Ruckig 计算与显式分段系数导出分开。 |
| <a id="dsh-030"></a>DSH-030 | 受限跟随接入同一原生物理执行器：三块独立运动仿真、参考跟踪误差、公开 `NativeExecution` DTO、原系数与应用参考、测量及应用/采样时钟逐步核对；旧 MOTION 路径保留。 |
| <a id="dsh-031"></a>DSH-031 | 受限跟随公共协议与 ROS 后端：新增独立 `FollowerReference`/`SubmitFollower`，复用同一物理 owner、规划器、真实反馈和块历史；原 31 个 msg/srv/action 逐字不变，严格 `SubmitNative` 保留，配置显式启用才声明能力。 |
| <a id="dsh-032"></a>DSH-032 | 连续 Policy RPC、受限执行与 MCAP v4：真实 gRPC→ROS→MuJoCo 连续 8 块→v4→独立审计，3 类语义篡改被拒绝；修正整体/每块身份混用、停止时活动块漏终态和跨 topic 收尾竞态。 |

### S4～S5：任务判定、示教采集与训练

| 编号 | 压缩后的关键结论 |
| --- | --- |
| <a id="dsh-033"></a>DSH-033 | 独立堆叠判定与可选程序示教：任务状态机、双指接触聚合、真实 Stop/冻结事实、任务失败日志和官方 SDK 提升；30 个文件整合，1030 项业务回归及真实 ROS/CDR 两模式停止/重置通过。 |
| <a id="dsh-034"></a>DSH-034 | 示教运行时、任务收尾与有预算试采：teacher 与 policy 共用 stream、连续 follower/ACK/插值/反馈/受控停止链，导出逐帧 RGB/执行参考和质量标签。 |
| <a id="dsh-035"></a>DSH-035 | 数据转换、ACT 训练与离线模型包：45/5 独立数据集、train-only processor、追加式预算台账、safetensors bundle、CPU 合成检查、容器与本机 venv 入口。 |

### S6～S7：真实 ACT 评估与执行修复

| 编号 | 压缩后的关键结论 |
| --- | --- |
| <a id="dsh-036"></a>DSH-036 | 真实 ACT PolicyProvider 与固定种子评估：具名 ACT adapter/service、真实 bundle 校验、独立 Compose、pilot 1008/1009 与 test 20000..20019；20/20 可信执行闭环，物理堆叠 0/20。 |
| <a id="dsh-037"></a>DSH-037 | ACT 重规划周期修复：从 20 例原始 MCAP 重建被拒绝请求，独立实现 2 点 ACT 配方、拒绝证据和公共 backend 参数；原速 3 次、半速 1 次实测仍在 2 块后迟到/断流，失败保留在独立台账。 |
| <a id="dsh-038"></a>DSH-038 | ROS ACK 等待立即唤醒：两个客户端用 Future 完成回调设置 Event，保留原超时、停止、取消、异常、ACK 拒绝和清理规则。 |
| <a id="dsh-039"></a>DSH-039 | ACT 原始输出静止倾向独立诊断：关联 289 条原始预测，287 条有精确源观察；完整 60 步接近静止的块为 0，但实际常选 200/250 ms 低位移段。 |
| <a id="dsh-040"></a>DSH-040 | 静止冗余训练起点筛选：只过滤当前观察起点，不删除原始帧、不改变 50 ms 时间槽或未来 60 步动作；19676 个训练起点保留 19670、排除 6 个退开阶段起点，验证 5 条 2160 帧不变，并接入 sampler/checkpoint/bundle 来源绑定。 |
| <a id="dsh-041"></a>DSH-041 | batch16 / 20K 新训练配方与独立训练作业：完成部署文件、真实 GPU 预检和后台启动；首轮 600 秒限时退出 124，集中返工修正文档墙钟估时和无效线程环境变量。 |
| <a id="dsh-042"></a>DSH-042 | 文档职责收敛、历史归档与迁移验收；本文件由主助手维护。 |
| <a id="dsh-043"></a>DSH-043 | 移除业务 COPY、提取内嵌代码、SDK 来源检查与实际容器验收。 |
| <a id="dsh-044"></a>DSH-044 | 模型包与真实验证观察检查通过；新种子单次执行 82 块后因 CHUNK_LATE 停车，未堆叠成功，显存足够；保留视频与失败证据。 |
| <a id="dsh-045"></a>DSH-045 | 1s窗口恢复接近动作；新旧预测切换仍触发偏差拒绝，继续分析。 |
| <a id="dsh-046"></a>DSH-046 | 保持后端静止判定，按原总预算等待reset稳定，20项真实ROS回归通过。 |
| <a id="dsh-047"></a>DSH-047 | 模型与会话边界测试通过；真实融合与数值凸包修复已验收。 |
| <a id="dsh-048"></a>DSH-048 | 完整保留探索失败；30600同场景回归稳定堆叠成功，图像质量项未全过。 |
| <a id="dsh-049"></a>DSH-049 | 显式等待2s仿真稳定，避免reset极早期测量离开训练分布；29项目标回归通过。 |
| <a id="dsh-050"></a>DSH-050 | 修复22飞秒前缀误拒绝，41项真实Ruckig回归与16DOF失败重建通过。 |
| <a id="dsh-051"></a>DSH-051 | 与原卸载逻辑统一1e-9m/s数值容差，17项回归通过；夹持误停修复后完成堆叠。 |

### S8：目录、Docker 与运行入口整理

| 编号 | 压缩后的关键结论 |
| --- | --- |
| <a id="dsh-052"></a>DSH-052 | 迁移仿真/运行/学习实现、配置、Docker、Compose 与脚本；路径与只读回归通过，容器和集成验证留待 053。 |
| <a id="dsh-053"></a>DSH-053 | 宿主唯一 `scripts/env.sh`；三端 idle Compose 与容器内 `run.sh`；learning 生成真实 bundle 绑定、runtime 消费通用 JSON；旧宿主启动器/overlay 清理，静态与有限纯回归通过。 |
| <a id="dsh-054"></a>DSH-054 | 每实现一份 Dockerfile/docker-compose.yml/run.sh；根 Compose 只 include；env.sh 转发 run.sh 功能；旧入口、多 target 和宿主 venv 路线清理；候选静态与纯回归通过，实际构建/容器验收由主助手独立执行。 |
| <a id="dsh-055"></a>DSH-055 | 四个挂载、独立 SDK、依赖锁唯一来源、容器内业务入口；真实渲染、目标回归与 SIGINT/init 收尾验收。 |
| <a id="dsh-056"></a>DSH-056 | 移除编译 stage，单 FROM；固定产物/哈希/来源/许可证；285 项回归与实际场景、收尾验收通过。 |
| <a id="dsh-057"></a>DSH-057 | 沿用官方 ROS 环境，移除重复清单与安装步骤；65 项测试、实际构建/相机/后端收尾验收通过。 |
| <a id="dsh-058"></a>DSH-058 | 删除生成器与三处入口的报告依赖；105 项测试和真实环境/场景/运动学/后端验收通过。 |
| <a id="dsh-059"></a>DSH-059 | requirements.lock、/opt/dependencies、本地 COPY；57 项测试、实际构建/场景及跨 cwd 解析通过。 |
| <a id="dsh-060"></a>DSH-060 | 通用资产变量、四个标记移除、运行 ENV 后置；138 项测试和真实功能验收通过。 |

## 保留的失败与限制

这些历史事实必须保留，不能用后续成功或重新运行覆盖：

- 早期 ACT 固定 20 个测试种子：执行链可信，但物理堆叠 0/20。
- DSH-036 的 20/20 指执行与反馈闭环完成，不代表任务成功。
- DSH-048 的 30600 成功是**同场景修复回归**，不能当作独立泛化成功率。
- 30600 成功时 `task_success=true`、`success=true`，但右腕视角遮挡导致
  `quality_qualified=false`、`run_success=false`。
- 5K 模型当前仍可能出现动作偏差、夹持跟踪超时和释放阶段振荡；失败录像和日志
  必须保留，不要为了得到成功结果覆盖旧记录。
- 运行重试必须使用新的 `run_id`、`episode_id` 和输出目录；`interactive` 已自动
  处理身份，但手工命令仍可能触发 `FileExistsError` 或 `REQUEST_ID_CONFLICT`。

## 关键技术经验

1. 先定位最早失败事件；`STREAM_ANCHOR_UNAVAILABLE` 可能只是运动已停止后的下游
   结果，真正原因常在后端的 `TRACKING_TIMEOUT`、拒绝事件或超时日志中。
2. 预测、准入、融合、参考、实测和任务判定是不同层，不能用一个大容差掩盖。
3. 配置生成和审计 manifest 是必要的身份校验；用户应改源模板，不直接改
   `/data/prepared/...` 下的生成文件。
4. 仿真、训练和运行容器保持边界；viewer 只读物理快照，不进入控制链。
5. 任何历史成功都要连同模型、种子、配置、录制和质量门禁一起报告。

## 当前补丁（未单独编号）

在 DSH-060 之后，工作区又完成了以下运行体验改进：

- runtime 新增 `interactive` 命令：自动生成唯一 run/episode 身份，`y` 后复位并
  执行一局，结束后等待 `Ctrl-C`。
- 仿真端新增 `scripts/run.sh viewer`：独立进程订阅完整 MuJoCo 状态，不步进物理。
- 三端 Compose 增加 viewer 所需的 X11 可选挂载；无图形环境仍可运行 backend。
- 所有源 YAML、env 和 Compose 文件增加中文参数说明；JSON schema/profile 因 JSON
  不允许注释而保持原样。
- 根 README 改为从 `git clone` 开始，命令按单条拆分并说明含义。
