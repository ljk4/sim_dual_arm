# 文档索引

当前入口：[最小仿真闭环操作与结果](minimal-simulation-loop.md)，本轮结论见[采集、训练与固定20次评估结果](simulation-loop-result.md)。其余按设计、组件操作和历史验收查阅。

当前设计材料：

| 文档 | 内容 |
| --- | --- |
| [最小仿真学习闭环实施计划](minimal-simulation-loop-plan.md) | 已确认目标、模块边界、日志要求和 S1～S7 验收门槛 |
| [仿真闭环执行记录](simulation-loop-progress.md) | 当前阶段、决策、验收与限制 |
| [运行日志与原始记录约定](logging-and-recording.md) | 事件、故障、MCAP队列、v2结果与审计；顶层监督待完成 |
| [Episode记录格式](episode-recording-format.md) | v2三维结果、三相机audit、旧v1读取与人工结束语义 |
| [机器人 profile 与三相机绑定](../profiles/README.md) | v1/v2结构、双Panda 16维顺序、相机校准摘要算法与后端责任 |
| [仿真闭环讨论记录](simulation-loop-discussion.md) | 用户确认事项及开源调研来源 |
| [dsh 委派执行试验计划](dsh-collaboration-plan.md) | 主助手规划/审查、dsh 实施的试验流程与成本评价 |
| [学习训练操作手册](learning-training.md) | 变量/findmnt/df、真实 convert→verify→smoke/overfit/full→bundle、预算/恢复/日志、容器本机 venv 与跨机器 bootstrap |
| [学习数据转换、ACT 训练与离线模型包](learning-dataset-act.md) | DSH-035 45/5 独立数据集、train-only processor、预算训练、safetensors bundle 与公共命令 |
| [具名 ACT Policy 服务与固定种子评估](act-policy-service.md) | DSH-036 ACT adapter/入口、真实 bundle 校验、独立 Compose、pilot 1008/1009 与 test 20000..20019 台账/只读报告 |
| [DSH-001 试验结果](tasks/dsh-001-result.md) | 首次委派修复 R2 的独立验收、运行时间、用量及限制 |
| [DSH-002 修复结果](tasks/dsh-002-result.md) | 控制清理独立尝试、原实现回归验证与执行记录 |
| [DSH-003 修复结果](tasks/dsh-003-result.md) | 中断收尾、定点返工、ROS/Docker 验收与限制 |
| [DSH-004 配置整改结果](tasks/dsh-004-result.md) | S1a 类型配置、提前校验、快照隔离与独立验收 |
| [DSH-005 控制客户端结果](tasks/dsh-005-result.md) | S1b 控制状态所有权、续租/清理提取与独立验收 |
| [DSH-006 episode 提取结果](tasks/dsh-006-result.md) | S1b 生命周期、线程所有权、连续运行与中断回归 |
| [DSH-007 日志基础库结果](tasks/dsh-007-result.md) | 有界事件队列、文件轮转、并发故障复验与局部接管 |
| [DSH-008 runtime 日志结果](tasks/dsh-008-result.md) | 命令和停止事件、日志故障停止、配置快照、真实ROS/镜像验证 |
| [DSH-009 MCAP 队列与审计结果](tasks/dsh-009-result.md) | 慢盘不阻塞控制、有界记录、故障持久化与内存实测 |
| [DSH-010 结果分类与人工结束](tasks/dsh-010-result.md) | v2迁移、历史兼容、停止后的终止ACK与故障优先级 |
| [DSH-011 Compose 开发挂载](tasks/dsh-011-result.md) | 源码与配置挂载、契约生成保护、实际容器验收及暂停位置 |
| [DSH-012 人工操作命令](tasks/dsh-012-result.md) | up/start/status/end/down、指定 episode 等待、容器身份与失联验收 |
| [DSH-013 自动采集与整次监督](tasks/dsh-013-result.md) | 真实退出、审计/日志门槛、共享期限、资源归属、原始声明与验证分离 |
| [阶段 1～2 可维护性审查](maintainability-review.md) | 已复现问题、目录与职责评估、阶段 2R 整改顺序；R1～R3 已修复 |
| [阶段 1～2 使用与验证](stage2-quickstart.md) | 实际构建/启动命令、MCAP 输出、测试与维护规则 |
| [逐步实现路线图](implementation-roadmap.md) | 推荐实现顺序、每阶段产物、完成标准和首批工作 |
| [目录划分与组织规范](repository-layout.md) | 仓库分类、配置层次、构建输出、数据产物与扩展规则 |
| [环境边界与接口设计](architecture-and-interfaces.md) | 四类环境职责、Robot/Policy/Runtime API、时钟与控制权 |
| [仿真环境检查](simulation-environment.md) | S2a pinned MuJoCo/mink 依赖/资产/EGL 检查，以及 S2b 三相机场景可见性入口 |
| [双 Panda 堆叠场景](panda-stack-scene.md) | S2b 双 Panda/柱/桌/两块/三相机配置、构建、确定性复位与固定外参 |
| [16维SI执行器与测量映射](panda-io.md) | DSH-020 具名qpos/dof/actuator绑定、夹爪总开度转换、线程约束与真实测量边界 |
| [定时参考轨迹与物理执行边界](reference-trajectory-design.md) | DSH-021 分段三次系数契约、无SciPy采样/连续极值验证、SciPy一次性编译；DSH-023 显式起止参考速度与严格区间采样 |
| [连续动作块：时间、衔接与反馈](continuous-action-design.md) | DSH-023 观测窗口与预编译边界、DSH-024 有界连续执行；ROS/Policy现已接通，当前操作见闭环入口 |
| [无ROS物理运动、接触完成与有界停止](native-motion.md) | DSH-022/024 严格控制配置、真实步进参考/测量快照、双指接触完成、常加速度刹车、有界连续块与停钟/进度/跟踪故障 |
| [原生控制会话、幂等命令与复位](native-control.md) | DSH-025 控制身份/共享 generation、显式租约、整体命令与块幂等、有界历史、聚合状态和 episode 复位 |
| [可运行的原生 ROS 后端](native-backend.md) | 物理owner、规划/渲染、原生ROS服务和公共Compose |
| [示教运行时、任务收尾与有预算试采入口](native-teacher-runtime.md) | DSH-034 teacher/policy 统一 stream、任务真实 Stop/冻结事实、标签/质量导出与 pilot/formal 台账 |
| [原生诊断记录](native-recording.md) | 精确捕获、租约/ACK、MCAP v3与独立审计 |
| [原生受限跟随执行](native-follower.md) | 同一控制器、连续接续、2ms网格及可审计边界规范化 |
| [受限跟随公共协议与ROS后端](native-follower-protocol.md) | DSH-031 独立FollowerReference、SubmitFollower、可选后端配置与固定Ruckig配方 |
| [DSH-030验收](tasks/dsh-030-result.md) | 真实物理/ROS回归、双向边界与旧模式指纹 |
| [受限目标跟随](bounded-target-follower.md) | 原预测/实际参考分离、端速与连续偏差验证 |
| [DSH-029验收](tasks/dsh-029-result.md) | 严格模式数学反例、独立随机对照和待物理边界 |
| [DSH-028验收](tasks/dsh-028-result.md) | 真实ROS、故障、旧格式兼容及公共入口证据 |
| [原生参考与反馈公共类型](native-protocol.md) | DSH-026 NativeRequest/AcceptedReference/反馈DTO、canonical digest、显式ROS双向转换与独立重建；无服务实现 |
| [DSH-016 三相机验收](tasks/dsh-016-result.md) | 67 个初始布局、PNG、原生像素审计和关闭故障验证 |
| [DSH-017 场景阶段验收](tasks/dsh-017-result.md) | IK、关键位姿、采样净空、逐阶段视野与独立审计 |
| [DSH-018 三相机记录验收](tasks/dsh-018-result.md) | v2 profile、16维绑定、三相机mock/MCAP与暂停位置 |
| [DSH-019 命令状态查询、执行时间与反馈记录](tasks/dsh-019-command-status.md) | CommandStatus/QueryCommand、采样/执行时钟分离、feedback MCAP 审计与 mock 边界 |
| [DSH-019命令与反馈验收](tasks/dsh-019-result.md) | 时钟分离、查询、原生ROS故障、轨迹审计和历史兼容 |
| [DSH-020原生执行器验收](tasks/dsh-020-result.md) | 16维SI、真实测量/ctrl、声明标定、线程与原生动态对照 |
| [DSH-021参考轨迹验收](tasks/dsh-021-result.md) | 固定SciPy、连续约束、随机独立对照和无科学依赖验证 |
| [DSH-022原生运动验收](tasks/dsh-022-result.md) | 仿真/墙钟分离、真实接触、减速停止、原生重放和停止边界 |
| [DSH-023连续计划验收](tasks/dsh-023-result.md) | 显式速度边界、观测锚定窗口、刹车余量与独立对照 |
| [DSH-024连续执行验收](tasks/dsh-024-result.md) | 真实跨块衔接、事件身份、受控停止、接触与期限故障及独立物理重放 |
| [DSH-025控制会话验收](tasks/dsh-025-result.md) | 租约、幂等/有界查询、真实停止、reset/epoch、关键事件保留与跨episode兼容 |
| [DSH-026公共原生协议验收](tasks/dsh-026-result.md) | 完整请求、显式系数参考、真实反馈/接触、CDR往返和独立重建 |
| [S3 接口实施前核对](simulation-control-design.md) | 现有单相机/控制/观测缺口与下一阶段依赖顺序 |
| [关键位姿、IK、路径净空与视野检查](kinematic-scene-check.md) | DSH-017 的 67 布局右臂 6 目标、mink/DAQP、冻结、离散净空和三相机视野 |
| [环境与依赖分析](docker-dependency-analysis.md) | 镜像依赖归属、候选技术栈和运行组合 |
| [仿真环境设计](simulation-design.md) | 历史 Gazebo 候选分析及公共仿真边界；当前路线见最小闭环计划 |

唯一维护的协议与部署草案：

- [Policy API v1](../contracts/policy/v1/policy.proto)
- [仿真采集部署绑定示例](../deploy/examples/sim_collect.yaml)

历史材料：archive 中的早期 Humble 方案，以及 dependencies 中的 Python 3.10 解析快照。它们保留用于追溯，不能当作当前安装基线。docs/contracts 的旧文件路径仅保留迁移提示。
