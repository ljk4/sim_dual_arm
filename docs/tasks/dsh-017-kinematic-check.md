# DSH-017：S2 关键位姿、IK、路径净空与视野检查

2026-10-31。前置 DSH-016 已独立验收并整合。用户要求继续用本地 headless dsh。当前任务完成运动学诊断，不接入 ROS，不执行物理抓取，不创建正式数据。

## 输入与已知证据

- 使用当前工作树的固定 MuJoCo 3.17.0、mink 1.3.0、DAQP、Pillow 和 Panda 资产；镜像、SDK、摘要由监督者只读挂载，禁止下载/安装或调用 Docker。
- `Scene` 提供真实双臂/桌/柱/两块模型、具名索引和确定性 reset；`SceneRenderer` 提供私有数据快照三视图。复用它们，不另写机器人模型或渲染栈。
- DSH-016 默认 3 seeds + 64 XY/yaw 联合边界布局，全部初始三视图通过。复用 `visibility.build_layouts`，不缩小随机范围。IK/路径全覆盖同一 67 布局。
- 监督者已对 seed11 做非执行设计实验：固定左臂和物体自由度，mink `FrameTask + PostureTask`、`ConfigurationLimit + CollisionAvoidanceLimit`、`DofFreezingTask` 硬约束，DAQP 可解 6 个目标；线性关节路径未发现穿透，但没有正净空认证。不得把该实验当成验收结果。
- mink 1.3 提供 `DofFreezingTask(model, dof_ids)`，必须传 `solve_ik(..., constraints=[freeze])`，不能仅作为软任务。right_tcp 是已存在的 site；只允许更新右臂 7 个 hinge DOF，左臂、左右指、两物体均保持初始值。
- 右 TCP 方向候选为世界向下 `Rz(source_yaw) @ diag(1,-1,-1)`，整个序列保持此方向；左臂待机。不是控制器速度/执行时间设计，IK dt 不能冒充运动周期。

## 文件范围与预算

允许新增：
- `configs/simulation/kinematics_check.yaml`：有型、冻结、来源明确的检查参数；
- `components/backend_sim/src/robot_backend_sim/kinematics_config.py`：严格配置解析；
- 同目录 `kinematics.py`：私有状态上的目标生成、mink IK、几何净空和采样路径诊断；
- 同目录 `check_kinematics.py`：日志/报告/PNG 的薄编排；
- `tests/unit/test_kinematics_config.py`、`tests/integration/test_scene_kinematics_native.py`；
- `docs/kinematic-scene-check.md`。

允许修改：
- `tools/run/sim_check.py`、`deploy/compose/sim_check.yaml`、`tests/unit/test_sim_environment.py`：固定 `--check kinematics` 和 `--kinematics-config` 参数/RO 挂载；environment/scene 兼容；
- README、docs/README、docs/panda-stack-scene.md、docs/simulation-environment.md 的入口与边界说明。

如确实需要共享检查辅助函数，允许新增 `diagnostics.py` 并从 `check_scene.py` 提取小型无业务依赖的原子 JSON、源码摘要、有效场景配置文档等函数；保留原测试覆盖和行为，不整体重写已验收入口。不从 tools 导入业务，也不跨模块调用大量私有函数。无需为每个数据类新建文件或抽象插件框架。

本任务不修改 `Scene`、`SceneRenderer`、场景外参/几何/随机化、日志库、ROS/runtime/协议、镜像或依赖锁。禁止修改其他工作树、Git 提交或启动其他代理。首轮 25 分钟，最多一次 10 分钟定点返工；一次必要自测后交付，不循环探索已有缓存。交付 `.build/reports/dsh-017/report.md`，说明真实测试、未完成项与诊断限制。

## 运动学与净空要求

1. 显式类型表达目标、解和路径结果；输入来自完整初始快照。IK/几何查询使用私有 `mink.Configuration`/MjData，不写物理 Scene.data。只更新 7 个右臂关节；验证 frozen qpos 不变（四元数允许数值舍入），有限值、关节范围、残差与明确收敛状态。
2. 配置初值：IK dt .01 s、最多 250 迭代/目标、位置误差 .0005 m、旋转误差 .005 rad、posture cost .0005、frame cost 1/1、LM damping .001、collision safety .002 m、检测距离 .03 m。采用 mink/DAQP，不实现伪逆 IK。失败/不可达不可默默跳过。
3. 6 个配置目标，source/target 表示初始物体中心：pregrasp=source+[0,0,.12]，grasp_pose=source+[0,0,.004]，lift_pose=source+[0,0,.20]，place_hover=target+[0,0,.16]，place_pose.z=table_top + 2*target_half_z + source_half_z + .006（XY 为 target），retreat=target+[0,0,.20]。升降/放置高度均配置化，不在脚本散布魔数。
4. 两块始终留在初始位置；夹爪始终开启。因此 lift/place 只是空手关键姿态预览，不能声称物体已经搬运或完成堆叠。S4 再验收真实接触/抓持物体的完整路径。
5. 明确枚举机器人自碰撞、双臂互碰、机器人与柱/支架/桌/物体的有效物理 geom 对。复用 mink/原生碰撞过滤规则，忽略无碰撞 visual geom、同 welded body、原生允许忽略的相邻体/显式 exclude；记录对数和排除依据，不用“全部机器人碰撞关闭”通过。固定连接处接触不能当作自由运动净空。
6. 初始姿态及每相邻关键位姿的关节线性路径均检查；最多相邻采样 .01 rad，包含两端，单段最多 2000 样本。报告最小净空与对应 layout/segment/sample/geom pair（含具名 body），要求 >= .002 m。可使用 `mj_geomDistance` 有限检测距离及有根据的保守 broadphase，不能把没有穿透 contact 当作正净空证明。处理无法测量/非有限距离并失败，不返回伪造大距离。
7. 另检查采样关节限位、冻结自由度和有限值；耗时、迭代数、样本数有界。采样检查不等于连续几何碰撞证明，报告明确注明离散分辨率；轨迹插值与执行约束在 S3 实现。不要改物理 margin 或删除碰撞几何来通过检查。

## 关键视角与可审计输出

对 67 布局的 6 个关键姿态生成来自同一诊断快照的三路 RGB PNG，至少一张带 layout/phase/camera 标注的概览。不要在前后截图间搬动物体；记录 source/target/table 的实际 geom ID 分割像素。所有布局初始三视图门槛仍归 scene 检查，本任务不能降低它。

运动过程中左腕目标会被右臂短暂遮挡，不能要求全部相机始终看到两块，也不能仅用非黑图判断。以下是独立设计实验支持的首版门槛，全部放配置并按 phase 显式解析：
- right_wrist：pregrasp/grasp_pose/lift_pose 看 source；place_hover/place_pose/retreat 看 target，目标至少 64 像素；
- global：target 全阶段至少 64；source 除 grasp_pose 外至少 32；grasp_pose source 只记录，抓取目标由 right_wrist 门槛保证；
- left_wrist：每阶段可见桌面至少 4096 像素；同时记录两块像素与遮挡。S4 必须另检实际搬运后有效任务区域和遮挡时长，不能以这里空手画面替代。

任何净空、IK、冻结、关键视角失败都导致最终非零；保留已完成的诊断和首个准确原因，不自动降阈值/调外参/重采种子。每 layout 独立初始化，不沿用上一随机布局的解。

检查配置严格拒绝未知/重复字段、bool 数字、NaN/Inf/巨大整数、无效阈值、超过上限布局/迭代/采样等，错误带文件与字段；在 GL/大量 IK 前拒绝。报告不存全部图像数组或大量每样本轨迹，只存有界摘要、关键关节解、最差样本和图像相对路径。报告应有 schema/kind=`kinematics`/run/instance、配置有效值及 hash、代码/依赖/资产引用、seed/layout、所有失败、残差/关节解/净空/视野/耗时、PNG 清单、logs_closed、最终 status/exit/reason。PNG 总量上限 128 MiB，日志复用已有 32 MiB 容量，报告有界；接近上限失败并保留原因。

复用 EventLogger，component=`kinematics_check`，目录实例名一致，输出 start、每 layout 结论、失败与 finalizing，不把每个 IK 迭代写入日志。建日志在先；资源和日志关闭失败均传播到真实退出码；最终报告写入失败也失败，close 前不能宣称全任务成功。宿主沿用固定命令/精确容器清理，不接受任意模块。

## 自测及监督者验收

纯 Python 验证配置/目标生成/路径采样上界与来源、入口固定命令与挂载；原生测试验证真实 mink 解/冻结、不可达目标不伪成功、真实穿透/净空不足路径被拒绝、可见性失败和 close/report 故障。不要写只检查源码字符串或只把 mock 返回值断言一遍的测试。

使用 PROJECT_PYTHON/PROJECT_RUFF，记录真实 unit、Ruff/format 结果。宿主缺 MuJoCo 时仅 skip 确需原生的测试，不自行安装。监督者在固定镜像运行真实 67 布局检查、查看关键 PNG、独立复算最差净空和 FK，核对失败反例、默认旧入口、日志及资源清理。未过门槛不得称 S2 完成。
