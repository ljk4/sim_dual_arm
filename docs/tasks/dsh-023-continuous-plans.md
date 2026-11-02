# DSH-023：连续参考边界与观测锚定动作窗口

2026-11-01。用户已选连续接续，[设计](../continuous-action-design.md)。只做预编译计划/时间映射，不接物理状态机。首轮25分钟，最多一次10分钟定点返工，交付即退出。

## 文件范围

可修改：`contracts/python/src/robot_contracts/reference.py`、`components/backend_sim/src/robot_backend_sim/reference_compiler.py`、`tests/unit/test_reference_plan.py`、`tests/integration/test_reference_compiler_native.py`。

可新增：`contracts/python/src/robot_contracts/action_chunks.py`、`components/backend_sim/src/robot_backend_sim/chunk_plan.py`、`tests/unit/test_action_windows.py`、`tests/integration/test_chunk_plan_native.py`。

可更新：`docs/continuous-action-design.md`、`docs/reference-trajectory-design.md`、`contracts/README.md`、`docs/README.md`和README当前边界。其他生产代码（尤其motion.py）、ROS/Policy类型、profile、配置/镜像/依赖/启动器不改。禁止安装依赖、Docker、非固定native缓存、其他工作树修改、提交Git、启动代理或申请权限。host缺native可skip，不能算原生通过。

不实现状态机、队列、抢占、日志/线程/网络、ROS或Policy，不新增通用注册器/继承/solver。保持587 unit、Ruff check及format通过，旧零速端点、MCAP和mock行为不变。

## 1. 显式起止参考速度

为 `compile_reference_plan` / `verify_reference_plan` 增加仅关键字 `initial_velocities=None, final_velocities=None`。None各自表示同维0，旧调用保持原行为；显式数组按已有1..64轴、有限实数、深tuple、拒bool/字符串/溢出/错维规则校验。逐轴绝对速度不得超过JointSpec.max_velocity，错误VELOCITY_LIMIT，不能clip。共享辅助校验可以复用，避免两处各写一套规则。

用既有SciPy1.17.1 CubicSpline，axis=0，bc_type指定两端一阶导数，extrapolate=False。没有显式边界时可保留clamped以保证旧系数。共享验证改为核对所声明p/v边界，仍检查所有原始路点的原时间/位置、内部C2、连续区间p/v/a限位。不能仅让编译器接受非零速度而共享验证仍偷偷要求零，也不能靠读计划本身的v作为“预期边界”绕过调用者声明。

ReferencePlan原sample保持旧horizon后hold语义；另加清楚的严格区间采样方法（如sample_interval），只允许0..duration_ns，越界ContractError/REFERENCE_EXPIRED。新的连续动作块必须走严格采样，不能将非零末速度瞬间hold0，不外推。

## 2. 共享时间窗口（仅标准库）

提供冻结 `ActionWindow` 和 `ChunkAnchor`，可用等价清楚名字，文档列API。

ActionWindow输入：`observation_id`、`observation_time_ns`、`handoff_time_ns`、原始 `source_targets: tuple[Target,...]`、`target_count`。原始预测时间相对observation_time。handoff≥observation_time。选择原始绝对时间**严格大于**handoff的前target_count个点，并改写为相对handoff的正时间差，位置值不变；恰在handoff的点不重复。保留完整源预测/索引/跳过重叠计数供追溯，不静默丢未知尾部或缺点。选点不足应INSUFFICIENT_HORIZON，全部过期应CHUNK_EXPIRED。这里是预定窗口选择，不是迟到预测补救，实际准入期限归后续执行层。

规模：源1..128点，1..64维、长度一致，正时间严格递增，预测horizon≤3_000_000_000ns；target_count为2..128。完整源（含不执行前缀和尾部）都校验有限实数/维度/时间，不能靠切片藏坏数据。source Targets深tuple化，输入list之后改动不污染。派生的选点/索引/结束时刻用属性计算或少量必要缓存，不保存多份可不一致状态。

时间必须整数ns，拒bool/负数/非整数，observation_id为1..uint64max，机器人时间及绝对相加结果为0..int64max；先用整数计算绝对时刻/相对差，再转局部秒，不能大时间先转float。错误带字段。直接构造类型也校验，不只factory校验。

ChunkAnchor输入：非空且≤128字符的`stream_id`、`predecessor_seq`（0..uint64max-1）、`start_sim_time_ns`、不可变`reference: ReferenceSample`、非空`robot_profile_hash`。参考p/v/a只是控制元数据，不是新增ACT特征；跨块匹配p/v，不声称a连续。该锚点后续由物理所有者在预定交接点生成；本项不生成身份/不读Scene。每块seq由predecessor+1得到。

## 3. 仿真侧预编译块

冻结 `PreparedChunk` 与 `prepare_chunk(anchor, window, profile, config, *, final_velocities=None, settle_at_end=False)`：

- anchor/window起点相同，profile摘要与关节顺序匹配。保存锚点、源窗口、显式plan、joints/accel、已选定末速度、末端模式；可派生的名称/维数/时长/seq不重复存。直接构造同样校验计划及完整源，深层不可变。
- 使用anchor.reference的p/v作为初边界。继续块显式final_velocities优先；None时按选中最后两点的 `(q_last-q_prev)/(t_last-t_prev)` 得到末参考速度，严格限位。settle_at_end必须bool，True使用0末速度；显式非零末速度与True冲突应拒绝。末速度差分只用于插值边界，不篡改任何位置目标或时间。
- 全部原始source点按profile位置硬限位检查，即使不执行的前缀越界也拒绝。验证成功后编译一次，不能在sample再次拟合。最大加速度来自已有控制配置，不新增或改宽阈值。
- `sample_at(sim_time_ns)` 先检查合法整数与绝对有效区间start..end，再用整数差转本地elapsed，走严格plan采样；区间外明确失败。不得读时钟/Scene、IO/网络/日志。静态准备允许导入已有SciPy，runtime共享时间类型不依赖科学库。
- NativeMotion/PreparedMotion仍保持原停稳行为，本项不让旧执行器接受此新块或宣称已能连续运动。

## 自测与独立门槛

纯测试：非零参考速度边界的手算多项式/错误边界/速度限位；严格采样前后界；时间窗口的重叠/恰在起点/索引/缺尾/全过期/因果反向/完整源坏前缀；bool/NaN/Inf/大整数溢出/维数、直接构造与深不可变；靠近int64上界的整数差精确，预期源时刻可手算。

固定native测试：SciPy边界导数/系数/roots独立对照；把原合法3s/右q1+0.15rad的完整轨迹以50ms采样切3个1s块，携带解析p/v边界后应恢复原曲线且原限位通过，旧每块clamped仍拒绝；Panda16维/静止左臂/夹爪m通道；默认末速度估计、settle末速度冲突、坏前缀限位拒绝、只准备不写物理状态。宿主skip不可替代固定镜像。

报告 `.build/reports/dsh-023/report.md`：文件、API/时间和速度约定、实际host结果、待native项、限制。PROJECT_PYTHON/PROJECT_RUFF用任务头绝对路径；只运行一次必要自测后交付，不搜索主目录缓存native或反复试装版本。
