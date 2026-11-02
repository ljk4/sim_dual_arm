# 定时参考轨迹与物理执行边界

2026-11-01。DSH-020已验收I/O；DSH-021已实现可验证的计划值、SciPy编译器和连续约束检查，已由DSH-022接入无ROS运动执行层；公共执行ACK、lease/reset和ROS接线仍未完成。

采用仿真镜像已有的SciPy 1.17.1 `CubicSpline` 编译定时位置路点。旧调用仍使用两端速度为零的clamped边界；DSH-023增加显式起止一阶导数，供连续动作块携带解析p/v边界。段内位置、速度和加速度连续；官方接口与系数定义见 [CubicSpline](https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.CubicSpline.html)、[PPoly](https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.PPoly.html)。在线文档当前显示其他版本，本地1.17.1接口另做实际检查，不据在线版本更换依赖。

不直接接入Ruckig社区版的中间路点方案：其官方说明该路径会调用云API，且minimum_duration不保证精确时长，与当前固定本地时间戳的动作路点不直接对应。[Ruckig说明](https://docs.ruckig.com/tutorial.html)。后续若要接在线重规划/jerk约束，单独评估接口，不隐含改变当前命令时间。

```mermaid
flowchart LR
  Targets[带相对时间的SI位置路点] --> SciPy[仿真侧SciPy编译]
  SciPy --> Plan[不可变分段三次多项式]
  Plan --> Verify[无SciPy的共享数学校验与采样]
  Verify --> Control[后续物理控制核心]
  Plan --> Audit[后续ROS传输及离线审计]
```

第一步只实现计划值、编译和校验，不同时新增ROS类型、状态机或平行启动器。编译器留在仿真侧；共享契约只保存显式系数，并用标准库计算多项式、导数和极值，runtime无需导入MuJoCo/SciPy。

各段以局部秒为变量，`p(s)=c0+c1*s+c2*s²+c3*s³`；时间边界仍用整数ns。保持原始路点时间，逐个通过路点，不悄悄retime、clip或用测量值替换参考。除端点外还检查位置、速度、加速度的内部极值；三次样条可能过冲，路点合法不代表整段合法。

初值来自控制核心最后保持的参考；实际测量另存。首版计划只从已保持状态开始/结束，不提前承诺动作块执行中的无缝抢占。超过horizon后保持末参考，不外推；数学参考结束不等于机器人完成，后续核心仍须按真实跟踪与停止反馈判定。

## DSH-021 当前实现

`robot_contracts.reference` 只依赖标准库，冻结并校验显式系数：

- `CubicSegment(duration_ns, c0, c1, c2, c3)`，系数按关节顺序升幂存放；
- `ReferencePlan(joint_names, segments)`，`duration_ns`由段时长相加得到，不重复保存；
- `plan.sample(elapsed_ns)`返回不可变的`ReferenceSample`（位置/速度/加速度）。内部边界取结束该边界的段；严格晚于horizon时保持末位置且速度、加速度为0，不访问时钟、不外推；
- `compute_reference_extremes(plan)`公开连续区间极值；`verify_reference_plan(plan, initial, targets, joints, max_accelerations)`检查时间/边界/连续性并返回极值。位置极值检查速度二次式实根，速度极值检查加速度一次式实根，加速度极值检查段端点；不靠稀疏采样；
- 结构错误使用`INVALID_REFERENCE_PLAN`，连续限位分别使用`JOINT_LIMIT`、`VELOCITY_LIMIT`、`ACCELERATION_LIMIT`。位置/速度/加速度连续性绝对容差为`1e-9`，相对容差`1e-9`；限位比较使用同样的相对容差和`1e-9`绝对吸收量，只吸收边界处浮点误差；
- 构造时深层tuple化，原始list/数组后改不影响计划；系数、采样和极值若无法保持有限会明确失败。

`robot_backend_sim.reference_compiler.compile_reference_plan(...)`先完整校验输入规模、时间和端点限位，再用SciPy 1.17.1 `CubicSpline`（`axis=0`、`extrapolate=False`）编译一次，将降幂`c[0..3]`反转成升幂`c3..c0`，最后必须通过共享验证才返回。两处均未显式给出端点速度时保留`bc_type='clamped'`以维持旧系数；给定任一端时用`bc_type=((1, initial), (1, final))`，未给出的另一端为同维零。编译器不改路点时间、不retime、不clip；SciPy缺失或native异常映射为`ContractError`。采样与验证不导入SciPy。

该实现只是数学参考计划。`sample`到末尾或计划通过验证都不表示物理执行完成；DSH-022已补无ROS控制状态机、真实跟踪与停止，见[原生运动](native-motion.md)；公共ACK、lease/reset和ROS传输仍待接线。

本阶段不声称jerk受限、运动学/接触可行或物理运动完成。夹爪接触完成口径已由用户确认，归后续控制核心：臂关节按测量达标，闭合夹爪允许双指同物体稳定接触，原目标/误差/证据分别记录。

## DSH-023 连续边界与严格采样

`compile_reference_plan`和`verify_reference_plan`新增仅关键字参数`initial_velocities=None, final_velocities=None`。`None`分别表示该端同维零速度，旧调用行为不变；显式数组沿用1..64轴、有限实数、深tuple、拒bool/字符串/溢出/错维的规则，并逐轴按`JointSpec.max_velocity`检查，超限使用`VELOCITY_LIMIT`且不clip。共享校验函数`validate_velocity_boundaries`同时供编译器和验证器使用。

`verify_reference_plan`核对调用者声明的起止p/v边界，而不是读取计划自身导数作为“预期值”；仍检查全部原始路点时间/位置、内部p/v/a连续性和连续区间p/v/a限位。因此非零端点速度不会被共享验证悄悄改回零要求。

`ReferencePlan.sample(elapsed_ns)`保持旧语义：严格晚于horizon时保持末位置且v/a为零。新增`ReferencePlan.sample_interval(elapsed_ns)`只接受`0..duration_ns`，区间外（含负值）以`REFERENCE_EXPIRED`失败，并在horizon处返回真实端点速度，不做零速hold、不外推。连续动作块只使用严格采样。

连续动作块的预编译（`PreparedChunk`）在共享计划验证之外，还用计划实际末端p/v调用既有`.braking.plan_braking`，以原joints/max_accelerations预演无后继时的合法刹车；该刹车计划不保存、不改原曲线。曲线p/v/a合法不代表可以安全等待后继：非零末端速度若刹车会越过位置限位，准备阶段即拒绝。缺块退路因此是执行前门槛，不是运行时才补齐的数学性质。
