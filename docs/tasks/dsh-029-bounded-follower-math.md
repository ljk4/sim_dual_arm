# DSH-029：受限目标跟随的纯数学层与对比

## 背景和边界

用户已选择：允许通用限速/限加速度目标跟随；原始 ACT 预测、实际执行参考及偏差分别保留；
采集/推理共用，标签用实际参考；不使用物体真值补救，双臂16维均保留。先对比验收再接入物理。
严格逐点拟合仍保留。本项只做数学/配置/标准库DTO及测试，不改ROS IDL、controller、worker、runtime或训练。
不执行Docker、不联网安装、不提交。首轮30分钟，最多一次集中返工。

现有 main 已验收027；028仍由监督者审查，与本项无文件重叠。不得依赖028源码。
允许新增 contracts/python/src/robot_contracts/follower.py（标准库），
components/backend_sim/src/robot_backend_sim/follower_plan.py、follower_config.py，
configs/simulation/panda_follower.yaml、对应tests、docs/bounded-target-follower.md。
可拆一个明确职责的 ruckig_export.py；不得新建通用框架，不修改已有严格参考验证契约。

## 已查实可用依赖

Ruckig Community 0.15.3，监督者隔离构建成功：
/home/ljk/work/sim_dual_arm/.build/follower-probe-venv/lib/python3.12/site-packages
使用任务头PROJECT_PYTHON，加该路径到PYTHONPATH即可；不要修改任何venv。
Community公开 trajectory.profiles[section][axis]，Profile.t/p/v/a/j及brake/accel可读取。
本轮只使用二阶限加速度，max_jerk默认无穷，不使用Pro/cloud/waypoints，不新增jerk约束。
官方源码：https://github.com/pantor/ruckig/tree/v0.15.3 。不得从网络加载运行代码。

## 输入与输出

复用 ActionWindow/ChunkAnchor/ReferencePlan/CubicSegment/JointSpec/PandaControlConfig。
新 PreparedFollowerPlan 是不同的显式类型，不冒充 PreparedChunk；不得绕开其逐点通过原目标的验证。
保存原window及anchor，不改原预测位置。输出显式系数参考、实际末端p/v、原始与受限终端速度、
各轴最大目标偏差、发生限速/限加速度/边界保留的标志及真实duration。
不可变、维度明确、具体类型、SI/纳秒命名。共享DTO不导入Ruckig/Numpy/SciPy/ROS。

初始工程参数：每次选择5个点/.25s，source仍为60×16/20Hz；终端尾段最多3s，偏差上限
机械臂0.15rad/夹爪总开度0.02m（配置数组，非放宽关节硬限位）。全部配置严格解析、拒绝未知键/NaN/bool。

规划方法：单个执行window仅计算一次Ruckig，目标取选中最后位置，端速取末两点后差分。
端速允许被通用限速及位置边界刹车可行速度限制，必须保留原始端速和实际端速，不能修改raw目标。
非final：min_duration=window.end_time，规划至少覆盖该时段，裁剪前缀到固定handoff；
实际末端可能未到原目标，继承该实际参考p/v给下块，绝不从测量值重基准。
final：终端速度0；必要时延长实际参考直至到达原最后目标（总duration不得超过3s），明确记录延长量。
若无法满足界限/偏差预算/刹车可行性，显式ContractError，不静默降低约束或把未到达当完成。

## 精确系数导出与验证

从公开Profile各轴phase分段导出p/v/a/j为c0=p,c1=v,c2=a/2,c3=j/6。
合并各轴分段边界，截断非final时域；ReferencePlan使用整数纳秒。
二阶加速度跳变允许，p/v必须满足原有C1容差；不要放宽全局容差，也不要以20Hz采样代替系数证明。
处理0时长phase、接近同刻的边界、不同轴同步、静止轴、方向反转及brake段；不支持的结构明确拒绝。
使用共享ReferencePlan极值/输入工具验证整条曲线的p/v/a硬限位，验证末端可刹停。
不要调用旧 verify_reference_plan 假装原预测逐点匹配；可以用实际各系数断点的Target辅助原数学验证。
记录/验证 desired(t)：anchor在t=0，selected raw targets 按其相对handoff时间作分段线性目标，
final延长尾段目标保持最后值。偏差最大值必须用各目标分段与参考分段交集上的三次多项式极值求解，
不是仅在控制tick抽样。上限超出显式拒绝。
外部可独立重建/校验所保存系数及摘要，保留全部raw source targets。

## 验收

单元：纯标准库DTO、配置、静止/单轴/16轴/反向/极小时间/边界夹爪/非法输入/超偏差。
Ruckig原轨迹与导出曲线在边界左右和密集确定性时刻的p/v/a比较，量化纳秒舍入误差。
确定性随机至少100组16轴可行例，解析限位及C1检查，无需MuJoCo。
复现严格模式首窗反例：.04rad/3s smoothstep，lead250ms，严格拟合maxa1.8284756>1；
受限跟随在相同原始预测与延迟下生成合法参考并如实报告偏差（如果偏差过大，明确拒绝）。
另验2rad/7s片段lead50/100/250ms。严格baseline使用现有prepare_chunk，不改旧实现。
输出可读JSON对比摘要（不是伪物理成功），包含原始/实际参考/极值/偏差和拒绝原因。
Ruff、原单元回归及git diff --check。报告 .build/reports/dsh-029/report.md列职责、API、实际检查。
不要声称已接入控制器、已经训练、已经完成双臂任务。完成后退出。
