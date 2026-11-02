# DSH-021：可验证的分段三次参考计划

2026-11-01。DSH-020已验收；用户已确认后续夹爪接触完成口径。先实现物理控制可复用的轨迹计划与约束检查；实际物理执行/ACK/lease/reset/ROS仍在下一项。[决定与来源](../reference-trajectory-design.md)。首轮20分钟，最多一次10分钟定点返工，报告后立即退出。

## 范围

允许新增：
- `contracts/python/src/robot_contracts/reference.py`：无ROS/NumPy/SciPy/MuJoCo依赖的不可变计划、采样、导数、极值、验证；
- `components/backend_sim/src/robot_backend_sim/reference_compiler.py`：调用固定环境已有SciPy编译；
- `tests/unit/test_reference_plan.py`：纯数学与非法输入/约束行为；
- `tests/integration/test_reference_compiler_native.py`：固定SciPy实际编译与系数/导数/极值对照；
- 可更新本文对应设计文档 `docs/reference-trajectory-design.md`、`contracts/README.md`、`docs/README.md`、README当前边界。

不改任何既有ROS/Policy类型、CommandStatus/既有线性校验或mock行为、profile、物理场景、依赖锁/镜像/Compose/工具入口。不要让既有status声称支持尚未接线的cubic流。不实现控制状态机、线程/队列、RPC、日志库或新框架。不安装依赖、不运行Docker、不使用缓存的非固定native版本、不申请权限、不启动代理、不修改其他工作树、不提交Git。

## 契约和数学

1. 直接、少量的冻结类型即可：`CubicSegment(duration_ns, c0, c1, c2, c3)`，每个c是按关节顺序的系数tuple；`ReferencePlan(joint_names, segments)`；采样结果含positions/velocities/accelerations；必要时提供清楚的极值结果类型。可用等价清楚命名，不引入继承/注册器/多版本框架。
2. 局部变量为秒，`p=c0+c1*s+c2*s²+c3*s³`。段duration为正整数ns（拒bool）；最多128段、总horizon最多3_000_000_000ns，与现有轨迹边界一致。关节1～64维，唯一非空名字；数组长度一致、深层tuple化、有限实数，拒bool/字符串/非有限/超大整数转换溢出。构造/验证错误为带字段的ContractError，不泄漏IndexError/OverflowError。
3. `plan.sample(elapsed_ns)`以本地执行时长采样，不访问墙钟/物理时钟。拒负数/bool/非整数。0和内部边界有确定语义；horizon内按段求位置/一二阶导数，超过horizon保持末位置且速度/加速度置0，不外推。采样值不可变、所有计算结果有限；不能把“sample到末尾”说成已执行完成。
4. 显式验证函数接收plan、initial_positions、原始`Target`序列、`JointSpec`序列及逐轴`max_accelerations`（由后续控制配置调用者传入，不在本模块藏控制阈值）。检查关节/维度、原始时间顺序和horizon、初值/路点与计划边界对应、两端速度为零、内部分段位置/速度/加速度连续。所有比较使用写明的合理数值容差；不能靠大容差接受实际不连续。
5. 校验整个连续区间的关节范围、max_velocity和max_acceleration，超限明确JOINT_LIMIT / VELOCITY_LIMIT / ACCELERATION_LIMIT；不得只采样稀疏网格。位置极值检查速度二次式在段内的实根，速度极值检查加速度一次式实根，加速度检查端点。处理退化常数/线性/近零系数和数值溢出；使用稳定二次求根/必要缩放，无法可靠计算要清楚失败，不能将NaN当“无极值”。返回或公开可读极值结果供后续日志与独立核对。
6. 共享契约只计算和校验显式系数，不自行实现三对角样条求解器，不调用文件/网络，不打印数组。段/计划不可被原始list/NumPy数组后续修改。避免重复保存可从系数/段时间直接得到的同一状态。

## 仿真侧编译器

`compile_reference_plan(initial_positions, targets, joints, max_accelerations)`（或等价typed API）使用已有SciPy 1.17.1 `CubicSpline`，时间轴为 `[0, *targets.time_ns]/1e9`，位置为初值和后续原路点。`axis=0`、`bc_type='clamped'`、不外推。正确把SciPy降幂系数`c[k,i]`转换为上述升幂c0..c3；不得转置左右臂或通道。

输入先完整校验/控制规模，再调用native；异常映射成清楚ContractError，成功计划必须通过共享验证后才返回。保留路点时间/位置，非法时间/限位/过冲不能偷偷放宽、改时长或clip。计算只在编译时做一次；plan采样不再次拟合或依赖SciPy。该包不需要新的依赖声明或安装，宿主缺SciPy时native integration可以skip，由监督者在固定镜像执行。

## 自测、交付与独立门槛

纯测试要有可人工验算的单段闭式例子、常数、多轴/不规则时间/内部边界/超时hold/不可变性、非法尺寸/时间/bool/NaN/Inf/溢出、断开位置/导数、端点合法但内部位置过冲、速度峰值和加速度峰值超限。不要用两个相同实现互相比证明正确。现有499 unit与Ruff lint/format必须保持。

新增native测试使用真实SciPy，与其调用/导数/roots结果核对，覆盖Panda16维混合单位及常数待机通道。监督者另做随机多轴/不规则间隔及反例的独立检查，并验证在不安装SciPy的ROS环境仍可导入/采样/验证显式计划。

报告 `.build/reports/dsh-021/report.md`：修改文件、数学约定、实际host结果、待native项、限制。host工具只用PROJECT_PYTHON/PROJECT_RUFF，一次必要自测后交付退出。没有控制器或ROS链路证据时不得宣称S3通过。
