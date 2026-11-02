# DSH-022：无ROS物理运动、接触完成与停止

2026-11-01。DSH-019～021已验收；用户已确认接触约束的夹爪完成规则。首轮30分钟，最多一次10分钟定点返工，交付报告即退出。

## 范围

本项交付完整的无ROS运动模块，不同时实现公共Robot协议。允许新增：

- `components/backend_sim/src/robot_backend_sim/motion_config.py`：严格加载不可变控制配置，纯Python/PyYAML；
- 同目录 `contacts.py`：具名原生双指接触、力、单指速度的不可变快照；
- 同目录 `braking.py`：纯数学有限减速参考；
- 同目录 `motion.py`：组合PandaIO、参考计划和真实MuJoCo的单运动状态机；
- `configs/simulation/panda_control.yaml`；
- `tests/unit/test_motion_config.py`、`test_braking.py`、`test_gripper_completion.py`、`tests/integration/test_native_motion.py`；
- `docs/native-motion.md`及文档索引/README当前边界。

返工补充允许 `panda_io.py` 增加公开的Scene/Thread归属校验方法；不改变既有I/O语义。可从 `contracts/python/src/robot_contracts/reference.py` 提取公开的连续限位校验函数，供normal/brake共用；保持既有normal验证与采样语义，不重写样条/求根。其他既有生产文件、ROS/Policy类型、mock/runtime/profile/场景/镜像/锁/Compose/工具入口不改。不引入继承体系、插件、通用事件总线或调度框架。

不实现lease、网络命令幂等历史/query/reset/epoch、ROS队列/三相机/RPC/teacher/任务成功判定，不宣称S3通过。不安装依赖、运行Docker、使用非固定native缓存、申请权限、启动代理、提交Git。宿主缺native依赖可skip；监督者用固定镜像实跑。

## API与所有权

提供typed API，例如 `prepare_motion(initial_reference, targets, profile, config)` 返回深层不可变 `PreparedMotion`，`NativeMotion(scene, io, config, initial_reference=...)`，及 `start(prepared, wall_time_ns)`、`step(wall_time_ns)`、`poll(wall_time_ns)`、`request_stop(reason, wall_time_ns)`。等价清楚命名可接受，文档写明返回值和时间语义。

prepare不访问Scene，编译/连续验证在物理循环外一次完成；PreparedMotion直接构造也必须验证。start仅检查已验证计划与当前profile/限位/保持参考匹配，不在step拟合或验证整条曲线。所有运行方法限制在创建NativeMotion/PandaIO的同一Thread对象；不创建线程、不睡眠、不读写文件或网络。

沿用场景2ms步长，只经PandaIO写ctrl并正常步进，禁止写qpos/qvel/time、物体位姿/外力或更改模型参数。初始保持参考显式验证，默认profile home，不是测量值；后续从最后保持参考开始，不用实测静差重新定基。

active/settling/stopping拒绝重叠；正常完成/停止后允许后续运动；故障锁存后禁止新运动，恢复留reset任务。start不算started，第一次实际应用参考并步进才started。poll不步进。所有入口拒绝bool/负数/非整数/回退的wall_time_ns，无隐藏墙钟；真实调用方以后传monotonic_ns，测试可传显式假时钟。

## 参考、测量与完成

step在物理时刻t应用参考，再mj_step至t+dt。不可变快照分别保留参考应用时间/elapsed、测量时间、参考p/v/a、实测q/qdot、误差与接触证据。如读取步后接触力须mj_forward刷新，使证据与测量时刻一致，不能标旧力为新时刻。独立raw actuator/step/forward重放应得到一致qpos/qvel。elapsed不等于步后measurement_time-start。

阶段区分ACCEPTED、RUNNING、SETTLING、STOPPING、COMPLETED、STOPPED、FAILED，可用枚举。horizon到时仍等实测完成。terminal快照冻结，后续hold/step不篡改；实时物理快照与冻结result清楚分开，只保留有界状态。局部motion序号尚不冒充ROS command ACK。

14个臂关节须同时满足逐轴位置误差和绝对速度。每只夹爪可：

1. 总开度误差与总开度速度均达标；或
2. 最终参考不比本次初始参考更开，且实测开度大于该参考、误差尚未达位置门槛；两根具名finger body接触同一个允许的自由物体，每指法向合力达标、每指关节绝对速度达标。重复保持已为0的闭合参考仍可采用此规则；正在打开的参考不能采用，双指总速度相消不能掩盖单指运动。

只接受scene配置的具名freejoint物体source/target，排除world/桌/机器人link；每指对同物体的各接触点mj_contactForce法向力相加。无接触/单指/不同物体/力不足/单指过快均失败；双方共同接触多个候选时判歧义，不任选。完成模式或物体身份改变、任一臂/夹爪失稳，重置整个连续稳定窗口；horizon之后才计时。

保持原闭合目标，不改成实测开度。记录每夹爪POSITION/CONTACT模式、同物体、双指力、单指速度和原目标误差。执行完成与堆叠成功分开，无物体位姿送入Policy。

## 有界停止与故障

正常计划用DSH-021 clamped cubic。stop从最后实际应用参考p/v开始，各轴按配置最大减速度做常加速度刹车至0再hold；不瞬间把参考速度设0。停止时间向上取整ns，实际减速度不超阈值。合并各轴停止边界为显式分段计划，位置/速度连续并验证连续区间限位；acc可阶跃，不声称jerk受限或刹车C2。零速度轴保持，全部零速度使用清楚hold路径，不造零时长段。

Stop请求只进入STOPPING；继续真实步进、保持目标，实测逐轴速度连续稳定达标才STOPPED，不强制张开夹爪。重复stop不延期。处理未started即stop、settling中stop；原轨迹终点不覆盖刹车保持终点。无法生成合法刹车则锁存FAILED/STOP_PLAN_REJECTED并保持最后已应用参考，记录降级，不能声称有界停止通过。

settling有独立仿真时间上限，整个运动有墙钟期限；停钟时poll仍触发STOPPING，STOPPING另有墙钟期限，持续不步进应FAILED/STOP_TIMEOUT。tracking超时进入刹车，停稳后仍FAILED，保留TRACKING_TIMEOUT/WALL_TIMEOUT；禁止新运动绕过故障。没有隐藏容差扩大或失败转成功。

## 配置初值

固定16维joint_names和逐轴数组，SI单位明确；位置/速度硬限位来自profile，不重复手填。拒未知字段、bool、非有限/溢出、长度/顺序错、非正阈值/期限；错误含来源路径和字段；有效配置可序列化为快照。

| 项 | 候选值 |
| --- | --- |
| max_accelerations | 臂1.0 rad/s²；总开度0.5 m/s² |
| position_tolerances | 臂0.012 rad；总开度0.002 m |
| velocity_tolerances | 臂0.025 rad/s；总开度0.004 m/s，stop也用此逐轴门槛 |
| stable_duration | 正常完成和停止均0.2s仿真时间 |
| settle_timeout | 参考horizon后2s仿真时间 |
| motion_wall_timeout / stop_wall_timeout | 10s / 3s单调墙钟 |
| 每指接触力下限 / 单指速度上限 | 0.5N / 0.003m/s |

上游模型未补偿重力时已测臂静差约0.00658rad，0.012候选据此选择，不要求假零误差。不得为新用例调宽阈值或修改模型。

## 验收

保留538 unit与Ruff。纯测试覆盖严格配置/快照不变、闭式刹车、多轴不同停止时间、连续限位/零速度、非法输入/溢出。native覆盖真实小幅右臂/夹爪及左臂保持、到时尚未完成、稳定/超时、两次非home连续运动、拒重叠、所有运行入口线程限制、terminal冻结、运动中刹车连续与实测停稳、settling取消、重复stop不延期、停钟墙钟故障。

接触夹具允许仅在运行前设一次自由块位于已经收窄的双指间，之后仅ctrl与正常物理；不算teacher/正式episode。预检方法：home的right_tcp放source并按site旋转对齐，两指初值各0.02m，ctrl闭合0；1s后实测约0.03985m，每指合力约1N，warnings0。dsh可按描述建自己的夹具。证明目标仍0、保留约4cm误差且CONTACT模式完成，再检无接触/单指/不同物体/速度/力不足等负例。合成证据只证明逻辑，正例必须真实MuJoCo。

本模块不另建日志writer或启动器，返回明确阶段/原因/快照供后续唯一ROS层记录。监督者本项用既有EventLogger记录阶段与结果，高频轨迹另存诊断文件；完整日志故障停止/ACK/MCAP随ROS接线再次验收，当前不声称完整链路已接入。

报告 `.build/reports/dsh-022/report.md`：文件/API、阶段与时钟定义、实跑/skip、限制、待固定镜像项。host只用任务头PROJECT_PYTHON/PROJECT_RUFF，不找缓存native环境，不用skip冒充通过。
