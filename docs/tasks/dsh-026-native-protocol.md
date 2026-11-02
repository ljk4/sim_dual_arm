# DSH-026：原生参考与反馈的公共类型

2026-11-01。DSH-025已验收：685 unit、92 native、115旧ROS/MCAP与13独立组。
用户要求持续推进闭环。本项只实现可独立审计的传输类型/转换，不启动ROS服务或策略。
首轮35分钟，最多一次10分钟集中返工；报告交付即退出。

## 范围与原则

允许新增：

- `contracts/python/src/robot_contracts/native.py`：纯标准库原生请求、已接受参考、反馈/接触/块状态DTO及校验。超过约500行可按明确职责拆为同包`native_reference.py`、`native_feedback.py`，不得制造通用schema框架。
- `libraries/ros_support/src/robot_ros_support/native_conversions.py`：上述类型与新ROS类型的显式双向转换；不导入backend/MuJoCo/SciPy/NumPy。
- `components/backend_sim/src/robot_backend_sim/protocol.py`：请求预编译及NativeController结果到共享DTO的转换；不导入ROS、不写盘、不步进、不创建线程。
- 下表列出的新`contracts/ros/robot_api_msgs/msg/*.msg`和`srv/*.srv`，允许更新同包CMakeLists注册。
- `tests/unit/test_native_contracts.py`、`test_native_reference_record.py`；`tests/integration/test_native_protocol_ros.py`、`test_native_protocol_physics.py`；`docs/native-protocol.md`及`docs/README.md`。

另允许一个很小的既有诊断修复：`backend_sim/.../motion.py`及`tests/integration/test_native_motion.py`。
当前`_fail`只因存在stop_plan就标reference_kind=BRAKE，暂停物理、未执行任何brake却STOP_TIMEOUT时会误报实际参考类型。
应保存/复用最近**实际应用**的kind（优先放在既有不可变_AppliedStep内），失败反馈沿用真实kind，不把“已规划刹车”冒充“已应用刹车”。
不改变p/v、ctrl、时间、阶段、限位/完成规则，不复制最后应用的多份状态。补无brake应用和已实际brake两种原生断言。

不修改其余旧IDL字节、旧linear_v1状态校验、mock/runtime/MCAP、Policy proto、控制会话/场景/profile/config/依赖/镜像/启动器。
不安装、不Docker、不权限、不代理、不提交、不改主目录。host可skip ROS/native，报告必须准确；监督者构建新SDK并固定镜像验收。

控制包使用具体类型和SI数组，不用pickle、动态反射、通用插件或JSON字符串充当ROS控制载荷。
共享DTO只依赖标准库和既有contracts；参考数学继续用既有ReferencePlan/CubicSegment/ActionWindow，不另造插值/极值算法。

## 请求与模式

共享 `NativeRequest` 包含既有 `Trajectory`（context/lease/command/seq/observation/deadline、joint_names和完整source targets），
以及kind、可选ChunkAnchor、observation_time_ns、target_count、settle_at_end、可选final_velocities。
冻结且深规范化。共享ControlCommand负责身份校验；源路点1..128、轴1..64、正递增时间且horizon≤3s、维数一致、有限实数且拒bool。
header/frame/额外velocity/acceleration/effort等不支持语义在ROS转换中明确拒绝，不静默丢字段。

三种模式明确区分：

1. MOTION=1：源时间相对本次实际STARTED，使用既有prepare_motion零起止速度。anchor=None、observation_time_ns=0、target_count=0、settle_at_end=True、final_velocities=None；拒非默认的无意义字段。实际初始参考由调用方提供最后hold参考，不能使用测量重定基。
2. STREAM_FIRST=2：anchor.predecessor_seq=0、窗口时间相对**源观测捕获时刻**、其余语义沿用ActionWindow/prepare_chunk。
3. STREAM_NEXT=3：同一整体command_id/seq，anchor.predecessor_seq>0，observation_id可变但与窗口匹配。

stream只在固定handoff选择严格未来点，完整源（含不执行前缀/尾部）不隐藏坏数值或坏限位；不retime。
共享DTO做结构校验；profile关节/速度/加速度/连续极值由后续预编译及参考记录验证负责。
整型边界与ControlCommand一致（身份正数、时间非负int64；bool拒绝）。None在wire上用显式has_*或约定的空可选字段表达，真实0时刻不能当成未started。

## 新ROS类型（名称固定）

| 类型 | 字段语义 |
| --- | --- |
| `ReferenceAnchor.msg` | stream_id、predecessor_seq、start_sim_time_ns(int64)、robot_profile_hash、positions/velocities/accelerations，数组≤64 |
| `NativeCommand.msg` | 常量MOTION=1/STREAM_FIRST=2/STREAM_NEXT=3及kind；旧TrajectoryCommand command；ReferenceAnchor anchor；observation_time_ns；target_count；settle_at_end；has_final_velocities及final_velocities≤64 |
| `ReferenceSegment.msg` | duration_ns和c0/c1/c2/c3各≤64；p(s)=c0+c1*s+c2*s²+c3*s³，s为局部秒，不能倒置SciPy系数顺序 |
| `AcceptedReference.msg` | 原NativeCommand request、motion_seq、accepted_sim_time_ns、joint_names≤64、ReferenceSegment segments≤128、initial_positions/initial_velocities/final_velocities/max_accelerations各≤64、selected_indices≤128、reference_digest、interpolation_version固定explicit_cubic_v1 |
| `NativeFinger.msg` | side、finger_index(1/2)、body_name、joint_speed_m_s；object_names/normal_forces/contact_counts三个等长有界数组≤8，保留每物体的合力与计数，不只传单个contact布尔值 |
| `NativeGripper.msg` | side、mode(NONE/POSITION/CONTACT)、position_ok/contact_ok/contact_object/ambiguous、原target/initial/final_reference、实测总开度/速度/误差、两个NativeFinger |
| `NativeExecution.msg` | 完整context/整体command_id/seq/初始observation_id；整体phase/reason/stop_reason；motion_seq、stream_id、当前实际chunk_seq(0表示未应用块)；accepted/start/finish sim与wall字段及has_started/has_finished；真实motion_phase；reference_application_sim_time_ns、measurement_sim_time_ns、reference_elapsed_ns、stable_for_ns；joint_names、参考p/v/a、测量p/v、位置/速度误差；reference_kind、tracking_quality/violation_ns；两只NativeGripper |
| `NativeChunkStatus.msg` | context/command_id/seq/chunk_seq/observation_id、phase/reason、accepted/start/finish sim与wall和has_started/has_finished；不把RETIRED称为运动COMPLETED |
| `NativeControlEvent.msg` | ControlEvent原context、event_seq、command_id/seq/observation_id、is_chunk/chunk_seq、phase/reason、sim_time_ns；不生成新的身份/时间 |
| `SubmitNative.srv` | request NativeCommand；response accepted/reason、has_execution/NativeExecution、has_chunk/NativeChunkStatus、has_reference/AcceptedReference |
| `GetNativeAnchor.srv` | request context/lease_id、has_start/start_sim_time_ns；response success/reason/ReferenceAnchor |
| `QueryNativeCommand.srv` | request context/command_id；response success/reason/NativeExecution |
| `QueryNativeChunk.srv` | request context/command_id/chunk_seq；response success/reason/NativeChunkStatus |
| `ResetSimulation.srv` | request context/lease_id/seed(uint64，执行限制按既有validate_seed)；response success/reason/new_context |

所有新序列尽量使用ROS bounded sequence；编码/解码严格验证长度/范围/有限性/模式/has字段，而不是相信生成类。旧19个契约源中现存msg/srv/action字节保持，只允许CMake新增注册。
NativeExecution的整体terminal必须取ControlStatus冻结snapshot；后续RobotState实时测量不写回已冻结terminal，也不带新context冒充旧命令反馈。stream当前来源观测通过chunk_seq与AcceptedReference关联，整体observation_id保持最初引用。
Acquire/Renew/Release/Stop复用现有服务，不重复定义。ResetSimulation是可选仿真能力；不要求真机实现。

## 可审计参考记录

共享 `AcceptedReference` 保存NativeRequest、显式ReferencePlan、实际边界速度/初值/最大加速度、选中索引、accepted时刻和motion_seq。
完整原请求不被改成selected targets。reference_digest来自本公开DTO的稳定canonical JSON（格式有版本，非有限拒绝），包含请求/全部系数/时间/身份；不复用不可独立解析的backend私有对象哈希。

提供标准库 `validate_accepted_reference(record, profile, max_accelerations)`，复用现有verify_reference_plan/validate_reference_inputs：

- 关节/单位/配置顺序与profile一致，完整源所有点限位通过，实际系数通过连续p/v/a极值限制。
- MOTION初末速度0，selected_indices为空，初值等于记录的实际参考初值。
- STREAM由ActionWindow重新计算selected_indices和局部targets，anchor p/v匹配计划；explicit末速度按请求，None按最后两点后向差分，final settle强制0。
- reference_digest校验，拒时间/目标/系数/身份/selection/末端模式被篡改。跨块只声明p/v连续，不声称a连续。
- 不要求accepted_at已等于实际started。MOTION的实际计划零点来自后续NativeExecution.started_sim；STREAM计划零点是anchor的固定绝对时刻。

提供按绝对应用时刻重建记录参考的清楚函数，MOTION显式传actual_started_sim_ns；STREAM从anchor取start。
非final stream过horizon必须REFERENCE_EXPIRED；final零速末端可显式HOLD，不能为未续接块自动零速。BRAKE不伪装成原normal计划，完整取消审计留后续接线，反馈原p/v/a与实际类型仍保留。

## 反馈DTO与backend适配

NativeExecution共享DTO包含上表真实字段，冻结/深不可变；校验context身份、合法phase/模式、数组顺序/长度、实数、时间/flags、误差=参考−测量。
夹爪保留原目标和实测误差；CONTACT不是物体堆叠成功。NativeFinger的force非负、count整数、object_names唯一，与side/index一致。
不要把boolean转float、空started转0后丢失has_started、或让查询改测量。只检验传输结构/不变量，不在contracts里重新实现物理接触算法。

backend `protocol.py` 至少提供：

- `prepare_request(request, profile, control_config, *, initial_reference=None) -> PreparedControlCommand`，只有MOTION需要明确提供hold初值，stream从anchor取；在规划调用方运行，不读Scene。
- `accepted_reference(request, packet, accepted_snapshot)`，绑定实际预编译packet/接受事实，拒request与packet不对应；输出共享可审计记录。
- `execution_feedback(ControlStatus)`、`chunk_feedback(ChunkStatus)`、`control_event(ControlEvent)`，不改源对象、不读隐藏clock/物理真值。

ROS native_conversions提供每个公开DTO的显式双向函数，异常均为稳定ContractError，不吞错误。
没有ROS服务/队列/线程实现，没有MCAP writer，也不提前宣称Native capability已上线。

## 验收

685 unit、92 native、115旧ROS/MCAP保持。纯DTO单测包括直接构造/深不可变、mode/None/flags、int64/bool/NaN/超大数组/坏边界、完整坏前缀、canonical hash、计划重建/篡改与严格采样。
ROS测试真实serialize_message/deserialize_message往返及错误载荷，旧消息原始字节核对；不能用fake ROS类冒充。
native用实际PreparedMotion与三块PreparedChunk做request→compile→record→ROS→record→独立重建，每步与NativeMotion真实参考/测量对照，含夹爪CONTACT证据和故障terminal。
固定ROS镜像阻断mujoco/scipy/numpy时共享DTO/参考校验可导入运行；科学库不泄漏进runtime。
新增实际应用kind回归明确：停钟导致STOP_TIMEOUT且未应用brake保持原CHUNK/TRAJECTORY；已应用brake后失败为BRAKE。只修诊断事实。

报告`.build/reports/dsh-026/report.md`列精确类型/函数/时间/系数约定、实际host检查与skip、待监督者SDK/native项。只用任务头PROJECT_PYTHON/RUFF；不为host缺ROS/native而安装或修改旧测试。
