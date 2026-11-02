# DSH-019：命令状态查询、执行时间与反馈记录

2026-11-01。用户已恢复任务。S2已验收，DSH-018完成多相机profile/记录；本项补S3协议与mock证据，不实现MuJoCo物理控制、reset或Policy。首轮25分钟、最多一次10分钟定点返工；完成一次必要自测后写报告并立即退出。

## 目标与兼容性

新增公共命令状态查询与参考/测量反馈，使runtime能读取命令实际阶段及执行时间，不把ROS goal接受视为完成。此时mock的参考等于合成测量，文档明确它不证明物理跟踪。

**不得修改任何现有msg/srv的字段或现有ExecuteTrajectory.action结构**，避免破坏旧MCAP与调用者。通过新增CommandStatus.msg、QueryCommand.srv及独立command_feedback话题扩展；BackendInfo仍protocol_major=1，新增能力command_query/command_feedback_v1。reset/epoch的运行实现留后续物理控制任务；不新增返回成功但没有物理效果的reset占位接口。

## 允许范围

- 新增 `contracts/ros/robot_api_msgs/msg/CommandStatus.msg`、`srv/QueryCommand.srv`，更新同包CMakeLists；
- `contracts/python/src/robot_contracts/models.py`，允许新增同包 `status.py` 集中纯领域校验；`libraries/backend_core/src/robot_backend_core/engine.py`；
- `libraries/ros_support/src/robot_ros_support/conversions.py`；
- mock `tests/doubles/ros/mock_robot_backend/mock_robot_backend/node.py`；
- runtime `control_client.py`、`node.py`、`audit.py`；
- 新增 `tests/unit/test_command_status.py`、`tests/integration/test_command_status_ros.py`；现有 backend_engine/control_client/recording/episode测试若确实受影响可小幅更新，不能删除旧行为断言；
- `contracts/README.md`、`docs/architecture-and-interfaces.md`、`docs/episode-recording-format.md`、`docs/simulation-control-design.md`、`docs/README.md`、`README.md`。

不修改配置、profile、依赖、镜像、Compose、日志/队列库、生成代码、构建保护或其他工作树。不运行Docker、不安装依赖、不申请权限、不启动其他代理、不提交Git。需要额外生产文件在报告说明，不擅自扩范围。监督者负责真实生成SDK、ROS/MCAP、公开入口与历史记录验收。

## 状态模型与时钟

1. 新增冻结领域 `CommandStatus`：context、command_id、command_seq、observation_id、phase、reason、sample_time_ns、accepted_at_ns、started_at_ns、elapsed_ns、joint_names、initial_positions、reference_positions、measured_positions、tracking_error、progress。数组全为tuple；不暴露可变Execution/history。initial_positions是该命令开始参考，用于重建第一段插值，后续反馈不变。所有时间为整数ns；accepted/started/sample使用机器人时钟，elapsed为执行时长。当前mock立即开始，accepted_at=started_at合法；物理后端后续实现实际STARTED，不能沿用mock的墙钟插值。
2. ROS CommandStatus字段相应采用RunContext、string/uint64、Time/Duration、string[]/float64[]/float64 progress；含相同身份和顺序。转换集中在ros_support；字段命名稳定明确，单位写注释。tracking_error固定为reference-measured，progress有限且[0,1]。
3. mock现有插值仍按单调墙钟，不偷偷改成物理实现。必须独立记录机器人时钟的accepted/started/采样/事件发生时刻；`submit(robot_now_ns, steady_now_ns)`已分离。tick可增加显式robot_now_ns（保留旧单参数测试语义）；stop/release/expire/cancel等在真实ROS调用时传入实际机器人时间。不可用单调墙钟伪装机器人时间。CommandEvent的stamp改为捕获的转换发生时间，不能延后到drain时的发布时间。
4. 活跃状态快照与同一次tick的位置一致；terminal保存最后状态/时间，不随后续命令运动改变。query是只读，不推进插值、不产生重复事件、不执行/续租/恢复命令。历史上限沿用max_history；查询不能无限缓存。lease失效即使机器人时间不动也取消；已结束记录可以在租约失效/释放后查询。
5. query要求完整原command.context与command_id匹配。历史记录可跨同generation内的控制权交接查询，但只允许匹配其原context；后端generation变化拒绝STALE_CONTEXT。未知或被逐出的ID明确RESULT_EXPIRED，不猜测成功。不要求活跃lease；不允许拿同ID配其他episode/control_epoch读取记录。相同ID幂等和冲突拒绝规则不退化。

## ROS与runtime

- QueryCommand.Request包含context/command_id；Response含success/reason/status。失败status仅默认值，调用者不能作为有效结果使用。mock发布能力并提供服务；服务读取快照，无I/O或等待。
- 活跃期间每tick将同一捕获快照发往 `<robot_namespace>/command_feedback`；terminal至少发布一次；空闲不无限重复terminal。独立高频参考进入MCAP，不把数组写进普通JSONL日志。保留现有ExecuteTrajectory反馈接口。
- runtime `ControlClient.query_command(context, command_id)`提供有界调用与身份/数组/有限性/时间/阶段校验，超时或未知状态清楚失败。日志只记录查询意图/结果身份、阶段、原因、进度等标量。
- **本项不自动重发、不把query COMPLETED改写成丢失的action结果、不掩盖原execute超时**。默认execute行为不变，清理仍Stop/等待真实terminal/Release。查询恢复编排在物理接入时再做；不能为了这项改episode状态机。
- runtime订阅command_feedback、按sample_time记录，校验当前完整context、关节顺序、数值与时序；坏反馈触发现有episode故障/停止路径，不能只是忽略后继续成功。旧context反馈不进入当前episode判断。共用小而清楚的status校验/转换函数，避免runtime与audit各写一套。
- audit只在存在新反馈流时核对：身份/关节数组与有限性、reference-measured、时间/progress/阶段、command_seq/observation_id关联、参考与实际接受轨迹及STARTED对应，terminal不回退到STARTED。新反馈不影响历史无该topic的数据；禁止修改旧消息结构/历史文件来伪装兼容。保持流式读取，不保存整段反馈数组。

## 自测、交付与验收

自测用PROJECT_PYTHON/PROJECT_RUFF。至少覆盖活跃/terminal冻结、query不推进、lease失效/停机器人钟、history逐出/上下文错配/跨租约查询、重复命令不重发事件、不同robot/steady时钟、参考/测量/误差/进度。新增真实ROS集成测试供监督者运行：公开QueryCommand、command_feedback与MCAP审计，错误身份/NaN/错参考/时间回退等反例；宿主缺ROS可skip，不能用自制ROS替身冒充真实通过。

监督者使用隔离生成SDK和源码挂载，运行现有ROS回归、公开collect、历史v1MCAP原件读取、反馈反例及实际服务超时/错误上下文探针。dsh交付 `.build/reports/dsh-019/report.md`：修改范围、自测、未完成项、接口字段与时钟、旧记录兼容边界。保持函数短、显式依赖、单向状态所有权；不要加入泛型总线或多层注册框架。

## 独立审查后的补充

首轮真实探针确认停钟时长失真、中间参考/进度/终止反馈审计遗漏、非home连续episode回归和关键查询日志故障被吞。唯一返工增加以下明确约束，原目标与预算不变：

- 新增消息尚未验收，补 `execution_clock`（mock为steady，后续物理为robot）与 `interpolation_version`（当前linear_v1），使执行时长与采样时钟各自可追溯；旧IDL继续保持字节不变。
- `elapsed_ns`表示生成实际参考时使用的执行时长，不要求等于机器人采样时间减开始时间。停止冻结最后实际执行参考与对应时长；进度、参考和时间不能互相矛盾。
- 逐样本关联已发送/接受轨迹；声明反馈topic时要求已接受命令有terminal反馈，并在原有有界收尾等待中等待它入队。历史无该topic的数据维持原要求。
- 首命令不假定profile的home；episode可从前次停止保留的位置开始。查询关键日志错误必须传播；错误响应保留原原因。
- 为可读性额外允许新增 `runtime/feedback_audit.py`，将反馈审计提取为具体职责；共用纯领域轨迹关联校验放 `contracts/status.py`。不创建通用框架。
