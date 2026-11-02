# DSH-028：原生运行时与精确观测记录

DSH-027公共入口、真实ROS/EGL及原生控制已验收，源API已核对。用户授权持续推进；本项不采正式数据、不训练。
首轮45分钟，最多一次15分钟集中返工。监督者参数以分派时任务头为准。

## 交付边界

让运行时通过真实ROS后端完成一条诊断运动，并留下可独立审计的MCAP。
保留现有mock入口及v1/v2记录可读性；原有node.py/episode.py已较大，不在其中继续叠加原生分支。
在同一个dual_arm_runtime包内增加职责明确的native_config.py、native_node.py、native_episode.py、
native_control_client.py、native_audit.py；如有必要再拆小模块，不构造通用工作流框架。
复用EventLogger、EpisodeRecorder、BoundedRecordingQueue、既有shutdown和CLI辅助，不新造writer。
原生运行时不得导入MuJoCo、SciPy、NumPy、Torch或backend_sim；只依赖共享contracts/ROS支持层。

允许改observation.py增加明确exact_capture模式（默认旧causal行为不变），recording.py增加显式格式版本，
audit.py做小型版本分派，contracts/python/episodes.py及新增v3/episode_manifest.schema.json。
新增configs/runtime/native_collect.yaml、deploy/compose/native_runtime.yaml、docs/native-recording.md，
以及对应unit/ROS集成测试。可更新docs/README.md和README.md。
不改既有IDL、物理后端、Policy、训练或场景参数；不安装/调用Docker/提交/改主目录。

## 精确观测

- 新模式按完整context和capture编号配对：state_seq等于三路frame_seq，四个stamp严格相等。
  新state先到、旧图后到时，应选最新的完整capture，不能把旧图配给最新state；未完整的一组不输出。
- 缓存、就绪索引及ROS载荷均有界，容量与max_age配置明确。处理乱序、重复、同ID冲突、淘汰和计数；
  跨backend/clock_epoch/episode/control_epoch消息拒绝。不要只在每个stream要求严格递增，导致合法异步到达被误丢。
- 同一capture重复读取返回同一observation_id；新完整capture才增加。不能因API调用次数生成伪新观测。
- 观测引用和真正的RobotState/三图载荷一起取一致快照，避免先选索引、后从已覆盖缓存取错数据。
  无需把RGB内容复制进纯ObservationRef，使用明确的有界载荷缓存和不可变关联。
- 在录制中保存原图、原测量和ObservationIndex；图像尺寸/编码/步长/frame_id/标定与profile一致。

## 原生控制与episode生命周期

使用027的Acquire/Renew/Release/Stop、SubmitNative、QueryNativeCommand及ResetSimulation；
新原生反馈必须按026共享DTO/转换校验，不能伪装为旧linear_v1 CommandStatus。
ROS执行器保持响应；episode工作线程等待有界Future，租约独立按单调时钟续期，不能在等待完成期间停续约。
提交超时/OUTCOME_UNKNOWN（含明确后缀）保留command_id并查询，不用新ID盲发。完整回执/查询不一致或未知结果无法确认则失败并停止。
执行完成只认同context/command的实测终态COMPLETED；FAILED/CANCELED/超时不作成功。
拒绝旧epoch、错profile、假native能力、test_only不符等来源。租约丢失、backend断连、记录/日志故障均走公共停止。state/完整观测还需独立墙钟到达期限，不能因仿真钟冻结而无限保持“新鲜”。
Stop/Release后等待真实终态反馈/查询，有界兜底；不能只收到Stop ACK就宣布机器人已停稳。

先只实现配置驱动的停稳诊断MOTION：配置中每个运动有完整16维目标/时长；不把它当最终ACT连续执行。
连续Policy接续和teacher将在下一项接同一原生入口。共享参考数学只用contracts，不能在runtime重新拟合轨迹。

每次开始可按配置显式reset到指定seed：用临时准备context取lease执行reset，再用返回的新epoch建立实际episode context/lease。
重置和正式控制身份要区分记录，不能复用reset前结果。S3仅配置一个诊断seed，后续批采由独立批次配置分配种子。
复用现有StartEpisode/EndEpisode和/runtime/status，phase和终态语义与既有CLI兼容；支持--once便于公共自动验收。
状态维度仍区分runtime结果、recording完整性、task结果，本项task=not_evaluated。
信号处理只设标志；有界关闭顺序及日志flush沿用现有原则。原生node构造失败也不得漏线程/recorder/logger。

## 可审计MCAP v3

现有默认EPISODE_SCHEMA_VERSION=2及mock行为保持，新增显式NATIVE_EPISODE_SCHEMA_VERSION=3和schema路径映射。
EpisodeRecorder增加format_version默认2，原生调用明确3；保持唯一原生writer/有界写后队列及partial->审计->hash->提交。
v3可沿用v2三维结果字段，明确原生格式版本；完整配置/profile/schema/控制约束快照必须可独立校验。
不把“记录可读完整”和“运动/任务成功”混在一个success布尔中。

录制至少包含BackendInfo、RobotState、三CameraFrame、ObservationIndex、原始NativeCommand意图、
AcceptedReference、NativeControlEvent、NativeExecution。源请求完整保存，不能只录selected targets。
接受记录按公开digest去重但保留重试/未知ACK诊断；同身份不同内容必须审计失败。
原生audit流式读MCAP：验证context/序号/捕获配对、profile/标定、命令与整体/块事件状态迁移、
接受系数/源窗口/限位/digest和实际STARTED/RETIRED/终态事实。复用validate_accepted_reference/sample_accepted_reference。
每条反馈正常参考按实际应用时刻重建并核对p/v/a，BRAKE和terminal hold不能拿原预测计划误重建。
校验测量误差、终态冻结、真实0时刻与has标志；未执行的候选不能冒充训练标签。
合法取消/中断可得到recording.complete=true、runtime非completed；未确认执行的意图不强求伪造ACCEPTED。
缺少必要命令/参考/终态、同ID冲突、篡改、错配观测及计数/hash不符要明确失败，不能提交完整目录。
缓存只留审计需要的有界元数据，不把全episode RGB加载到内存，不复制JSONL/MCAP另一套写入线程。

## 配置、入口与验收

严格配置、相对配置文件路径、SI单位、超时/容量、seed、完整有效值快照；调用方传入profile/observation/action路径，
复用已有load_document/validate_bindings，不引入backend依赖。CLI参数以路径、run身份、--once为主。
Compose复用027相同project/network约定；runtime无GPU、固定dev-ros:stage2依赖镜像，源码/SDK/config只读，数据/log可写。
不要复制一个完整Compose生命周期启动器；提供可与native_backend.yaml合并的服务定义及明确命令即可，监督者实测后再形成闭环launcher。
文档先给ROOT/BUILD_ROOT/DATA_ROOT/ASSETS_ROOT和findmnt/df检查，再给启动、start/status/end、audit和产物说明。

验收：现有unit/native/115旧ROS回归不退化；exact_capture单测覆盖新state/迟图/乱序/冲突/换epoch/缓存淘汰/稳定ID。
真实ROS集成入口运行native node与027后端，验证运动ACK→实测终态→MCAP→独立audit；普通完成/EndEpisode中止/信号中断/
lease丢失/backend断连/写盘背压/日志失败各有明确结果、退出码和产物。
另构造三块真实物理参考的v3 MCAP验证完整原预测保存、退休与终态区别及重建，不能只用fake消息类冒充CDR/物理证据。
共享DTO/精确观测逻辑用无需ROS的unit验证；ROS/MCAP真测由监督者固定镜像执行，报告实际host结果与skip。
Ruff、diffcheck通过；新文件按真实职责组织，可读命名/具体类型/时间单位，避免object+反射大管理器。
交付.build/reports/dsh-028/report.md列API、文件职责、实际检查与待实测项，完成后退出。
