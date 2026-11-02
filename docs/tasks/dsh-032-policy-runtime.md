# DSH-032：连续策略RPC、受限执行与MCAP v4

DSH-031验收整合后分派；首轮45分钟，最多一次集中返工。完成S3接口闭环，
不做完整堆叠试采/正式数据/ACT任务训练。所有诊断仍task=not_evaluated。

## 范围和职责

复用028的native runtime/config/control_client/episode/recording/audit及统一日志，
复用031 SubmitFollower/FollowerReference；保留旧MOTION、mock、v1/v2/v3及默认格式2。
允许改native_config/node/episode/control_client/audit及recording/episodes的必要版本分派；
新增职责明确的policy_client.py、native_stream.py、必要时native_follower_audit.py，避免继续堆大模块。
新增配置native_policy.yaml、MCAP v4 schema、对应unit/RPC/真实ROS测试和操作文档。

允许完善contracts/policy/v1/policy.proto及tools/contracts/generate_policy_sdk.py；
新增ros-free的components/learning/src/dual_arm_learning/policy_service.py、必要的小型配置/会话模块，
tests/doubles/policy中的fake provider/启动入口；根pyproject可加入新src测试路径。
生产组件不能import tests；fake入口注入typed provider给实际服务层，后续ACT provider复用。
不预建空ACT框架，不动态加载Python类/插件，不复制物理循环或新造writer。
允许更新现有learning Dockerfile的protocol目标SDK、增加公共Compose overlay/薄入口和文档；
源码/配置/SDK只读挂载，数据/log可写，fake policy容器无ROS/无GPU。
不修改已验收物理/IDL（除Policy protobuf）、场景、机器人profile、成功阈值或训练依赖。
不调用Docker、联网安装、提交或修改其他工作树。监督者构建SDK/镜像并真实验收。

## Policy协议与服务

首个真实服务前固定时间语义：JointTarget点均相对原Observation.reference_robot_time_ns。
将历史字段time_from_start_ns改为清楚的time_from_observation_ns（保留字段号1），去掉draft误导注释。
补PolicyInfo的chunk_size=60、test_only（使用未占字段号），period=50ms和完整16维SI顺序。
SDK重新生成，报告实际导入路径/descriptor；不得读取旧SDK却假装新协议生效。

Open/GetInfo/Predict/Reset/Close保持原RPC。验证model/processor/profile/obs/action摘要、
session/request/episode/epoch/observation身份、关节/相机顺序、三图320x240 RGB/步长/标定与16维测量。
只传三RGB与测量位置（可选测量速度原协议允许），固定task文本；不传物体真值、示教phase或task evaluator信息。
每次Predict返回完整60点×16；均匀50ms、有限值、合法单位。schema不符不执行。

服务配置驱动、session/队列/消息大小有界、deadline明确。Open按open_request_id幂等；
同request重复的内容一致性可查，旧context响应不复用；Close有界且幂等。
一个小typed provider边界即可，fake provider在tests/doubles，服务层本身无Torch/ROS依赖。
fake输出无GT的可解释小运动，全16维，明确test_only=true；可配置延迟/错身份/断连等故障。
默认runtime拒绝test policy，只有诊断配置显式allow_test_policy=true。

## 连续执行

配置action_source选择旧diagnostic_motion或policy；旧默认行为保持。
初次lead沿panda_stream=.25s，follower每次选5个预测点（约.25s），最多active+pending。
每次用新完整capture，稳定observation_id/载荷一致；不能为同capture伪造新ID。
预测等待不阻塞ROS租约续期/停止/记录；提前预测下一块，按实际anchor继承p/v，不重基准。

诊断采用配置max_chunks=8、总sim/墙钟期限；最后一块显式settle_at_end=true，
等待真实NativeExecution COMPLETED后结束。此有限诊断不代表真实任务终止规则。
每个raw source保持60点/20Hz，不插入常量替换左臂或改预测位置；执行器唯一受限跟随。
超时/迟预测、非法响应/模型/旧epoch/断连/超偏差均受控停止，真实反馈冻结后才收尾。

SubmitFollower未知ACK保留command_id和chunk_seq，查询后同ID幂等重发取原FollowerReference，
不盲发新ID。expected requests/receipts按每个command+chunk保存，不能把整体首块当所有后继源。
保留031 FollowerValidationLimits具体约束校验，包含实际延长时域；deadline指接收截止，不要求晚于执行起点。
合法重试返回原accepted_sim/wall/digest；Stop ACK后仍要等待实测安全终态。

## MCAP v4与日志

显式schema_version=4；默认2及历史v1/v2/v3读写不变。
保存BackendInfo、状态/三原图/ObservationIndex、原始NativeCommand、FollowerReference、原生事件/反馈。
若保留严格MOTION准备动作，v4也需明确处理旧AcceptedReference，不混数学类型。
复用v3共同context/捕获/事件/终态检查，以明确reference分派校验和采样，不复制整套700行audit。
按应用时刻用共享record.applied_sample_at(profile)验证actual参考和数值修正；raw系数可独立重建。
只把实际应用的区间当未来标签，不把PENDING未执行或停止后原预测当标签。

Policy请求/会话、摘要、捕获引用、耗时、deadline、失败进统一结构化日志；不往日志塞图像/大数组。
原始有效预测在完整NativeCommand中保留；被拒的protobuf可保存小型.pb诊断+hash，不能让NaN破坏JSON日志。
配置快照明确backend test_only与policy test_only/action_source，fake数据不能冒充正式示教数据。
recording完整性、runtime结果和task结果独立；合法中止可完整录制，未知执行不能伪造成功。

## 公共入口与验收

Compose合并backend_follower/native runtime/fake policy，私有network，source/config/SDK只读挂载。
复用现有launcher生命周期helper；如增加薄入口，需拥有project guard、超时、信号、显式stop及清理；
不复制整套旧大型launcher。文档先ROOT/BUILD_ROOT/DATA_ROOT/ASSETS_ROOT/findmnt/df，再给可执行命令。
proto生成工具可在protocol镜像构建，runtime可挂载更新SDK；明确检查实际import路径/descriptor。

真实验收必须穿过：三RGB+16测量 -> 真实gRPC fake provider -> 真实ROS SubmitFollower ->
真实MuJoCo/ACK/连续参考与实测 -> v4 MCAP -> 独立audit。至少8块，raw60全保留。
另验RPC迟到/身份不符/旧epoch/断连、租约失效、EndEpisode/信号、日志/写盘故障及源/参考/捕获篡改；
缺宿主ROS时如实skip，不用fake物理反馈替代。全部旧unit/ROS/MCAP行为保持，Ruff/diff通过。
报告.build/reports/dsh-032/report.md列API/职责/实际自测/待监督者项，完成退出，不提交。

## 已验收API与本机依赖提示

FollowerReference.sample_at给原系数，applied_sample_at(sim_ns, profile或joints)给规范化实际参考。
validate(record,profile,FollowerValidationLimits(...))需具体限制，不能duck typing跳过检查。
backend follower_protocol.validation_limits(control_cfg,follower_cfg,duration_grid_ns=2ms)可参考，
runtime不能import backend/Ruckig；按配置和BackendInfo验证相同限制及来源摘要。
SDK已提升.build/ros-dev（仅ROS，不含Policy protobuf）；follower-dev为031正式镜像。
原生host自测如需Ruckig，将主目录.build/follower-probe-venv/lib/python3.12/site-packages加入PYTHONPATH，
必须继续使用本候选源码路径。gRPC依赖可用主目录.build/learning-dev-venv/bin/python，
或现有protocol开发环境；不得把ROS依赖带入learning生产组件。
