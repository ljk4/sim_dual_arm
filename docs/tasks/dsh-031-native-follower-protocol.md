# DSH-031：受限跟随公共协议与ROS后端

DSH-030已验收合入，API已核对。首轮45分钟，最多一次集中返工。
本项不接runtime/MCAP4/Policy RPC，不采正式数据或训练。旧33个ROS IDL必须逐字不变。

## 交付和范围

新增标准库 contracts/python/src/robot_contracts/follower_reference.py；
ROS FollowerReference.msg、SubmitFollower.srv及CMakeLists注册；
libraries/ros_support/src/robot_ros_support/follower_conversions.py；
backend_sim/follower_protocol.py，原worker/worker_owner/node/backend_config/backend_cli必要接线；
configs/simulation/backend_follower.yaml，可选follower配置解析；
对应unit/CDR/真实ROS测试及docs/native-follower-protocol.md。
允许修改现有backend-sim Dockerfile增加builder/follower-dev目标，以及新增两份Ruckig哈希lock和来源JSON。
不修改旧NativeRequest/AcceptedReference/NativeExecution语义，不复制NativeController或另造物理线程。
不改runtime、Policy、teacher、场景和硬限位，不联网安装、调用Docker、提交或修改其他工作树。

## 共享记录和类型

新FollowerReference是独立类型；不伪装为严格逐点通过的AcceptedReference。
保留完整raw NativeRequest（只允许STREAM_FIRST/NEXT）、显式曲线、原观测/anchor/window、
算法与采样规范化版本、实际限制参数、raw/restricted端速、延长时域、接受sim/wall时刻和canonical digest。
身份/维度/时间/有限值先做syntax校验；validate(record, profile, control/follower constraints)再独立重建
030的PreparedFollowerPlan，复用唯一标准库端速/刹车/连续偏差规则，元数据不能靠调用方自报。
保存系数原采样和实际应用参考的数值规范化关系；按030共享采样规则可重建actual参考及修正量。
记录sampler版本，未知版本拒绝；不能静默变更旧记录含义。
纯DTO不导入Ruckig/NumPy/SciPy/MuJoCo/ROS，不使用pickle、动态Python类型或JSON包裹替代明确字段。

canonical digest覆盖全部identity/raw source（含未选尾部）、系数、限制/模式/版本及接受时刻。
同一块合法重试返回原接受时刻/digest，不按新的测量重算已接受记录。篡改任何受保护字段须失败。
新转换显式处理has/空值/真实0时刻、整数ns和每轴SI单位，不用字段反射批量复制。

## ROS服务与已有反馈

SubmitFollower.request使用已有NativeCommand（模式由服务入口明确）；返回accepted/reason、
has_execution+NativeExecution、has_chunk+NativeChunkStatus、has_reference+FollowerReference。
旧33个IDL/常量/转换原文不变；不另造重复feedback消息。
原NativeExecution与QueryNativeCommand/QueryNativeChunk描述实际运动，仍可用于新模式；
它们不要求曲线必须逐点通过raw targets。原SubmitNative保持STRICT。

新 `/robot/follower_references` 发布具体FollowerReference；旧accepted_references只含严格记录。
未知ACK仍同ID查询+幂等重发取记录，不能新ID重复执行。跨服务/跨算法同ID应被指纹/模式规则拒绝。
源请求按context/lease/profile/obs/order和截止期限完整验证，不能借新入口绕过030物理规则。

## 同一个worker和生命周期

原有规划队列增加显式模式/请求封装，不复制planner；公开submit_follower，其余原API保持。
规划只读快照、Ruckig在planner线程，admit/apply/step/measure仍唯一owner。首块submit、后继queue_chunk。
新引用队列有界，关键接受/退休/终态不能静默丢；旧队列不混消息类型。
Future取消、lease续期/失效、Stop/Release、backend故障、reset旧epoch、关闭顺序沿027/030保证。
旧任务的原始请求缓存及幂等重查不能因新模式回退；同command不同chunk的完整源分别保留。

backend增加可选follower配置路径；原backend.yaml不启用也正常工作。
新backend_follower.yaml引用相同scene/profile/control/stream和panda_follower.yaml；相对路径按文件解析，
拒绝未知键、保存完整有效配置/来源hash/依赖版本。仅启用后公布bounded_target_follower_v1 capability。
原公共launcher支持--config/--image，优先复用，不复制生命周期脚本。

## 固定依赖构建（监督者准备的输入）

Ruckig Community 0.15.3、MIT，源码hash：
48f68535080a594e45275201896c3a71123ac4a64c668eb3cf10b8bc42f34183。
监督者输入在主目录（只读复制到本工作树对应deploy文件）：
- `/home/ljk/work/sim_dual_arm/.build/reports/s6-preflight-20261101/ruckig-requirements.lock`
- `/home/ljk/work/sim_dual_arm/.build/reports/ruckig-image-preflight-20261101/build-requirements.lock`

构建工具固定scikit-build-core0.9.10（新版不兼容该源码的cmake.targets）、nanobind2.4.0、
cmake3.31.6、ninja1.11.1.3及锁内传递依赖。所有安装必须require-hashes。
增加独立builder阶段装编译器/固定工具，用no-build-isolation、no-deps构建锁定sdist；并行编译最多2。
follower-dev从mujoco-dev延伸，只复制生成wheel和来源记录、no-index安装，不留编译工具。
旧15个native依赖、mujoco-dev及默认mock目标保持；源码/SDK/config/assets/data继续挂载。
构建中保存实际wheel SHA/包版本及构建工具；不要编造镜像ID，实际ID由监督者验收记录。
本地follower-dev预检镜像已验证ABI/2ms时域可用；本项需提供正式可重建配方，不能只引用预检产物。
DSH只写配方与检查，真实Docker build由监督者执行。

## 验收

- DTO无science依赖、canonical digest/元数据篡改及独立实际参考重建；真实新CDR双向往返，原33IDL hash不变。
- 真实ROS服务完成3块FOLLOWER，保留60×16 raw、稳定重试、退休/整体终态和系数/实际值；不靠fake执行对象。
- 模式混用/旧context/lease过期/reset/modified retry/迟结果/队列背压/有界停止及清理。
- 公开Compose入口用正式配方镜像运行；验收真实退出码、log/run/status和独立客户端，所有本次资源清理。
- 原unit/native、真实ROS及v1/v2/v3 MCAP回归不退化；Ruff/diff通过。DSH自测如实标宿主和skip。

报告写.build/reports/dsh-031/report.md，列类型/职责/自测与待监督者项，完成后退出，不提交。

## 已核对的030 API

- `prepare_follower_plan(..., is_final=bool, duration_grid_ns=2_000_000)`；PreparedControlCommand.kind=FOLLOWER。
- `PreparedFollowerPlan.plan`保留原系数，`sample_at`返回未规范化采样；实际应用使用唯一标准库
  `normalize_follower_positions(sample.positions, profile.joints)`，corrections=sampled-applied。
- MotionSnapshot提供sampled_reference_positions/velocities及reference_boundary_corrections，
  reference_positions是严格硬范围内的实际下发值；NativeExecution沿用实际下发值。
- 新FollowerReference sampler应返回或明确暴露raw采样、actual参考及修正量，按同函数重建；
  为位置边界规范化声明固定版本，并纳入canonical digest。
- 独立实际双向边界controller测得142个非零修正tick/方向，raw端点约越界2.85e-11m，actual端点为0/.08。
  旧严格路径、MOTION/CHUNK指纹、33IDL均未改变。
