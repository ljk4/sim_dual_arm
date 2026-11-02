# DSH-027：可运行的 ROS 原生仿真后端

2026-11-01。025/026已验收，默认SDK已包含新类型。用户授权持续推进到训练/仿真验证闭环。
本项交付物是可启动的真实物理后端，不是又一个协议草案。首轮45分钟，最多一次15分钟集中返工。

## 范围

允许新增 backend_sim 包内 backend_config.py、worker.py、render_worker.py、node.py、必要的职责清楚的小模块；
新增 configs/simulation/backend.yaml、deploy/compose/native_backend.yaml、tools/run/native_backend.py、
docs/native-backend.md、tests/unit/test_backend_config.py、tests/integration/test_native_worker.py、
test_native_backend_ros.py。可更新 docs/README.md、README.md、docs/simulation-control-design.md。
允许小幅扩展 control.py/control_state.py/protocol.py 及对应原测试，原因限下文三项；保留既有API。
不修改IDL/profile/物理参数/完成或限位规则/镜像依赖，不改runtime、Policy、MCAP，不做teacher或训练。
不安装依赖、不调用Docker、不提交Git、不改主工作区。原生Python已可运行MuJoCo/SciPy测试，不得把native全部skip。

## 模块边界

1. 物理owner线程内部创建Scene、PandaIO、NativeController，唯一读写live MjData并执行mj_step。
   2ms步长仍由scene配置；按单调墙钟实时节奏运行，迟到有界追赶，不跳物理步，不把wall当sim。
   ROS回调、规划和渲染不读取live data，不调controller。worker所有读写均经显式固定操作的有界队列和Future。
2. 单规划worker仅接不可变profile/config/最后hold参考或anchor，运行prepare_request；不在物理线程跑SciPy/哈希大对象。
   最多一个在途预编译，准入前在owner重验context/lease/deadline/固定anchor。拒绝过期排队请求，关闭时有界解决所有Future。
   不做通用插件、任意callable RPC、动态getattr命令执行框架。
3. 独立renderer线程只创建/使用自己的SceneRenderer/MjData/EGL上下文。复用现有rendering.py。
   owner在20Hz捕获完整Snapshot及不可变测量/身份，renderer同一snapshot一次产出三路RGB；不再读live state。
   snapshot队列和输出队列有界，可丢旧未渲染capture并计数；禁止静默丢控制事件/响应。共享model只读。
   render异常/过长停滞需要报告故障和受控停止，不能假ready继续无图运行。
4. node.py只负责ROS类型转换/发布/服务与生命周期。服务可用async callback等待rclpy Future桥接；
   不在SingleThreadedExecutor内阻塞等物理线程完成，不靠busywait/sleep轮询服务结果。
   不把已交给owner但ACK丢失的请求声明为“保证未执行”，保留ID供查询；不能新ID自动重发。

## 必要的小扩展

- owner内部 system_stop(reason, wall_ns)：用于进程关闭、日志/队列/渲染故障，绕过已过期lease但复用既有真实刹车/终态逻辑。
  不暴露无认证ROS stop接口。系统软件故障必须使进程/运行失败，即便物理命令最终CANCELED。
- 从现有有界history查询原不可变PreparedControlCommand的清楚只读API，不泄漏可变Record。
  MOTION幂等重试必须复用原编译包，不能因当前位置改变重新拟合而误报冲突；流也同理。
  原request与保存包完整比对，变化字段不能被当幂等。不要建立第二个无界权威缓存。
- accepted_reference允许用历史真正accepted_sim/motion_seq等事实构造稳定回执。
  重试返回同一reference_digest/accepted时刻/曲线；不得用当前latest snapshot冒充原接受时间。
  保留026现有accepted_reference API及测试，可加明确helper。完整request保存方式须有界，淘汰/epoch与history一致。

## ROS公开入口

namespace可配置，默认 /robot。BackendInfo protocol_major=1、backend_kind=mujoco、test_only=false；
声明 native_motion、continuous_chunks、exact_capture、simulation_reset 等实际能力，不声明未实现的旧linear action。
复用原 /info、/state、/cameras/{name}/frame 和Acquire/Renew/Release/Stop。
提供 SubmitNative、GetNativeAnchor、QueryNativeCommand、QueryNativeChunk、ResetSimulation 服务，snake_case名称与类型对应。
提供 /native_events、/native_feedback、/accepted_references；共享DTO/ROS转换只用026实现。
无需伪装旧ExecuteTrajectory action；runtime将另项接新服务。旧mock路径不变。

所有三图和测量属于同一 capture：RobotState.state_seq == CameraFrame.frame_seq，context一致，
RobotState.stamp、joints.header.stamp、image.header.stamp、camera_info.header.stamp均为捕获sim时刻，非完成渲染时刻。
三路320x240 RGB及camera_info遵循profile标定。BackendInfo初始/租约context变更/reset后及时更新。
state可早于图发布，后续runtime按capture精确配对；旧epoch渲染结果不得泄漏到新epoch。
clock_epoch变化后capture_seq可清零但配对键必须包含backend实例/epoch/episode/control身份。
终态feedback来自ControlStatus冻结snapshot，不能用最新live测量覆盖。实时RobotState继续hold步进。
高频feedback每capture即可；状态迁移/terminal/accepted_reference不能因渲染丢帧漏掉。

## 配置、日志、生命周期

backend.yaml严格校验未知键/有限值/bool/队列/时限/路径，所有路径相对该配置文件或显式CLI根，不能依赖cwd。
引用现有panda_stack、panda_control、panda_stream和robot profile，不复制参数。显式seed、capture_hz、实时倍速（默认1）、
队列容量、planner/renderer健康期限、关闭预算、namespace、log settings。物理步长只从scene读。
CLI指定 --config、--assets-root、--asset-summary、--run-root、--run-id，拒绝覆盖已有run。
复用唯一EventLogger，记录启动/资产与配置摘要/队列计数/接受和阶段/故障/reset/关闭，关联context/command/capture。
不逐图逐500Hz写JSONL、不新建日志writer。logger故障同样触发system_stop和非零退出。
共享数据/通知队列皆有明确容量及满队列行为，worker异常传到主线程，不留永远未完成Future。
关闭依次拒新任务、受控刹车/墙钟超时、renderer在所属线程close、join、logger close；所有等待有界。
run目录写清楚summary/status JSON（成功关闭或失败原因/计数/版本），可读日志与异常栈保留。

Compose使用既有backend-sim:mujoco-dev和dev_ros entrypoint，只读挂载源码/SDK/资产/配置，run-root可写；
配置NVIDIA EGL/compute/utility/graphics，独立项目网络，不用host网络；复用fastdds.xml UDP。
launcher只管理自己本次project/container，信号和非零传播正确，复用现有tools/run中合适的路径/构建检查工具；
不要复制整个sim_check/mock生命周期框架。先提供简明foreground启动/停止命令也可，只需归属明确、无遗留。
docs先给可复制ROOT/BUILD_ROOT/DATA_ROOT/ASSETS_ROOT变量和findmnt/df挂载验证，再给实际启动、服务/话题、正常和故障退出用法。

## 验证及交付

运行现有697 unit与97个无需ROS native（工具头环境及固定只读资产）；报告真实通过/skip数量。
新worker真实物理测试至少覆盖：单owner、测量随步进变化、队列有界、幂等稳定回执、租约过期刹车、
reset换epoch清队列、关闭解决Future、planner异常、render故障/慢消费者（可注入专用测试renderer，不能冒充GPU验收）。
三视图真实EGL和ROS服务由监督者Docker验收，但要提供实际可执行pytest入口，不能只写文档/自述。
ROS测试需正常MOTION和连续流、查询terminal冻结、重复/改载荷、lease/reset旧context拒绝、capture四时间与ID一致、节点异常关闭。
已有unit/native/旧ROS不退化，Ruff和git diff --check通过。文件职责清楚，约500行以上考虑按真实职责拆分，
显式类型/命名/时间单位，关键边界解释原因，拒大而全管理器/反射/压缩一行代码。
交付 .build/reports/dsh-027/report.md：文件说明、精确API、线程/队列/故障边界、执行命令/结果、待监督者实测项；完成后退出。
