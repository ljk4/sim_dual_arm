# 真机硬件接口边界（Hardware）

本目录定义真机驱动的接入边界。现有工作流运行于 MuJoCo，真机驱动与采集入口待实现。

## 端间产出和消费

| 生产者 → 消费者 | 通道 | 数据及职责 |
| --- | --- | --- |
| Shared → 各端 | 配方、档案、生成接口 | 方案 `binding.yaml` 选择各端配方。Robot Profile、观测和动作档案定义相机、关节顺序、单位及限制。ROS 2 IDL 和 Policy Protobuf 定义线上消息。 |
| Simulation 或 Hardware → Runtime | ROS 2 话题 | 以 `/robot` 为默认命名空间，`info` 提供就绪、Profile、相机和能力，`state` 与 `cameras/<name>/frame` 提供实测观测。`control_events`、`execution_feedback`、`follower_references` 描述实际执行。Runtime 用运行上下文、采样序号和时间配对观测。 |
| Runtime → Simulation 或 Hardware | ROS 2 服务 | `/robot/{acquire_control,renew_control,release_control,stop}` 管理控制权。`get_stream_anchor` 提供交接状态，`submit_follower` 接收原始窗口和分段轨迹，`query_stream_command` 查询状态。后端拥有设备循环、执行和停止。提交响应表示准入，执行反馈描述后续运动。 |
| Runtime ↔ Simulation 的任务功能 | 可选 ROS 2 服务、话题和日志 | `reset_simulation` 复位物理场景，`prepare_task`、`start_task`、`get_task_state` 和 `task_state` 提供仿真任务生命周期。`get_task_chunk` 返回方案 Teacher 的动作。Renderer 逐帧像素统计写入后端日志，由当前采集导出读取。 |
| Runtime → Learning 转换 | 共享文件 `var/<robot>/<task>/datasets/raw/` | Runtime 产生 MCAP、任务结果、RGB、标签、质量结果及 `formal-select.json`。当前转换器消费选定 episode 的导出文件，生成 LeRobot 数据集。MCAP 保留原始观测和执行证据，供回读、审计与重导出。 |
| Learning → Learning 策略服务 | 数据集、模型文件 | 训练读 LeRobot 数据集并输出 `model.safetensors`。策略服务加载权重和归一化数据。 |
| Runtime ↔ Learning 策略服务 | gRPC，默认 `learning:50051` | Runtime 获取策略元数据、打开会话，用 `Predict` 发送同步关节与相机观测，收到带时间偏移的关节目标块后选窗并经 ROS 2 提交给执行后端。Teacher 采集可以独立运行。 |

公共接口见 [Robot API](../shared/common/interfaces/ros/robot_api_msgs/README.md)。仿真扩展见 [Sim API](../shared/common/interfaces/ros/sim_api_msgs/README.md) 和各方案的 `extensions/ros/`。学习接口见 [Policy API](../shared/common/interfaces/policy/README.md)。

本机容器中，Runtime 同时加入 `ros` 和 `learning` 网络，Simulation 加入 `ros`，Learning 加入 `learning`。手动仿真与运行端默认使用 ROS 域 42，批采为每个实例分配独立域和命名空间。磁盘产物通过宿主机 `var/` 挂载的容器路径 `/data` 交接。

## 当前耦合点

- 通用 Policy 运行按 Robot API 能力接入。仿真复位、任务判定和程序示教通过仿真路径启用。当前采集方案解析要求 simulation Binding、Sim API 和 Teacher 扩展。见 [每局准备](../runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/episode_runner.py)和[采集方案解析](../runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/offline/scenario_selection.py)。
- 当前导出与资格判定依赖 MuJoCo 分割数据和真值任务状态。并行采集入口启动独立 MuJoCo 实例，属于仿真批采。见 [导出](../runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/offline/export_episode.py)和[并行采集入口](../record_parallel.sh)。
- Robot API 中部分时间字段名为 `sim_time_ns`。真机需提供统一的后端单调采样时钟和重连 epoch 语义。目前运行配方按 2 ms 规划周期验收，接入真机前需对齐其执行时钟和轨迹交接网格。

## Runtime 分层方案

ROS 2 Runtime 的主包提供公共运行能力。`dual_arm_runtime/simulation/` 提供仿真任务和 Teacher 支持。真机示教与验收将在有实际后端时接入同一 Robot API。

| 模块 | 输入和产出 |
| --- | --- |
| 公共运行层 | 根据 Profile 和能力接入后端，管理时钟上下文、租约、观测同步和 episode。通过 gRPC 获取策略目标，负责选窗、轨迹生成、提交和反馈管理。保存 MCAP，执行图像齐全、观测对齐、可解码性和执行证据检查。 |
| 仿真运行层 | 调用仿真复位、Teacher、任务生命周期。从仿真状态和分割日志生成任务与可见性验收结果。宿主机批采入口启动和回收独立后端，仿真运行层消费批次配置。 |
| 真机运行层 | 从操作者或遥操作接收示教及开始、结束信号，组织单台机器的 episode。任务验收消费人工确认或外部感知结果。设备通信、控制循环、硬限位、看门狗与停止由 `hardware/` 中的执行后端负责。 |

## 轨迹生成与执行边界

Policy 和 Teacher 共用同一提交路径。Runtime 选窗后，从继承的交接位置和速度生成受限的分段多项式，用 `SubmitFollower` 提交原始窗口、锚点和 `ReferenceSegment` 系数。Learning 通过 gRPC 输出相对于观测时间的目标点。

执行后端校验租约、上下文、位置和速度连续性、连续位置/速度/加速度限制、全窗偏差及端点制动条件。通过后按本地时钟采样生效系数，并发布 `FollowerReference`、控制事件和实测反馈。超时、失联或跟踪失效时由后端安全停止。导出按生效参考及开始、退役和制动事件生成 `observation + 50 ms` 的动作标签。

## 接入准则

- 真机的设备 IP、序列号、账号凭证与单机标定参数属于本地私有配置，严禁提交至代码仓库。
- 在确认具体机械臂/夹爪硬件驱动、开源许可与安全急停边界前，不在本目录创建占位空模块或假驱动。

目录导航：[上级目录](../README.md)
