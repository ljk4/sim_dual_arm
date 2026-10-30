# 无真机采集与推理的仿真环境设计

更新日期：2026-10-29。本文描述目标 Gazebo 后端。当前 backend-sim Dockerfile 已提供测试专用 mock target，物理场景与 Gazebo 适配节点仍待阶段 4 实现；已运行的 mock 链路见 [阶段 1～2 指南](stage2-quickstart.md)。

## 1. 仿真环境在系统中的位置

职责与通信约定以 [环境边界与接口设计](architecture-and-interfaces.md) 为准：真实驱动位于 backend-real，仿真位于 backend-sim，公共应用位于 runtime，模型位于 learning。本文进一步描述仿真后端。

仿真后端实现公共 Robot API，替代真实机械臂与相机，产生图像、关节状态、TF 和时钟，并执行来自 `runtime` 的受控轨迹。`runtime` 仍负责观测整理、原始数据记录、推理请求、动作转换和执行调度，`learning` 仍负责数据转换、训练和模型服务。

这样无真机时可以走完整链路：

```text
仿真双臂/相机 → ROS 公共观测 → runtime 原始采集 → 训练数据转换 → 模型训练
       ↑                                                ↓
       └──── ROS 控制接口 ← 动作适配 ← 模型推理服务 ←─────┘
```

真实后端与仿真后端使用相同语义的数据契约，分别由配置映射具体话题、关节和控制接口。接入真实设备后仍需校准、控制参数和真实延迟验证；仿真闭环成功不代表策略已能直接用于真机。

## 2. 仿真器选择

| 候选 | 与当前目标的关系 | 首版选择 |
| --- | --- | --- |
| Gazebo Harmonic | 官方提供 Jazzy 配套安装路径，ros_gz 与 gz_ros2_control 可用于接入 ROS 观测和控制 | 首选，优先减少 ROS 集成工作 |
| MuJoCo | 有成熟的原生 Python API，适合后续复用具体学习任务；ROS 接口、时钟和控制器仍需适配 | 当目标任务已有更完整的 MuJoCo 实现时重新评估 |
| Isaac Sim | 面向另一类仿真与渲染需求，当前官方最低配置列出 16 GB 显存 | 本地 6 GB 方案不以它作为首版基线 |

以上是结合当前 ROS 接口验证目标的设计取舍，不是三个仿真器的性能测试结论。[Gazebo 配对表](https://gazebosim.org/docs/harmonic/ros_installation/)、[MuJoCo Python API](https://mujoco.readthedocs.io/en/stable/python.html)、[Isaac Sim 要求](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html)

首版采用 Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic。若未来改为 Humble，需要连同 Gazebo 和控制插件一起重新选择；官方默认推荐配对为 Humble / Fortress。

## 3. backend-sim 镜像的候选依赖

| 类别 | 依赖 | 用途 |
| --- | --- | --- |
| 基础 | 与 runtime / backend-real 相同的 Jazzy ROS 基础层和公共接口包 | 保持 ROS 通信与工作空间一致 |
| Gazebo / ROS 集成 | `ros-jazzy-ros-gz` | 安装推荐的 Gazebo 库及 ROS 集成包 |
| 传感器桥接 | `ros-jazzy-ros-gz-bridge`、`ros-jazzy-ros-gz-image` | 时钟、图像、相机参数和所需服务的桥接，可能由元包传递安装 |
| 仿真控制 | `ros-jazzy-gz-ros2-control` | 将仿真关节连接到 ros2_control |
| 控制器 | `ros-jazzy-ros2-control`、`ros-jazzy-ros2-controllers` | 状态广播、关节轨迹和夹爪控制 |
| 机器人描述 | xacro、URDF/SDF、robot_state_publisher、机器人模型与网格文件 | 双臂、夹爪、惯性、碰撞体和 TF |
| 场景 | SDF world、传感器/物理/场景插件、桌面和物体模型 | 相机、碰撞、重力、抓取与复位 |
| 图形运行库 | OGRE2 对应的 OpenGL/EGL 运行时与容器 GPU 访问 | 相机渲染；GUI 作为可选启动模式 |
| 开发 | colcon、rosdep、编译器、CMake | 构建机器人描述和项目仿真适配包 |

候选包需要在镜像实现时实际检查 apt 依赖、模型插件和启动流程。首版优先使用 ROS 仓库提供的配套二进制包。[官方安装方式](https://gazebosim.org/docs/harmonic/ros_installation/)、[Jazzy 控制插件安装](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html)

`backend-sim` 不需要安装 LeRobot、PyTorch 或真实相机 SDK。机器人描述与几何资产可由 backend-real 和 backend-sim 共用；runtime 只依赖公共接口和语义 profile。同一仿真运行中的 controller_manager、关节状态和 TF 发布者统一在仿真后端启动，避免重复发布。图像和 `/clock` 使用 ros_gz 桥接，关节控制通过 gz_ros2_control 提供内部 ROS 控制接口；后端适配器再发布/接收公共 Robot API，补齐实例、时钟 epoch、控制 lease 和事件反馈。runtime 不依赖 Gazebo 原生接口。

## 4. 首个场景与演示来源

首个场景采用可替换的双臂模型、两个夹爪、桌面、简单物体和一台固定 RGB 相机。优先选择有可用开源描述、惯性/碰撞模型和适当许可的机械臂模型，关节数与名称通过配置映射，不预设它就是未来真机型号。

场景需要能够启动、暂停、复位以及恢复机器人/物体初始状态。模型和材质等资产应保存来源与版本，并尽量在构建或准备阶段缓存，避免每次启动临时下载。

仿真本身不会自动产生用于模仿学习的演示，因此按以下顺序增加数据来源：

1. 预设的关节轨迹与夹爪开合脚本，用于验证动作、关节反馈、相机和记录链路。
2. 基于该具体场景调通的简单示教脚本或人工遥操作，用于产生有明确任务结果的 episode。
3. 按需加入末端控制、IK 或 MoveIt；用户选择的遥操作设备可以作为可选依赖。

首个任务可以从固定物体位置的抓取/搬运开始，再扩展为双臂配合或物体交接。第一阶段的关节运动脚本只能证明接口连通，不能直接当作已经验证有效的抓取示教或算法基准。

## 5. ROS 接口约定

下表是待实现的规范化接口草案；通过 launch 配置和适配器映射实际仿真/真机名称。接入真机时还要匹配动作单位、关节顺序、控制频率与限位。

| 数据/控制 | 建议接口 | ROS 类型/含义 |
| --- | --- | --- |
| 后端信息 | `/cell_01/robot/info` | 公共 BackendInfo：实例、profile/schema、时钟 epoch、能力与就绪状态 |
| 双臂关节状态 | `/cell_01/robot/state` | 公共 RobotState 包装标准 JointState，携带实例、epoch 与 state_seq |
| 固定 RGB 相机 | `/cell_01/robot/cameras/front/frame` | 公共 CameraFrame 包装 Image / CameraInfo，携带 epoch、frame_seq 与标定信息 |
| 坐标树 | `/tf`、`/tf_static` | profile 定义的公共 frame ID |
| 动作入口 | `/cell_01/robot/execute_trajectory` | 带上下文/lease 的公共 action；后端内部适配轨迹与夹爪控制器 |
| 动作反馈 | `/cell_01/robot/command_events` | 接受、开始、完成、取消、失败与可提供的目标反馈 |
| 时钟 | `/clock` | Gazebo 单向发布的仿真时钟 |
| 世界复位 | `/cell_01/robot/reset_world` | 后端 capability；episode 复位与记录由 runtime 编排 |

控制权 lease、上下文校验与动作反馈由仿真适配器实现。轨迹发送、取消和替换必须与控制器语义一致；模型的动作块经 `runtime` 转换后再执行，不能假设任意频率覆盖 action goal 都能形成连续动作。若真机使用左右独立控制器，适配器负责拆分与统一时间调度。

Gazebo 真值（如物体精确位姿）可用于评估成功率或生成示教；默认不进入策略观测，以便后续使用真机可获得的图像和机器人状态。

## 6. 时钟、复位与采集一致性

- 仿真后端只提供一个 `/clock` 源，并以 Gazebo → ROS 单向桥接；相关 ROS 节点启用 `use_sim_time`，由 backend info 与部署配置进行一致性检查。不要让仿真和 bag 回放同时向同一运行发布时钟。[ros_gz 时钟说明](https://github.com/gazebosim/ros_gz/tree/jazzy/ros_gz_bridge)
- 采集节点等待首个时钟、相机帧、关节状态、必要 TF 和控制器就绪后再开始 episode。rosbag2 的 `--use-sim-time` 可使记录时间跟随 `/clock`。[rosbag2 仿真时间说明](https://github.com/ros2/rosbag2#simulation-time)
- 每次复位先停止并取消当前动作、结束记录、拒收旧推理结果，然后重置场景，恢复初始状态并清空模型历史/动作队列，再创建新 episode。请求与响应携带实例、clock_epoch、episode、control_epoch 与序号；复位后重新获取 lease，拒绝旧命令。
- 首版每个 episode 单独保存一组原始记录，避免时钟回跳跨越同一条训练时间序列。数据中同时保留消息源时间、记录时间及必要的单调时钟延迟测量；RPC 超时不依赖会暂停或回跳的仿真时钟。
- 模拟时间推进速度可与实际时间不同；先以接近实时的单场景验证，后续再评估加速采集和固定步长推理。
- 元数据保存 `source=simulation`、episode、场景/机器人/转换器版本、相机参数、随机种子、物理参数和示教来源。真实数据与仿真数据通过显式配置组合。

## 7. RTX 4050 6 GB 的启动与资源安排

默认单场景、单 RGB 相机，先采用较低分辨率与可配置帧率；腕部相机、深度图、复杂光照和多环境并行在测量后逐步增加。仿真采集不需要加载模型；仿真推理时渲染和模型会共享 GPU 显存，需要联合测量。

首版提供无 GUI 的 server 模式，相机使用 OGRE2 / EGL 离屏渲染；调试时再开启 GUI。无 GUI 仍然需要相机渲染，GPU 设备可见、EGL/OpenGL 和驱动库都要在容器里验证；仅 CUDA 计算成功不足以证明渲染可用。Gazebo 文档也描述了软件渲染路径，但其性能需另行测试。[无界面渲染](https://gazebosim.org/api/sim/8/headless_rendering.html)

若本地模型与渲染组合超出资源预算，可先降低相机开销或将 `learning` 推理服务迁移到另一台 GPU 机器，保持 ROS 采集和场景配置一致。这里不预先承诺特定帧率或模型的显存占用。

## 8. 目录、启动组合与验收

目录归属遵循 [仓库组织规范](repository-layout.md)：Dockerfile 放 `deploy/docker/backend-sim/`；仿真 API 适配、复位与场景编排放 `components/backend_sim/`；机器人描述包放 `assets/robots/<model_id>/description/`；被动场景放 `assets/scenes/<scene_id>/`；运行配方放 `configs/backends/sim/`。控制器、桥接、相机和 episode 参数按职责配置，机器部署绑定单独放在 deploy。

Compose 提供 backend-real / backend-sim 二选一，再组合 runtime 和按需的 learning。真机和仿真都通过同一 Robot API 接入应用。ROS 容器保持相同的发行版、domain 和可达的 DDS 配置，具体网络模式在首次容器联调时验证。

后续按以下顺序验收：

1. 仿真启动后，双臂、夹爪、相机和 `/clock` 可用；发送命令能看到实际仿真关节与图像变化。
2. 脚本演示经统一 `runtime` 链路记录，MCAP 可读取，包含图像、关节反馈、实际下发动作、TF、时钟与 episode 元数据。
3. 同一个 `learning` 镜像转换这份记录，读取训练样本，完成训练步并保存模型及预处理配置。
4. 模型服务根据仿真观测返回动作，`runtime` 转换并执行，下一帧观测反映动作结果，持续形成闭环。
5. 暂停、复位、切换 episode、模型服务断开或迟到结果均能被正确处理；上个 episode 的动作不会在新场景中执行。
6. 同机运行时测量相机吞吐、GPU 峰值占用和端到端延迟。另用明确任务和示教评估抓取/交接成功率，与接口验收分开记录。

以上为物理仿真的待验收项；阶段 1～2 已验证 mock 控制、采集与记录，不能据此认定 Gazebo 物理与渲染已通过。
