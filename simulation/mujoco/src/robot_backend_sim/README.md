# robot_backend_sim

MuJoCo 双臂后端按职责拆分，同时服务 Marvin 堆叠（`marvin_sim/stack`）与 Marvin 交接（`marvin_sim/handover`）。顶层只保留包标记与导航，不提供旧模块路径兼容导出。

- **[`common/`](common/)**：机器人/任务无关的通用机制。
  - `config/`：声明式配置及校验。
  - `control/`：控制会话、历史、制动和反馈邮箱。
  - `follower/`：受限跟随规划及协议。
  - `kinematics/`：通用位姿/路径采样与间隙检查。
  - `motion/`：运动执行、状态转换、Ruckig 导出和流式接续。
  - `physics/`：场景容器、资产、接触与通用机器人 I/O。
  - `render/`：相机渲染、通用可见性计数与 viewer。
  - `ros/`：ROS 节点、服务适配与 CLI。
  - `runtime/`：物理线程所有权、跨线程协议与通用诊断。
  - `tasks/`：任务插件协议与独立 ROS 转换器。
- **[`marvin_sim/`](marvin_sim/)**：Marvin 机器人装配（`robot_config.py`、`assembly.py`、`kinematics.py`、`io.py`、`scene_base.py`、`plugin.py`）、`stack/`（堆叠场景、可见性、诊断及任务插件）与 `handover/`（交接场景及任务插件）。
- **[`tools/`](tools/README.md)**：场景和环境离线检查。

按原 import 图保留现有组合关系；`common/motion/planner.py` 仍协调任务规划线程，`common/physics/gripper_contact.py` 仍与运动类型协作，本轮未重写控制生命周期。
