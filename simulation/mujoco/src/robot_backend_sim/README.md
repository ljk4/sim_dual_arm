# robot_backend_sim

MuJoCo 双臂后端按职责拆分，同时服务 `marvin_sim/stack` 与 `marvin_sim/handover`。顶层只保留包标记与导航。

- **[`common/`](common/)**：100% 机器人与任务无关的通用机制。
  - `config/`：声明式配置及校验、`scene_loader.py` 场景加载分发。
  - `control/`：控制会话、历史、制动（复用 `robot_trajectory`）和反馈邮箱。
  - `follower/`：受限跟随准入及参考反馈。
  - `kinematics/`：通用位姿/路径采样与间隙检查。
  - `motion/`：运动执行、状态转换、流式接续与任务规划线程。
  - `physics/`：场景容器、资产、接触与通用机器人适配器/I/O。
  - `render/`：相机渲染、通用可见性计数与只读 viewer。
  - `ros/`：ROS 节点、服务适配、`marvin_teacher_api_msgs` 等动态扩展加载与 CLI（支持 `--namespace`、`--no-visualization` 与 `ready.json` 就绪标记）。
  - `runtime/`：物理线程所有权、跨线程协议与通用诊断。
  - [`tasks/`](common/tasks/README.md)：基于 `(assembly, task_type)` 双键注册的任务插件协议与通用转换器。
- **`marvin_sim/`**：Marvin 机器人级实现（`robot_config.py`、`assembly.py`、`kinematics.py`、`io.py`、`scene_base.py`、`plugin.py`、`checks/`），以及平级对称的子任务目录 `stack/`（含 `checks/` 场景与运动学自检）与 `handover/`（含 `checks/` 路径自检）。
- **[`tools/`](tools/README.md)**：保持方案无关的容器环境离线自检（`probe_environment.py`）。

`common/motion/planner.py` 协调任务规划线程，`common/physics/gripper_contact.py` 与运动类型协作完成接触卸载追踪。
