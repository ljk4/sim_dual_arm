# robot_backend_sim

MuJoCo 双 Panda 后端按职责拆分。顶层只保留包标记与导航，不提供旧模块路径兼容导出。

- **[`config/`](config/README.md)**：声明式配置及校验。
- **[`control/`](control/README.md)**：控制会话、历史、制动和反馈邮箱。
- **[`follower/`](follower/README.md)**：受限跟随规划及协议。
- **[`kinematics/`](kinematics/README.md)**：逆运动学和间隙检查。
- **[`motion/`](motion/README.md)**：运动执行、状态转换、Ruckig 导出和流式接续。
- **[`physics/`](physics/README.md)**：场景、资产、接触与 Panda I/O。
- **[`render/`](render/README.md)**：相机渲染、可见性与 viewer。
- **[`ros/`](ros/README.md)**：ROS 节点、服务适配与 CLI。
- **[`runtime/`](runtime/README.md)**：物理线程所有权、跨线程协议与诊断。
- **[`tasks/`](tasks/README.md)**：任务插件与独立 ROS 转换器。
- **[`tools/`](tools/README.md)**：场景和环境离线检查。

按原 import 图保留现有组合关系；`motion/planner.py` 仍协调任务规划线程，`physics/gripper_contact.py` 仍与运动类型协作，本轮未重写控制生命周期。
