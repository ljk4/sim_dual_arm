# 轨迹算法共享库

`dual-arm-trajectory` 的源码包为 [`robot_trajectory`](src/robot_trajectory/README.md)。仅依赖 `dual-arm-robot-contracts`，不依赖 ROS、MuJoCo、NumPy 或 PyTorch。simulation 与 runtime 通过 Compose 的源码路径加载；learning 的生产路径无需加载本库。

`ReferencePlan`、`FollowerReference` 等跨端记录仍由 `robot_contracts.trajectory` 定义。本库接收这些数据，负责求值、连续极值、跟随计划验证和物理应用参考重建。Ruckig 求解器仍属于 simulation。

源码安装：先安装 `shared/common/python/contracts`，再 `pip install ./shared/common/python/trajectory`。契约记录见 [`robot_contracts.trajectory`](../contracts/src/robot_contracts/trajectory/README.md)，本库的求值与验证入口见源码包 README。
