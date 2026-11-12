# panda_stack_learning_v1

双 Panda 堆叠学习任务专用的机器人 Profile 变体：

- **继承与差异**：16 维关节顺序、物理单位、位置/速度硬限位以及 `global` / `left_wrist` 相机参数与 [`panda_stack_v1`](../panda_stack_v1/README.md) 完全一致；仅将 `right_wrist` 相机的垂直视场角（FOV）扩大为 **68°**（标定详情记录于 `right_wrist_calibration.json`）。
- **场景绑定**：对应仿真场景配置文件为 [`shared/scenarios/panda_stack/configs/simulation/scene/panda_stack_learning.yaml`](../../../configs/simulation/scene/panda_stack_learning.yaml)。修改关节或公共相机参数时须同步更新两个 Profile 并通过变体一致性单元测试（[`test_learning_camera_variant.py`](../../../../../../tests/unit/test_learning_camera_variant.py)）。

目录导航：[上级目录](../README.md)
