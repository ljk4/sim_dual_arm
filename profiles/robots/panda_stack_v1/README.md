# panda_stack_v1

双 Panda 机械臂基线 Profile（`schema_version=2`）：

- **自由度与顺序（16 维）**：左臂 7 旋转关节（`rad`）、左夹爪总开度（`m`）、右臂 7 旋转关节（`rad`）、右夹爪总开度（`m`）。
- **相机配置**：固定包含 `global`、`left_wrist`、`right_wrist` 三路 320×240 RGB 相机，标定摘要由场景几何声明计算并写入 `profile.json`。

目录导航：[上级目录](../README.md)
