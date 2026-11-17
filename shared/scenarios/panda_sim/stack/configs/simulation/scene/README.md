# 场景与任务配置（Scene）

- **`panda_stack.yaml`**：双 Panda 堆叠基准物理场景（立柱、双臂基座安装位姿、桌面、大小方块尺寸/质量/摩擦系数及三路相机外参）。
- **`panda_stack_learning.yaml`**：学习任务场景配置（物理几何与 `panda_stack.yaml` 完全一致，右腕相机垂直视场角设为 68°，与 `panda_stack_learning_v1` Profile 绑定）。
- **`panda_stack_task.yaml`**：堆叠任务的方块初始采样范围、成功判定几何/速度/持续时间（2.0 s）门限及程序示教关键路点参数。

修改相机参数时必须同步更新对应的 Robot Profile 并通过相机变体一致性单元测试。

目录导航：[方案说明](../../../README.md)
