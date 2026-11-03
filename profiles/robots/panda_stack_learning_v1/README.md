# panda_stack_learning_v1

学习任务相机标定变体。关节顺序、单位、硬限位和其他相机继承 `panda_stack_v1`；
右腕 FOV 为 68°，声明在 `right_wrist_calibration.json`，摘要由
`camera_declared_description`/`semantic_hash` 生成。

对应场景 `configs/simulation/panda_stack_learning.yaml` 只改右腕视场，机械几何和
物理与 `panda_stack.yaml` 一致。相机变化会改变 profile digest；旧数据保留其
manifest 内 profile 快照，禁止混用；关节或其他相机变动需同步两个 profile 并通过
变体一致性测试。

目录导航：[上级目录](../README.md)。
