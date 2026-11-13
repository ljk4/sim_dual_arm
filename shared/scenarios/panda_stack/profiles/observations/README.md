# 观测绑定（Observations）

- **`panda_rgb_joint_v1.json`**：双 Panda 16 维实测关节/夹爪位置 + 三路具名相机（`global`、`left_wrist`、`right_wrist`）RGB 观测定义。

相机名称、分辨率与关节顺序必须与所绑定的 Robot Profile 完全一致。

`max_skew_ns` / `max_age_ns` 由本档案唯一定义。`dual_arm_runtime.runtime_config` 加载运行配方时会把这两个值作为默认值注入 `observation` 块；`configs/runtime/*.yaml` 不再重复声明它们，只有需要有意覆盖时才显式写出同名键。修改本档案会直接影响运行时观测时效校验。

目录导航：[上级目录](../README.md)
