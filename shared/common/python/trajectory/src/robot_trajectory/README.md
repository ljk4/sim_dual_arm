# robot_trajectory

- `spline_math.py`：连续极值与限位验证；算法输入 `ReferenceInputs`。
- `sampling.py`：`sample_reference_plan`（含终点保持）、`sample_reference_interval`（严格时间区间）。
- `follower_math.py`：终端速度估计、约束与连续偏差。
- `follower.py`：算法产物 `PreparedFollowerPlan`、归一化及完整规划验证。
- `follower_reference.py`：`sample_follower_reference`、`applied_follower_sample`、`follower_reference_from_prepared`、独立数值验证 `validate`。

轨迹数据结构与边界常量从 `robot_contracts.trajectory` 导入。
