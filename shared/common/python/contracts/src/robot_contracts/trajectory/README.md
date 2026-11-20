# 轨迹契约数据

- `constants.py`：维数、时间边界与共同数值容差。
- `spline.py`：`CubicSegment`、`ReferenceSample`、`ReferencePlan`、`JointExtremes`、`ReferenceExtremes`。
- `follower.py`：`FollowerLimitFlags`、`NormalizedFollowerPositions`。
- `follower_reference.py`：`FollowerReference`、`FollowerAppliedSample`、`FollowerValidationLimits`、身份/摘要校验。

保留结构与边界校验；曲线求值和规划算法由 `robot_trajectory` 自由函数实现。数据对象不含采样方法，也不反向导入算法库。
