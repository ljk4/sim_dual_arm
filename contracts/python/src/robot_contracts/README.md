# robot_contracts

- `models.py`、`profiles.py`、`episodes.py`、`action_chunks.py`：profile、观测、
  episode manifest、动作块与语义哈希。
- `control.py`、`status.py`、`native.py`、`native_reference.py`、
  `native_feedback.py`：命令状态、原生请求/参考/反馈和校验。
- `reference.py`：不可变分段三次参考、采样、连续极值和约束验证。
- `follower.py`、`follower_math.py`、`follower_reference.py`、
  `follower_reference_digest.py`：受限跟随计划、端速/偏差计算和参考摘要。

共享代码只用标准库（`models`/`profiles` 另用 PyYAML/jsonschema）；仿真侧 SciPy
编译和 Ruckig 计算不进入本包。

目录导航：[上级目录](../README.md)。
