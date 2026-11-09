# robot_ros_support

- `conversions.py`：基础状态、观测、命令和轨迹的双向转换。
- `stream_conversions.py`、`follower_conversions.py`：流式请求/反馈和跟随参考。
- `qos.py`：相机、状态和命令流的显式 QoS。

转换函数不隐藏字段或补默认语义；不支持的版本/字段应拒绝。

目录导航：[上级目录](../README.md)。
