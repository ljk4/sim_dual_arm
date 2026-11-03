# mock_robot_backend 包

`node.py` 实现测试用 ROS 2 节点：发布状态/相机、接受控制权与轨迹命令，并返回
确定性反馈。`__init__.py` 只做包标记。该包属于 test doubles，不安装到正式运行
组合；测试入口见 [上级说明](../README.md)。
