# 真机端边界

真机实现尚未开始，本目录只固定与仿真共用的机器人边界，不提供驱动、控制器或
可运行入口。

## 统一 Robot API

真机与仿真后端必须实现同一套公共边界：

- 接口与消息：`contracts/ros/robot_api_msgs/`；Policy 协议在 `contracts/policy/`。
- 语义与限位：`profiles/robots/`、`profiles/observations/`、`profiles/actions/`
  及对应 `contracts/schemas/`。
- 运行侧只通过 Robot API 或经校验的产物访问机器人，不导入具体驱动或物理实现。

驱动的设备地址、序列号、账号和本机标定属于忽略的本地绑定，不写入仓库。

## 当前状态

- 真机驱动、相机采集、安全限位和硬件在环测试均未实现。
- 不创建假驱动、占位节点、动态插件框架或“未来会自动补齐”的空模块。
- 新增实现时先确认具体硬件、驱动来源、许可和可验证的安全边界，再在本目录建立
  有实际文件与责任的最小实现。

仿真端的当前实现见 [simulation/mujoco](../simulation/mujoco/README.md)，运行端见
[runtime/ros2](../runtime/ros2/README.md)。

目录导航：[上级目录](../README.md)。
