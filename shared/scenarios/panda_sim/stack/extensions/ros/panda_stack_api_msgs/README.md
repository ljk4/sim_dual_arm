# panda_stack_api_msgs

`panda_stack` 方案专用 ROS 接口包，不属于公共 Robot API，也不由普通控制加载。

- [`msg/TeacherCapability.msg`](msg/README.md)：Teacher 能力/版本标识。
- [`srv/GetTeacherChunk.srv`](srv/README.md)：固定 60×16 堆叠 Teacher 动作块。
- 服务名保持 `<namespace>/get_task_chunk`；客户端必须同时匹配生成类型和
  `TeacherCapability.ID`，不能仅凭同名服务连接。

本包只由 `simulation` 的 Teacher 服务端适配和 `runtime` 的 Teacher 客户端适配
延迟导入。`robot_contracts`、`robot_ros_support`、普通控制客户端和
`shared/common` 的构建/检查工具都不导入它。

目录导航：[panda_stack 方案](../../../README.md) · [公共 ROS 接口](../../../../../common/interfaces/ros/README.md)
