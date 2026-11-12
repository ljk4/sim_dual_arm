# 阶段 06：可选 SDK 与 Panda Teacher 扩展

状态：待实施。本阶段遵守[总要求](README.md)，只完成本文件范围。

## 前置与读取范围

上一阶段 05 的必要验收通过；读取 `deliveries/phase-05.md` 的接口与限制。

必读总要求、本文件、已有 progress 和上阶段交接；源码按下面入口查找。除明确引用外，不要求阅读其他阶段全文。

## 目标与涉及入口

把固定堆叠 Teacher 从可选通用接口中拆出，并让 SDK 构建与验证按所选包集工作。涉及公共接口 CMake/package.xml、SDK 工具/marker、simulation ROS 服务端、runtime task/Teacher 客户端、campaign 路径和相关测试。

## 专用接口边界

- 复位、通用任务生命周期与不透明任务指标可保留在可选 `sim_api_msgs` 中；它不成为真机控制的强制依赖。
- 固定 60×16、堆叠阶段码和 IK 诊断的 `GetTeacherChunk` 转入 `panda_stack_api_msgs`，保留现有载荷与语义，不顺便设计任意任务 Teacher 平台。
- 更新 simulation ROS 适配、runtime Teacher 客户端及其测试。通用控制和 common 不导入该专用包。
- runtime 中读取专用扩展的客户端适配归 runtime，可按绑定的能力标识选择；不能把 `if scenario_id == "panda_stack"` 塞入公共解析器。
- Proto 注释中“固定三相机、16 关节、60 点”等配方描述移入方案说明。公共字段仍保持明确单位、顺序和时间语义。

保持 Teacher 的 60×16 载荷、阶段码和已有语义；包变为方案目录下 panda_stack_api_msgs。扩展导入只出现在相应端适配中。

更新真实 binding 的 common_ros/extension_ros、服务端和客户端类型、worker 与 campaign.required_paths。保持服务名称时也必须检测消息类型和能力版本不匹配，不能靠同名服务误连。

## SDK 规则

- ROS 生成器支持核心包、选中的可选公共包、选中的方案扩展包。核心 Robot API 不要求安装 sim 或 Panda 扩展。
- 无方案输入的核心构建只构建核心；构建 `panda_stack` 完整功能时由 binding 选择额外包并校验依赖闭包。
- 根据选中的包集校验 SDK 完成标记、生成模块归属与源摘要，不能硬编码所有端都必须存在的包列表。
- completion marker 应明确所选包及其源码来源，防止某次核心构建产物被误当作完整 Panda SDK。
- Policy SDK 仍只从公共 Proto 生成，不依赖 ACT 或 LeRobot。
- 修改 `.msg/.srv/.proto`、包构建元数据或选择的包集必须使旧标记失效；README 变化不应使生成代码被判为过期。
- SDK 产物只写 `.build/` 或显式新输出根；不得修改生成文件来假装已迁移。

不再由通用 SDK 检查导入 dual_arm_runtime 或固定私有源码路径。明确核心包集与完整 Panda 包集的独立输出或受控重建策略，不能污染活动 SDK。

Proto 中配方性注释可归入方案说明，但不改未变化的 wire 字段、编号、单位或 Policy 主版本。

## 验收

- 核心包单独构建/导入，普通控制及 ROS-free 公共包不要求 sim/Panda 包。
- 完整 Panda 包集包含通用、可选及专用接口；Teacher 服务端/客户端类型相同。
- 修改 IDL、包构建元数据或包集使 marker 失效；README 变动不失效；缺模块、旧安装模块或错误来源被拒绝。
- 更新的 campaign、prepare、worker 不再引用已删除服务源；相关协议与 SDK 测试通过。
- 类型迁移与历史读取影响有明确记录；已有历史数据不改写。
- 交接必须给出实际协议/能力版本、包列表、构建命令、marker 格式和兼容性边界，供阶段 07/08 延续。

## 交付与停止点

- 更新 `progress.md`，写 `deliveries/phase-06.md` 简短交接；基线清单只查询/补充相关行。
- 交接列出实际入口/接口、关键决定、命令与结果、未运行项及原因，状态必须真实。
- 完成本阶段后停止，不自动进入下一阶段。必要验证未完成时保留“待验证”，不得写通过。

目录导航：[总要求与阶段索引](README.md)
