# DSH-026 结果：公共参考与真实反馈协议

2026-11-01，28个允许文件已验收整合。对应[任务书](dsh-026-native-protocol.md)，
接口与字段约定见[原生协议](../native-protocol.md)。本项提供传输契约及转换，尚未启动ROS服务。

共享DTO保存完整原请求、显式分段系数、来源观测/选中窗口、接触证据和真实反馈；
原生侧只负责请求预编译及结果转换，ROS侧显式编码/解码，二者通过纯标准库契约连接。
新增9个msg和5个srv，旧17个msg/srv/action文件字节不变，旧linear_v1路径保持。
新参考使用explicit_cubic_v1，不冒充旧线性插值反馈。

首版697 unit通过；真实SDK构建后原生/序列化组合98通过、3项测试设置失败。独立检查
确认MOTION收尾保持重建被误判过期、ACCEPTED阶段误称已应用轨迹。唯一返工修正这两处，
并调整测试的合法时长、wire/list断言。非final连续块仍严格过期，不以零速保持救场。
返工后99通过、2项测试夹具失败；监督者接管末端HOLD类型、零数组断言形状，复用此前
已验收的运行前自由块稳定夹持夹具。没有改控制参考、物理限位、模型增益或执行中物体位姿。

| 最终检查 | 结果 |
| --- | --- |
| 冻结源码unit / Ruff | 697通过，lint/变更Python格式通过 |
| 固定MuJoCo + 真实ROS序列化 | 101通过、0 skip |
| 新SDK下旧ROS/mock/MCAP | 115通过、0 skip |
| 独立物理与CDR检查 | 6/6；普通运动与闭式三次曲线最大误差约3.47e-17，三块记录重建实际参考误差0 |
| 篡改拒绝 | 系数、deadline/digest、测量误差不变量、started flag均拒绝 |
| 真实参考类型 | 启动前HOLD、有效区间TRAJECTORY/CHUNK、明确末端HOLD；未应用/已应用brake的停止超时分别保留原类型/BRAKE |
| 依赖隔离 | ROS镜像阻断NumPy/SciPy/MuJoCo/ROS/后端导入，共享DTO与参考验证通过 |
| 日志与源码 | 6条检查事件schema/关闭通过；127个Python文件与冻结源码一致，其中69个业务文件 |

真实闭合接触例保留目标0、实测约0.04m、双指同物体力/计数/速度证据并完成CDR往返。
初始夹持只用于协议诊断，不计teacher或正式数据。整体终态冻结，后续实时物理不改写它。

证据根`var/runs/dsh/dsh-026/20261101T093023Z/`，包含first/rework/primary归档、
`source-final`、`accepted.patch`、`native-final`、`ros-final`、`independent-final`及`stdlib-first`。
输入`d0813abbdb6426eae3cac7bf7d2a4e8afb11b947`，session
`session-455a38ff-b181-434e-9fd2-55eb96f032e6`。候选与默认SDK均已真实构建，33个源文件摘要为
`0889138136a082d1b383815b5a877179a4de1e496c04e67428628ce8928e4435`。
构建禁网、源码只读挂载，未重建依赖镜像；原生镜像ID保持既有4422f876版本。

dsh首轮637.754s、返工116.871s，共104步/120次工具调用；input135,011、output125,064、
cacheRead17,289,216、total17,549,291 tokens，费用未知。headless已退出，未提交Git，
主HEAD/暂存区不变。继续进入ROS actor/渲染与精确观测接线；S3整体仍未完成，正式数据0。
