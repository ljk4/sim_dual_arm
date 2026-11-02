# DSH-025 结果：原生控制会话

2026-11-01，9个允许文件已验收整合。执行依据为[任务书](dsh-025-native-control.md)，
精确接口见[原生控制会话](../native-control.md)。本轮仍采用dsh实施、监督者独立验收模式。

新增纯标准库ControlCommand与共享generation比较；NativeController组合既有NativeMotion，
提供租约、整体命令/块幂等、有界查询、真实停止与episode边界reset。一个连续流只有一个
整体命令，块退休不等于整段完成。期限使用显式墙钟，参考与测量使用仿真时间。

首版679 unit通过，固定原生87通过/2项新测试设置失败；独立扩展12组中10组通过。
确认错误续租会吞掉已产生的停止事件，过期envelope也会掩盖原生迟到刹车。唯一返工
增加有界DeferredEvents并保留原事件身份，修正合法迟到路径及两个测试夹具。
返工后685 unit/91 native通过，独立13组中12组通过；新增跨episode检查发现新租约未重置
命令序号，与现有runtime每个episode从1开始不兼容。监督者局部接管修复并保留回归，
旧终态历史仍可按原context查询，复用旧ID仍拒绝。

| 最终检查 | 结果 |
| --- | --- |
| 冻结源码unit / Ruff | 685通过，lint与变更Python格式通过 |
| 固定MuJoCo原生回归 | 92通过、0 skip，含原81与新增11 |
| 旧Jazzy/mock/MCAP回归 | 115通过、0 skip |
| 独立真实行为 | 13/13：raw ctrl重放、租约刹车、停钟、幂等/冲突/淘汰、reset、14入口线程归属、deadline与事件完整性 |
| 日志 | 47条事件逐条schema通过、序号连续、零丢弃、关闭complete |
| 源码 | 主目录118个Python文件与冻结候选一致，其中64个业务文件 |

证据根为`var/runs/dsh/dsh-025/20261101T084852Z/`，最终候选`source-final`，
测试在`unit-final.xml`、`native-final`、`ros-final`与`independent-final`；
`accepted.patch`为整合补丁，初版/返工/接管源码和原失败输出保留。
输入快照`b4d7bee5b63a4848bf9e6a026f426611247a4b36`，session
`session-888b423d-b0f6-4b5c-ab2c-318cd88cc751`。

固定原生镜像仍为`sha256:4422f876ae463391f520aacfa3966d8a9c4d9d15a40c7a571cc453133e81b10a`；
ROS IDL未改，SDK摘要仍为`b5a26e7935020a4d3d00a1633dc0f150eedb4ad7f03af7df9ec8cd1f32493e94`。
未修改模型/增益/真实测量来通过，没有新建日志writer。公共ROS/精确观测/Policy尚未接线，
S3整体未验收，正式数据0，ACT未开始。

dsh首轮1080.906s、返工362.542s，共133步/155次工具调用；input167,481、output226,133、
cacheRead32,160,128、total32,553,742 tokens，计费未知，不推算节省比例。headless已退出。
未提交Git，主HEAD/暂存区不变；按用户持续推进指令直接进入后续阶段。
