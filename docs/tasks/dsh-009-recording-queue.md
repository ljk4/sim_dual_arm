# DSH-009：S1c 有界 MCAP 写入与流式审计

前置：DSH-008 已验收。本任务解决 R5：磁盘写入不阻塞 ROS 回调，审计不保留整段图像，文件哈希不整文件读取。保持现有 mock/v1 manifest 的解释；结果分类迁移和人工操作另开下一任务。

## 范围

允许修改 runtime 的 `recording.py`、`audit.py`、`configuration.py`、`episode.py`、`node.py`（后两者仅记录健康检查/收尾接线）；可新增一个 `recording_queue.py`，隔离纯Python队列与ROS原生writer，禁止新增通用框架。允许修改 `contracts/schemas/v1/runtime_config.schema.json`、`configs/runtime/mock_collect.yaml`；新增 `tests/unit/test_recording_queue.py`、`tests/integration/test_recording_backpressure.py`，扩展现有 `test_recording.py` 与测试侧bootstrap；旧生命周期fixtures只做必要接口迁移，不删除原断言。文档限定 `docs/logging-and-recording.md`、`docs/stage2-quickstart.md`。

不修改公共日志库、Docker、其他协议、manifest schema、仿真或学习；不提交Git、不启动其他代理、不申请Docker。阅读当前相关模块与已有记录/中断测试即可，不全仓扫描。保持公开用途少且明确，避免把一次性记录器写成通用存储插件系统。

## 配置和数据所有权

- frozen `RecordingSettings` 放纯Python可导入位置；YAML可选 `recording` 默认 queue_capacity=1024、max_pending_bytes=67108864、startup_timeout_s=5.0、close_timeout_s=5.0。示例显式列值，快照补齐默认值；正整数/有限正数/禁止bool冒充数值，路径明确。直接构造 EpisodeRecorder 的旧调用保持可用，episode显式传入配置值。
- 每个 recorder 一个后台writer独占 rosbag2_py.SequentialWriter 的创建/open/create_topic/write/close。业务线程不能跨线程关闭原生句柄；不在持状态锁时执行磁盘I/O、join、回调或logger操作。线程名清楚，异常保存为不可清除的首要故障。
- `append` 在生产线程序列化为不可变 bytes，使入队时的值不受后续消息修改影响；序列化在队列状态锁之外，不等待磁盘或队列空位。这里“非阻塞”指不等待磁盘/容量，不能声称序列化没有CPU成本。所有权、时序在短注释中说明。
- 队列按条数和已序列化字节双重限制。预算包含正在write但尚未释放的条目，不能 dequeue 后就把慢盘持有的大帧从预算消失。单条超过预算、队列满、序列化/写入异常均明确失败，不能丢帧返回成功。只接受已注册topic、合法时间及payload；没有生产测试延迟开关。
- 关闭与最终入队使用明确的短临界区，关闭后append拒绝。并发关闭/卡住生产者不能绕过close预算。finish后没有late append被静默丢弃：episode停止生产记录、与在途append协调后再detach/finish。可以持episode recorder锁执行已确认不做I/O的append，但不能持锁join/finish或通知其他锁。

## 生命周期与故障

1. 初始化等待writer就绪有明确startup预算；失败不会启动控制。异步故障可通过 `check_health()` 立即观察，不能只在后续append或finish时检查。
2. ROS定时路径（即使没有新帧）及发送下一条命令之前检查当前recorder健康。故障进入 EpisodeRunner 的失败/停止链路，按已有独立步骤停止、释放，日志保留原因。健康检查不做文件I/O。避免同一个故障每个控制周期重复追加成无限长错误串。
3. `finish` 首先停止入队并在close预算内等待排空和原生writer关闭；失败/超时永不变成成功，不由调用线程强关writer。不修改main的worker超时124、SIGINT130/SIGTERM143及R1～R3清理语义。
4. 排空成功后，既有 episode worker 执行流式audit/hash/manifest提交；后台writer **不负责发布成功manifest或重命名episode目录**，避免关闭超时后的晚到writer提交成功。排空失败允许保留可识别partial并抛明确记录错误；没有完整manifest时也不能标complete或改名正式episode。迟到writer最多关闭bag，不得自行发布成功。
5. 本任务明确两个时间边界：close_timeout约束队列/原生writer关闭；随后离线audit/hash仍在episode worker中，其异常和信号收尾继续受现有主线程shutdown grace保护。不把close预算宣传成整个离线审计的硬期限。后续运行监督入口统一处理整个episode墙钟超时。
6. 记录数量必须是实际写入数量，不能把仅入队的数据算已落盘。提供少量只读队列统计（已接受/已写入、当前/峰值条数与字节、失败原因）便于测试和诊断；正常关闭后计数对齐、待处理字节归零。字段有类型，不暴露可变内部容器。由episode最终记录事件附加有用的标量统计即可，不高频打印队列状态。

## 流式审计

- rosbag reader一次处理一条消息，校验后只保留引用检查必要的元数据：state/frame时间与ID、observation引用、命令与事件的紧凑语义。不要保留Image/CameraFrame/RobotState对象或像素数组。关节运动范围用在线min/max；不保留所有positions再transpose。
- 不依赖MCAP到达顺序恰好与跨topic引用顺序一致，引用可在末尾核验。保留现有context、顺序、因果对齐、轨迹一致、事件连续、消息计数与损坏检测。参考元数据仍随消息数增长，文档如实说明；目标是去除与像素总量/整文件大小成比例的内存占用，不引入SQLite或新数据库。
- 文件SHA-256通过固定大小分块（例如1MiB）读取，writer提交和inspect复用同一小函数。保留路径边界与checksum检查。不得为通过测试放宽缺帧、错误轨迹、上下文或损坏文件检测。
- 暂时保留v1的mock必须运动/命令正常完成规则，以及source=mock/test_only=true；下一任务明确迁移，不在本轮悄悄改变旧数据解释。

## 验收

纯队列若拆为小模块，可用测试侧writer替身运行普通/多生产者顺序、按条数/字节溢出、持有的在写payload仍占预算、writer失败、关闭超时/重复失败、在途append与close、不可变载荷等行为测试。测试必须释放自己制造的阻塞，不留下线程。不要访问大量私有字段来镜像实现。

ROS测试使用真实MCAP：正常数据重开审计、修改原消息不改变已接受数据、慢写盘时append及时返回且控制续租仍工作、异步失败没有下一条轨迹且stop/release仍尝试、关闭超时保留partial、原中断回归。故障注入只在tests。

审计内存验证使用较大的真实图像记录或受控reader探针，说明数据量/测量方法/峰值，证明不会持有整段像素；不要拿“测试没OOM”作证据。监督者会独立运行原生writer故障与内存探针。不能为制造负载使用不受限的循环或无限文件。

运行全量纯Python回归与Ruff；ROS由监督者运行。报告写 `.build/reports/dsh-009/report.md`，明确已执行/未验证及API边界。首轮上限25分钟，一次定点返工最多10分钟。交付前同时检查新增unit/integration文件不会同名，并使用正确的相邻配对检查。
