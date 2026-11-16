# P3-G 提示词：实现已冻结的 R2 失败路径替换协议

状态：**暂缓实施。用户最新指令优先数据抽查、物理回放、模型输出和真实执行对照，见任务书§14；本文件保留作已冻结方案，不作为当前任务入口。** 用户已以“按推荐 R2 确认四项”确认语义，P3-F 已完成离线冻结。唯一规范正文为[任务书 §13 R2-F1](2026-11-15-inference-repair-taskbook.md#13-p3-f-已冻结r2-替换契约与离线验证2026-11-15)。本阶段落实生产协议与离线集成测试；新增任务闭环、模型推理、训练、采集预算仍为零。

---

请在 `/home/ljk/work/sim_dual_arm` 亲自实施 P3-G，遵守 `RULES.md`，保留 P1/P2/P3-B/P3-C 及用户既有修改，不执行任何 Git 写操作。四项语义已经明确确认，不再重复询问；不要停在再写一份方案或提示词。先查实际代码、生产者和消费者，然后完成范围内实现及验证。只有遇到必须改变 R2-F1 语义的新证据，才具体说明取舍，不静默扩大范围。

## 1. 开始前固定边界

- 只读记录当前工作树状态，使用 `GIT_OPTIONAL_LOCKS=0`；按相对路径把将修改的文件备份到唯一 `/tmp/sim_dual_arm-repair-P3G-<标识>/original/`，列出新增文件。备份不另造哈希/证明清单，不覆盖用户后续修改。
- 阅读 RULES、任务书 §10.9–§13、现有 stream/control/planner、请求/消息/查询/审计和测试入口。R2-F1 是唯一规范，不再复制可编辑配置或另建通用恢复框架。
- 先跑最小相关基线，记录既有失败；不以 skip、删负例、放宽断言制造通过。
- 仅复用现有镜像，不 build/pull 镜像；生成 ROS SDK 使用项目既有构建入口，在 `/tmp` 编译、`build/` 仅保留规则允许的安装产物。此处的 SDK 重建是接口验证所必需，不能用文本digest一致代替真实生成和两端type support导入。
- 历史 MCAP 的冻结输入与时序见 `p3d-inputs`、`p3e-candidates.json`、`p3f-candidates.json`。新IDL不能直接假定可解码旧CDR；使用已冻结JSON或在SDK更新前提取缺失的最小诊断字段，不改写旧包、不加生产legacy解码分支、不复制MCAP。

## 2. 必须完成的行为

按任务书 §13 落实单一 R2 路径：

1. 只有有效活动 Policy、非最终 STREAM_NEXT、无已接受pending的两类数学失败可进入恢复。runtime使用Policy类入口，backend使用既有当前任务准备事实/运行状态；不添加来源开关，不按机器人/任务名或日志判断。Teacher、first/final、STOPPING、安全/lease/context失败保持原拒绝与停止。
2. 原请求没有被接受。backend规划线程在有界期限内搜索并缓存一个S候选；owner只采样状态、校验与切换，不能在物理tick跑Ruckig扫描。返回保留original_reason的暂定状态，runtime不再立即把它当终止拒绝。
3. runtime仅确认同一token/anchor。保留首次失败提交的完整目标、O、绝对时间、选中索引、k、T、流command_id/command_seq及拟用chunk_seq；不重新推理/融合/准入/夹爪整形，不生成第二份可编辑目标。`selection_origin=原H`、实际handoff=S；普通请求显式origin=handoff。MOTION明确无窗口。
4. owner在确认和S生效前重新验证实时状态、期限、anchor采样与缓存适用条件。own与原曲线均为硬门，所有原限位、速度/加速度、3 s、制动、Follower/P2条件保持；不以缓存或必要包络代替真实守卫。每原失败最多一次逻辑替换，传输幂等重发不算新尝试。
5. S tick原子撤销旧尾部并启用新参考，安全/任务/lease停止先发生或同tick发生时优先。不能复活停止流、回填错过的S、修改旧FollowerReference或重置任务证明。
6. 扩展现有查询以确认OFFERED/SCHEDULED/EFFECTIVE/ABANDONED。同token/完整内容在排期和生效后均幂等；ACK或通知漏收不能造成重复安装或猜测旧轨迹。原失败请求在排期/生效后重发先查恢复记录，不能误报普通payload冲突。不同内容/旧context/token拒绝，不覆盖合法pending。
7. 恢复失败按原停止/制动路径收束，保留原reason与放弃原因，不递归发offer。后继只绑定新参考的T/q/v与n+1；确认已生效后再取新观测，仍验证普通选窗、60点horizon、freshness、提前量和全部deadline。不得仅因T相同宣称节奏自动不变。

搜索使用真实时钟，计算前后都检查截止；20 ms粗扫与实际物理网格细化只定义一处，网格/min_lead/max_age/预算仍读取既有SSOT。粗扫未命中且预算足够时覆盖剩余细网格；预算耗尽就停止。不得照搬离线拟合时钟或把诊断用50 ms确认预留当已测量网络保证，也不新增无依据的150 ms/35次/5%阈值。

## 3. 一次同步所有生产者与消费者

根据实际代码落实任务书 §13.3 的类型和字段，直接演进当前协议，不留旧兼容字段/别名/shim：

- 共享 `StreamRequest` / `ActionWindow`、验证、ROS StreamCommand/SubmitFollower/QueryStreamCommand及转换；
- backend planner、worker owner、control调度/命令历史/查询/事件；缓存与历史依现有有界保留方式，不能新建永久回执库；
- runtime控制客户端、Policy runner的请求确认/未知结果路径、普通/Teacher编码和错误传播；
- FollowerReference请求身份/选窗/既有digest的真实消费者，不添加新哈希平台；
- 录制、离线follower审计/闭环汇总：重建撤销后的唯一参考时间线，正常撤销不能计为任务失败；接受/排期/生效/任务成功分开；
- 当前生成SDK与所有受影响测试。learning wire/model/数据/Bundle不变。

可选数据按冻结规范使用零或一个元素的有界序列，避免新增has与payload的重复状态。可从token绑定、anchor和请求推导的S/seq/T/source不重复维护。通用层不写具体任务名，不加纯转发包装、默认feature flag或通用工作流引擎。

## 4. 验证与验收

将 P3-F diagnostics 规格转成真正生产路径上的回归；离线模型代码不得被生产导入，模型绿测不能冒充集成通过。

- **红/绿**：现有selection_origin缺失红测先保留；新增token绑定的提前替换红测覆盖真实planner/owner入口。普通boundary仍拒绝未授权的提前切换，不能通过删除INVALID_CHUNK_START获得绿测。
- **数学重放**：冻结六条20点原拒绝逐字符一致；同一活动参考下5点仍为5拒绝/1接受。11个候选在明确的可生效测试时钟内通过生产双门/C1/连续限位/真实制动/3 s；太晚或无解如实失败，不要求真实墙钟运行强凑11/11。5点反事实不是从头默认5点闭环。
- **目标与时序**：完整60点、k、原绝对目标、T与原indices逐值不变；重试不重跑P3-C。下一anchor确实来自新参考T/q/v，下一序号/观测计数只推进一次，覆盖horizon不足/观测过期/晚到。
- **未知结果与幂等**：OFFERED和SCHEDULED的响应丢失、S事件丢失、原请求排期/生效后重发、相同token重复确认均只切换一次；同token不同内容、错anchor/context、已有pending、过期和旧anchor后继均不覆盖合法计划。
- **停止优先**：S前/同tick的任务终止、安全、lease过期、context复位、流STOPPING均不复活；生效后的停止仍保留历史生效事实；任务证明不清空。
- **隔离和证据**：真实Teacher runner/source输入保持原目标，backend拒绝不吞掉；原失败/撤销/排期不能冒称任务成功。新录制审计在缺少事件通知但可查询确认时重建唯一参考，有歧义明确报错。
- **跨端协议**：新字段往返、optional空/单元素语义、两端生成类型/type support、普通请求/Teacher/MOTION回归；重建SDK后分别在runtime/simulation环境实际导入并执行相关测试。
- **相关回归**：按变更范围运行contracts、runtime stream/client、simulation follower/control/schedule和P2/P3-C关键回归；基线失败单列。无变化且已通过的范围不无故重复扩测。

不运行新任务闭环、批量推理、模型预测、训练或采集，不临时改target_count、融合、Profile、摩擦/servo/IK或阈值。若为单元/集成验证使用真实MuJoCo最小模型，明确它不等于完整任务闭环。时限导致部分离线请求不可排期是合法结果，报告具体失败，不能暂停物理时钟来凑接受。

## 5. 完成交付

- 在原任务书追加P3-G实际实现、字段生产者/消费者、红绿结果、相关回归、耗时和失败边界；同步既有文档索引，不再复制规范。
- 明确区分生产协议通过、离线数学通过、未验证物理跟踪/碰撞/接触和任务闭环。没有新闭环成功证据时，P3总体仍未通过；不声称提高成功率。
- 给出确切修改/新增文件、逐文件复原路径和SDK重建/复原条件，恢复env.sh临时改动、临时配置、容器原状态及/tmp测试视图。
- 不遗留无消费者的临时复制、哈希清单、测试视图或大中间产物；保留仍用于复核的失败证据。
- 在最终回复直接给下一步及可用提示词：只有生产协议与离线集成通过，才提出固定Bundle/默认配置的小批物理验证方案；**本提示词不释放该闭环预算**，先给出可审查的具体seed、布局、组别、次数与归因规则。
