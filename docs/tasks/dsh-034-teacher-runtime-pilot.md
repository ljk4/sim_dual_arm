# DSH-034：示教运行时、任务收尾与有预算的试采入口

033已验收整合（docs/tasks/dsh-033-result.md）。首轮45分钟、最多一次集中返工。
DSH实现/组件测试，不自行启动完整抓放；监督者验收后执行真正pilot，每次先记账。

## 运行时职责

teacher为新的显式action_source，与policy和旧diagnostic_motion互斥。
复用NativeControlClient、ExactCaptureBuffer、现有writer/logging和ContinuousStream执行核心，
避免复制PolicyStreamRunner300行；可提取一个小typed target-source边界，不动态加载类。
teacher适配器读033服务，策略适配器仍用真实gRPC；两者均60点、16SI、50ms、执行5点，
相同follower/ACK/anchor与原始预测保留；teacher不在runtime导入MuJoCo/mink。
不把teacher阶段、物体真值或评估结果放入Policy Observation/训练feature；task_text固定。

先完成reset/静止/有界Prepare，只读IK不移动机器人；Start明确本episode30sim s起点。
Start后初次预测等待也计时。准备/启动幂等、旧epoch拒绝、关闭有界。
继承原始6关键姿态/阶段目标，程序控制可读真值，ACT部署禁用teacher能力。

统一stream配置支持finite_chunks（现8块诊断）和task（成功/超时终止），保留旧默认。
最大128块配置需覆盖30s，内存仍有界。Raw完整60点可能覆盖未来，但仅实际执行区间作标签。
任务成功/超时/失败触发同一真实Stop和终态事实等待，不伪造最后块COMPLETED。
可将正常托管任务结束表示runtime.stopped，reason=TASK_SUCCEEDED/TASK_TIMEOUT；
task结果独立succeeded/timed_out/failed，实际NativeExecution CANCELED如实保存。
现有v4诊断completed仍严格要求全部8块/1final，不能为了task路径放宽旧诊断审计。
按source/termination_mode进行小范围版本分派，不复制v3/v4完整审计。

30s最大任务与刹车/实测稳定窗口协调，安全停止余量配置化（候选1s，由maxv/a与0.2s稳定解释）；
不能等30s后再执行长轨迹，不能暂停物理掩盖慢规划。Task判定稳定2s与总时钟真实记录。
教师阶段门控未通过则停并保留理由，不用无限保持或调整成功阈值通过。

## 记录与任务质量

保存033 typed TaskState（context/采样sim时钟）到独立MCAP通道，另有小型task_result.json。
任务状态可以由评估/teacher读取，Policy特征构造只允许三RGB+16测量。
Recorder.finish可增加可选TaskResult，默认仍not_evaluated，保持mock/v1-v3不变。
manifest任务结果要引用真正物理judge，不能由脚本phase结束、runtime退出0或source高度代替。
任务事实、动作来源、teacher/ACT、test_only区分明确，失败/超时也可完整录制。

采集模式可启用渲染线程在同一snapshot上统计source/target/table分割像素和出画时长；
不在physics线程渲染，不往普通JSON日志塞mask/图像，不把分割/真值传入策略。
至少：全局初始覆盖两块及目标区；右腕接近看源、放置看目标；左腕稳定有效桌面视野。
接触遮挡允许短时，沿用已验收phase阈值并显式配置max blackout/合格比例，报告所有phase。
保存接近/抓取/搬运/释放/退开三视角PNG及短视频；监督者实际查看目标位置/遮挡再锁定质量规则。
模型输入始终原始RGB，不加真值框/标签；预览可以单独有信息但不能回流原始数据。

实际标签是observation t对应applied reference(t+50ms)，完整raw预测仍另存。
读实际STARTED/切换/取消时刻：未执行pending、停止后旧预测和BRAKE后被替代区间不能冒充标签。
闭合参考0与测量约0.04保留差异。帧间隔不能把100ms伪装50ms。
先提供可审计轻量ROS导出（PNG/JSON/有界数组，无pickle），后续learning转换不导入ROS。
至少一个完整label重放计入pilot，从同一follower检查二次跟随/时间对齐；不能另开不记账的任务rollout。

## Campaign与额度

新增独立小配置configs/experiments/stack_campaign.yaml，候选seed：
pilot1000..1009，formal10000..10099，test20000..20019。
pilot先最多使用1000..1007（teacher/标签重放），1008..1009预留ACT推理试跑；合计≤10。
正式最多100尝试、取前50合格；成功序号[9,19,29,39,49]验证，其余45训练。
原始累计≤30GiB（含失败），开始前估算单次最大量并留收尾空间，运行中持续检查与磁盘余量。

每次完整尝试启动前锁住并原子写attempt ledger：purpose/seed/run/episode/config/code/预算；
RESERVED/RUNNING失败或中断仍计数，不能崩溃后重置额度或静默跳seed。
产物路径和状态可恢复；失败原因、墙钟/sim时长、原始字节、任务/运行时/完整性/视野分别保存。
无并行批量采集；全局配置/SDK/日志不可用应立即停campaign，不消耗后续所有种子重试。
DSH交付不运行pilot。监督者明确启动有预算命令后，所有完整抓放都进入台账；不能称“调试”绕过额度。
本项正式采集入口可准备但默认仅pilot，不会在验收前自动跑100次或启动训练。

## 独立验收

真实ROS teacher请求→同一follower→物理→任务事实→MCAP→audit；正常/迟到/中断/预算拒绝。
先确认scene/IK/控制各门槛，再从pilot台账执行，失败最多按剩余额度修复。
本阶段至少取得可重复的真实堆叠与三视角质量证据、一次真实label重放，才进入正式50采集。
不同seed和label重放均消耗相应pilot次数，静态判定fixture从不计正式数据。
最终学习策略成功率不作高门槛；数据50条仍必须物理成功、轨迹/视野/完整性合格。

## 具体实现与自测约束

阅读docs/stack-task-teacher.md、configs/simulation/panda_stack_task.yaml及新IDL；GetTeacherChunk positions按point-major60×16，prepare_kind区分teacher/evaluation_only，gate_state/failure保留。Start后等到capture时间不早于task_start_sim_time_ns，避免旧缓存时钟；查询任务恰好成功与teacher请求竞态，读取冻结TaskState再决定正常收尾。

允许修改runtime、必要backend只读观察质量/采样输出、契约必要注册、独立campaign/导出工具与配置、部署、测试和文档；不改几何、物理成功阈值、关节限位、60/5动作节奏。任何新增IDL必须必要且由监督者生成验证，不手写stub。尽量复用现有记录/审计路径；大模块按职责拆分，不引入动态插件、复杂继承或宽泛异常吞错。

使用绝对PROJECT_PYTHON；host单测缺ROS不得声称真实服务已测。Ruckig路径可加入主目录.build/follower-probe-venv/lib/python3.12/site-packages；backend source必须本候选。Ruff对本次改动通过。源码/资产只读、输出本工作树.build/reports/dsh-034。
禁止Docker、网络安装、Git提交、其他代理/其他工作树写入；禁止完整任务试采、正式采集或训练。可以纯组件/静态fixture自测，不执行完整抓放。交付可运行且配置驱动的入口及文档，明确由监督者执行的命令和预算语义。报告.build/reports/dsh-034/report.md，真实记录已测/未测与局限，完成退出。
