# DSH-036 唯一集中返工

继续原工作树/会话，25分钟。首轮归档source-first已冻结；主助手独立44项组件/RPC检查通过，但以下阻塞验收问题需修复。原约束不变：不Docker/联网/安装/训练/完整仿真/修改主目录/提交。明确使用本机learning venv（不能resolve到系统Python）、固定只读上游/官方Policy SDK/纯pytest覆盖层。

## 当前真实状况与用户最新决定

主目录已正式52次取得50条合格，45训练19676帧、5验证2160帧；真实smoke128及overfit500完成，full5000正在RTX4050执行，首次全5验证2160帧L1=0.142。真实模型不在你的工作树，不读取/修改主campaign或模型。
用户看到smoke输出越界证据后明确选择“保持严格拒绝，等完整训练输出后再决定”。**本轮不得添加投影/裁剪/放宽硬限位/替换左臂，监督者会在最终模型后离线复核，再决定是否讨论。**不运行ACT pilot/test，完整训练前不消耗这些种子。

## 必须修复

1. **不能把无ACT证据计入闭环/成功**。监督者复用test_stack_evaluation.make_attempt生成纯合成记录，去掉launch.act、设置policy_mode=fake、prediction_count=0并移除act-policy-log，保留policy来源labels；当前_metric_for_seed仍报告act_executed/closed_loop/success全部true、bundle_digest=null。反例在主目录.build/reports/dsh/dsh-036-fake-evaluation-counterexample.json。不能仅凭configuration.action_source==policy断定ACT。核对record.mode=act、launch.policy_mode=act、非synthetic/full bundle/profile/model身份、真实预测成功日志/计数、model digest一致以及实际接受/应用块证据；缺少/冲突/只有fake/replay必须明确incomplete，不算ACT执行。
   stack_once同时检查STACK_CAMPAIGN_MODE与实际来源/模式一致，act预留不能通过auto无bundle或显式fake静默变成fake rollout；普通teacher/replay已验收行为保留。增加确定性反例，不通过mock伪装已采20次。
2. **物理任务结果与图像/运行质量分开**。当前success要求quality_qualified，这会把policy模式全时段视野诊断（无teacher phase，初始遮挡可能失败）混作堆叠失败。分别报告task_success（独立释放/退开/稳定2s/30s物理事实）、act_executed/closed_loop、runtime/recording/launch/quality及run_success；固定20物理成功率必须由可信ACT执行+物理任务事实计算，质量另列，缺证据/未执行不能成功。不能仅有result.outcome字符串却无evaluated/test_only/task判定事实也算物理成功。故障与超时如实入20分母，不能替换seed。
3. **时钟语义准确**。first_applied现在取第一个label_sim_time_ns，是20Hz标签采样时间，不是2ms物理开始应用（fixture started1024ms、label1050ms）。分别命名first_started/first_reference_applied（有确切事实的时间）与first_exported_label；只有20Hz label时报告时间分辨率/缺精确信息，不能称精确物理首应用。sim_seconds当前max(label)-min(obs)排除启动与停止，不能冒充30s任务时长；从task_result.task_start/completed/sample的仿真时钟计算task_duration，执行片段另列，wall另列。新增这些时间有差异的检查。
4. **warm-up与缓存挂载**。ActProvider._warm_start只检查ndim=2，错误的(1,1)/NaN仍会ready。检查完整(60,16)及finite，失败不得listen/ready；用profile初始状态替代不合法全零关节状态作为warm输入（不依赖物体真值）。Compose的LEARNING_CACHE_ROOT=/opt/learning-cache指向RO权重盘，改为/data下的独立可写cache/HF_HOME，权重RO仍保留；本机数据/源码/模型/SDK均RO。进程加载/暖启动异常要留下结构化失败/summary或明确启动失败产物，不能仅stderr无法定位。
5. **操作文档不能重置台账**。docs/act-policy-service.md当前WORKTREE+默认campaign会新开工作树var/ledger，且没有开头路径/挂载检查。改为验收合入后的主ROOT，显式CAMPAIGN_CONFIG指向已有var/campaigns/panda_stack_v1/campaign.yaml，每次命令都传--project-root ROOT --config CAMPAIGN_CONFIG；不得用空campaign重新分配pilot/test。开头提供ROOT/DATA_ROOT/BUNDLE/SDK/资产/learning venv/source/cache变量、findmnt/df。pilot由现有台账分配，勿硬猜1008/1009。清楚说明模型需训练完成、离线严格范围复核后才pilot，再冻结模型一次跑20个未用种子；保持公共Compose入口和所属清理规则。

保留已有RPC兼容，勿新增第二套协议或复杂插件。报告真实测试/未测；对旧policy_service/server回归、fake/replay误计反例、task成功但质量失败、缺物理事实、时间分辨率、warm错误/非有限、act预留错模式及共享test池做有意义检查。完成退出，本阶段不再第二次返工，主助手接管剩余与真实验收。
