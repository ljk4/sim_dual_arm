# DSH-036：真实ACT PolicyProvider与固定种子仿真评估

035工具已验收并整合31文件；真实50条采集仍由监督者推进，随后训练，不能因暂时没有真实bundle生成假模型。首轮30分钟、最多一次集中返工。本项候选不运行完整任务，不开始额外训练；监督者用真实已训练bundle执行保留的ACT试采与20新种子评估。

## ACT服务

复用components/learning现有PolicyProvider/PolicyService/PolicyServer，新增具名ACT适配器/入口。禁止另建一套gRPC协议、复制fake provider或动态导入任意类。模型严格从035离线bundle加载，verify所有hash/profile/obs/action/stats；teacher.test_only假证据不能作正式模型。只输入当前三RGB320×240和16测量position，task_text固定。无物体姿态/教师阶段/成功判定输入；不调用IK/脚本补救，不覆盖左臂输出。

ROS-free独立进程，读取RGB payload并准确解码HWC→CHW浮点[0,1]，使用train processor；ACT eval/inference_mode，每个新observation调用predict_action_chunk完整60×16（50ms..3s），随后反归一化SI，原始16维输出全部保留。不能使用select_action的内部60动作队列掩盖每5步重规划；关闭temporal ensemble以匹配已验收调度。按协议处理open/reset/close/context、幂等request、最大会话、并发串行化和有界deadline。暖启动发生在ready之前，不把模型加载耗时藏在任务计时中。

测试provider可注入很小的typed model边界；正式只能ACT bundle，无constant/rescue fallback。模型错误、非有限/shape/名字/越界按已有契约拒绝并记录；不悄悄裁剪或放松控制约束。真实模型输出是否需通用投影，要等监督者证据再讨论，不在本项先加。

完整共享结构化日志：bundle/config/source摘要、加载/ready、open/predict/latency/拒绝/reset/close、峰值显存；不记录原始图像或张量。服务关闭有界且exit非零不伪装成功。

## 部署/评估

独立ACT Compose服务，复用035实际Ubuntu学习系统层与只读依赖/source/SDK/bundle/cache挂载，backend task_mode=inference，teacher能力禁用。沿用034 task stream/Stop/ACK/插值/实测反馈/MCAP/独立判定。配置endpoint和真实bundle/processor摘要来自文件，不手工在脚本散落假digest。只启动/停止本项目拥有的容器/网络。

评估沿用campaign锁/attempt ledger，可增加明确evaluation purpose；固定test=20000..20019，20次都计入分母，拒绝/超时/失败原样报告，不能替换seed、重复挑最好或按测试集选择checkpoint。默认先运行pilot预留1008/1009，两次仍计总≤10试采。任何完整task rollout先记账，包括ACT延迟/显存实测；不能称调试不计。

评价最低目标是已采集50→45/5实际训练ACT→实际RPC输出/跟随执行/传感反馈→独立任务结果的完整链路。用户本次不要求高成功率，16/20不作门槛；仍完整报告20结果及失败原因。若ACT全部在第一动作前被拒绝，只能报告闭环未完成，不能把20份失败manifest冒充执行过ACT。

评估报告列：模型/数据/代码/config摘要、seed、task/runtime/recording/质量独立结果、首个实际应用时间/块数、sim/wall时间、预测P50/P95/max、显存、失败类型与录制/预览路径；生成可检查三视角短视频和典型成功/失败关键帧。不把GT叠加图回流Policy或训练数据。后续优化建议简短，不主动扩大数据/训练预算。

纯组件测试：RGB通道/维度/顺序、train-only processor、60个新鲜点/时间、reset不复用缓存、wronghash/ctx、nan/shape拒绝、离线重载一致、服务关闭。候选不Docker/联网/安装/Git提交/其他代理/跨工作树写入、不完整任务。报告.build/reports/dsh-036/report.md，说明公共运行命令和真实已测/未测，完成退出。

## 035实际交接与公共部署补充

预期035交付dual_arm_learning.bundle.load_bundle/predict_action_chunk和BundleHandle，按035验收后的真实签名使用。bundle manifest有bundle_digest、processors.digest、profile.semantic_sha256、observation/action字段；不得手写假hash。原观察/action schema依profiles/observations/panda_rgb_joint_v1.json和profiles/actions/panda_joint_position_v1.json的canonical hash；服务open同时核对真实profile和calibration。模型包或生成配置应包含实际契约摘要，构造后快照保存。

当前公共入口tools/run/stack_once.py生成runtime/backend配置并复用native_policy.py；目前policy路径固定fake-policy服务。增加清晰的act选择、--bundle及学习依赖/source/cache挂载参数，继续同一生命周期监督/所有权清理，不能复制整个launcher。ACT使用独立act-policy服务和overlay，只合并backend/runtime基础层，不顺带启动fake-policy；runtime实际endpoint也应匹配。--bundle可来自仓库外的挂载数据盘，配置生成目录及RO边界须明确。teacher模式不需要Torch、Policy SDK变更或模型。

近期用户批准的接触卸载修复已由主助手在正式采集链独立验收：任务teacher/inference共用configs/simulation/panda_task_stream.yaml，contact_unload_timeout_s=2.0；新CONTACT_UNLOADING反馈/日志只描述接触卸载，不改变模型输出/参考/硬限位/机械臂检查。035工作树快照没有这些后续backend修复，036准备快照会包含；不要回退。

ACT的第0个输出对应当前观察t的执行参考t+50ms，学习dataset的delta0已是这条动作；服务输出时间为50ms..3000ms，不能再偏移一次。所有16轴原始输出仍照常传输，不替换左臂、不因夹爪闭合接触而替换预测开度。输出越界目前按契约拒绝，是否投影只能在监督者实际训练输出取证后再讨论，本任务不得先实现。

无GT补救的评估结果要能证明实际模型动作被接受并执行，而非只统计进程/服务起来。评估汇总可为read-only工具重建每attempt实际块、首应用时间、终态、指标，不更改已落盘台账。固定test20不得重跑替换seed，模型必须先冻结；pilot允许在剩余预算内按全局台账分配，不硬猜种子而覆盖已用值。

## 本机验收后的工具路径

PROJECT_PYTHON=/home/ljk/work/sim_dual_arm/.build/learning-dev-venv/bin/python（保留venv路径，不能resolve到系统Python）；LEARNING_SOURCE=/home/ljk/work/sim_dual_arm/third_party/src/lerobot-act-v0.6.1/src；POLICY_SDK=/home/ljk/work/sim_dual_arm/.build/generated/policy-sdk；纯pytest覆盖层/home/ljk/work/sim_dual_arm/.build/learning-test-tools。测试PYTHONPATH仅本候选learning/run_logging、上述上游/SDK/test-tools，不能加入ROS或dev整套site-packages。候选不安装依赖、不Docker、不联网、不完整任务、不训练、不改主工作树/不提交。

已实际构建并GPU验证sim-dual-arm/learning:act-dev，清除ENTRYPOINT，依赖挂载/opt/host-learning-venv（本机学习venvRO），固定上游/opt/lerobot（RO）、source/workspace（RO）、模型包另挂RO、日志output RW；NVIDIA4050/Torch2.7.1+cu126/NumPy2.2.6且rclpy不可见。Compose应使用该入口，其他机器用act目标按锁重建。参考035 docs/learning-training.md和Docker README。

035的bundle CLI新增--mode；模型checkpoint会绑定有效配置和数据manifest，训练--mode full导出同样传--mode full或用显式full.yaml。load_bundle/predict_action_chunk已通过真实ACT合成CPU往返/恢复，但任何synthetic bundle不得用于正式ACT服务。
