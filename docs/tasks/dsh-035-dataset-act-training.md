# DSH-035：数据转换、ACT训练与离线模型包

034的真实teacher与标签重放已通过；本任务在034最终收尾回归验收并合入后分派。首轮45分钟，最多一次集中返工。候选只做实现与合成组件测试；不自行采集正式数据、消耗任务训练额度或调用Docker。监督者负责真正50条采集、转换、训练和服务验收。

## 实现范围和边界

先读034实际导出格式和审计入口。保留ROS MCAP→轻量可审计导出→ROS-free LeRobot转换边界；不让NumPy2学习进程import rclpy。不要自行重新实现控制/MCAP审计。少量具名模块分管配置/转换/预算/训练/模型包，使用固定LeRobot源码，禁止动态插件和复制其ACT模型。

正式采集由034campaign取前50合格episode、≤100尝试、30GiB含失败，验证索引为成功顺序9/19/29/39/49；seed/source/context/代码/配置hash可追溯。独立task成功、录制完整、视野与轨迹质量分别通过才能进入正式集合。固定20测试种子不参与选择。合成样本测试明确synthetic，不冒充已获得50数据。

转换读取已验收的导出，逐帧核对capture和sim时间；不能按数组相邻假定20Hz，不把pending/BRAKE/Stop之后旧轨迹作label。动作保持t+50ms实际执行参考，原始模型/教师预测和实测均另存。60帧future action使用0..59×.05delta，episode边界padding mask正确且不跨episode。三RGB320×240的通道/颜色正确，16维state/action单位/顺序来自profile；三路H264 RGBEncoderConfig(vcodec=h264,g=2,crf=18,preset=veryfast)，视频写入有界。

分别创建45train/5validation两个独立LeRobotDataset（不是同一全局stats后筛episode）。processor只用训练meta.stats；保存processor和stats摘要，验证不得参与拟合。转换manifest含每episode原始引用/映射/字节/帧数/起止sim时钟/分区和文件hash，不能覆盖原始数据，半成品不当成功产物。抽查首/中/尾及最后padding、不同ep隔离。

## 环境与配置

固定third_party/src/lerobot-act-v0.6.1 commit7e241bd630a3719a56157a497ce5d08f244784f1；94项哈希锁从主工作区.build/reports/s6-preflight-20261101/learning-requirements.lock提升deploy规范路径。
已有真实Ubuntu24.04系统镜像sim-dual-arm/learning:system-probe-20261101-v3，ID5546a1acc44e5e6d0581903fe37bec311996429498bca8826008f2305c43ffac；已验证Python3.12/FFmpeg/libpython/libgomp。主.build/learning-dev-venv可只读挂载当前机器使用；跨机器容器内按锁重建环境，不复制未知ABIvenv。
现有deploy/docker/learning/Dockerfile保留protocol/policy目标，增加职责清楚的ACT系统层/依赖引导和开发挂载入口。源码/上游/SDK/依赖/数据/权重缓存只读，训练output RW。参考docs/learning-environment-preflight.md，Docker多挂载少COPY。

训练配置全部来自YAML：三RGB+16state→60×16，ResNet18预训练，dim256 FF1024 enc4 dec1 VAE4 batch8，float32（已验证本机6GB），seed、lr、weightdecay、gradclip、workers、steps、日志/val/save频次、预算/路径均可改。官方权重位于.build/cache/torch/hub/checkpoints/resnet18-f37072fd.pth，SHA f37072fd47e89c5e827621c5baffa7500819f7896bbacec160b1a16c560e07ec。
离线运行，不上传Hub/W&B，不在训练隐式下载。不能生成伪造权重或用其他任务checkpoint当已训练ACT。

## 训练和日志

实现明确smoke/overfit/full三种模式，smoke/overfit也计任务训练累计≤14400s；包含重启、验证、保存等墙钟，原子预算账本在真正GPU训练前预留/启动，异常不能重置。已有预检是synthetic，无任务训练记录；预算跨进程加锁，遗留RUNNING保守计实际elapsed或保留上限并报告，不静默清空。

先少量真实数据smoke，单episode overfit检查loss有限/下降/梯度非零，再45/5full训练。用户以先跑通为目标，默认计划可选smoke128steps、overfit500steps、full5000steps（仍用完整45/5数据，不缩成仅一条假训练），但必须配置而非写死。训练前核对50合格、45/5互斥及文件hash，full拒绝数据不足。
使用上游ACTPolicy.forward，包含action_is_pad；验证用train processor和masked L1，不只记总VAE loss。模型选择只看验证，物理成功率另报告；测试20不选checkpoint。保存latest/best与optimizer/scheduler/RNG/step、配置和预算引用，可有界恢复。训练结束/预算接近/中断保存真实最后checkpoint与结束原因；保存失败非零退出。

共享robot_run_logging日志与配置快照：每阶段开始/结束、step/loss/L1/KL/lr/梯度/耗时、val、VRAM、保存、异常、累计预算；普通日志不塞张量/图片。另轻量metrics JSONL/CSV可分析，字段含单位和step。结果报告实际模型参数/步数/设备/时长/峰值显存/验证值和限制，不把低loss当堆叠成功。

## 模型包

权重/config/pre/post processors、train stats/profile/obs/action摘要、dataset/split/seed/配置/上游引用、完整文件校验和组成离线bundle。模型初始化可pretrained_backbone_weights=None后严格load已训练完整state_dict，明确该初始化覆盖避免再次下载；同一实际batch导出前后预测一致验证。bundle无任意pickle执行作为部署路径（safetensors），训练恢复状态如使用torch.save只读本工具自己的可信文件并明确边界。

后续036接入实际ACT PolicyProvider；本项可以提供小型typed load/predict函数，不写第二套gRPC服务。不改现有Policy协议。推理调用predict_action_chunk，每次新鲜完整60点，禁止select_action缓存或覆盖左臂输出。部署前不裁剪模型输出、无物体真值/phase输入或脚本补救；若真实输出不合契约，由监督者记录和讨论。

## 交付检查

合成小导出→两个独立dataset→train-only processor→未来mask/顺序/时钟/颜色/反归一化检查，拒绝混seed、误label、重复/跳帧伪装20Hz、数据未齐full、hash错、预算耗尽/重启清零。预算/路径/非法配置等纯组件测试不需GPU；依赖可用的合成GPU检查由监督者另计synthetic预检，不得冒充任务训练。
DSH使用主目录.build/learning-dev-venv/bin/python（绝对），源码PYTHONPATH仅本候选learning、run_logging、上游固定源码；没有rclpy/Torch的unit环境可以skip集成但明确报告。候选不安装依赖、不申请Docker、不联网、不改主工作树、不Git提交、不运行其他代理。
报告.build/reports/dsh-035/report.md，列出公共命令、模块职责、真实测试和待监督者执行项。完成退出。

训练检查点保留策略配置化，默认latest+best，阶段最终状态另存，避免每次完整optimizer重复保存耗尽磁盘；保存前核对输出盘余量，不删除其他项目或用户文件。完整训练已更新为5000步的首轮短计划；后续不自行耗满4h来追高成功率。

## 034交接的实际接口（定稿时再核对）

当前学习任务使用profiles/robots/panda_stack_learning_v1/profile.json及panda_stack_learning.yaml，右腕FOV68度，三图仍320×240；旧panda_stack_v1是诊断基线，不混用标定。teacher与ACT均启用参考速度PD前馈、可靠传感流，限位/跟踪容差和2秒/30秒判定不变。
导出labels.json包含episode_id/robot_profile_hash/seed/summary/labels；每条row含capture state_seq、observation_sim_time_ns、label_sim_time_ns、applied_positions、measured_positions/velocities、rgb.camera.path/png_sha256/rgb_sha256以及raw源observation引用。raw_predictions.json按执行块单独存60点；frame_index.json提供完整映射。数据路径从正式campaign select结果读取，不扫描所有pilot/fixture凑50。
LEARNING_SOURCE=/home/ljk/work/sim_dual_arm/third_party/src/lerobot-act-v0.6.1/src；宿主自测显式PYTHONPATH为本候选components/learning/src、本候选libraries/run_logging/src和该只读LEARNING_SOURCE。工作树中third_party/src忽略未复制，不能因缺路径联网重克隆或改动上游。监督者将提供PROJECT_PYTHON=.build/learning-dev-venv/bin/python。

测试环境补充：学习venv不含pytest。监督者已提供只含pytest/_pytest/pluggy/iniconfig/pygments/py的纯Python测试覆盖层/home/ljk/work/sim_dual_arm/.build/learning-test-tools。测试时将它追加PYTHONPATH，并设PYTEST_DISABLE_PLUGIN_AUTOLOAD=1、PYTHONDONTWRITEBYTECODE=1；已验证pytest8.3.5、NumPy2.2.6且无rclpy。不要把dev/native整个site-packages加入学习环境，不能混入NumPy1或ROS。生产94项依赖锁不含测试覆盖层且未修改。

正式采集将由监督者与本任务并行运行；输入快照不含实时增长的数据，不能因此生成假50条或自行读取/修改主campaign。提供公共convert命令供监督者使用真实select结果。配置默认路径与035命令应能衔接tools/run/stack_once.py的export/labels.json；输出均可指定到独立学习DATA_ROOT。
