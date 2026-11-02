# DSH-033：独立堆叠判定与可选程序示教

S4第一部分，首轮45分钟，最多一次集中返工。032验收后分派。只做组件与静态/局部检查，不启动完整抓放试采。
之后单独034执行最多10次试采，按台账计数；再正式采集/转换、训练和20种子评估。
032的8块是短诊断，不可直接代表30s任务；本项规划至少120块/episode的有界容量，
核对expected requests/refs/事件、RPC幂等缓存与后端128块历史，不让64条旧缓存截断任务。
容量从配置/可解释硬上限读取；不能改成无限内存或静默丢失仍需审计的记录。

## 职责

在backend_sim添加task_config/stack_task/teacher等少量职责模块；新仿真专用IDL只做必要字段，旧33个msg/srv/action原文保留（另有CMake/package两个源文件）。
单物理owner仍是唯一可读写live MjData的线程。采样完整不可变任务状态，独立成功判定在物理tick推进；
IK使用私有mink.Configuration/MjData，可复用已验收kinematics目标/约束，不在物理tick中求IK。
teacher只产生16维原始目标，不能调用apply_targets/mj_step、修改物体或绕过SubmitFollower。
复用032连续runtime和日志/MCAP v4；action_source=teacher与policy互斥，推理配置不启用teacher。
不要复制完整Runtime/控制器/writer，不动态加载类，不引入新大型机器人框架。

## 独立任务状态/判定

显式可选task配置；读对象中心/四元数/速度、source-target接触法向/法向力、指-块接触、TCP和测量夹爪状态。
使用已知4cm/6cm真实几何，配置支撑高度误差、XY支撑边界余量、倾角、线角速度、接触力、
双指释放、TCP退开距离、稳定2s、最大30sim s及wall watchdog；阈值有单位、校验和解释。
仅source比target高不算成功；target必须在桌上、source在target上，不能指尖支撑/夹持、
两块都在桌面、source在空中、桌面误判或接触一下即通过。超时/掉落/控制失败与success独立原因。
稳定窗内任一条件失效或reset/epoch改变即重置；成功锁存，记录首次满足和完成sim时间、实际条件值。
任务计时以本episode正式开始（完成reset/有界IK准备后、开始获取首条示教/策略预测前）为起点，
到stable2s结束不超过30sim s；初次预测等待与活动动作都计时，不能无限把活动动作称准备。
reset/启动/只读IK预计算可在episode前完成，但各有wall timeout且期间不执行任务动作。
新增仿真状态topic/query包含context/序号/采样sim时间及task_config_digest；固定长度、无图像。
真值只送teacher与独立评估/诊断，不放Policy Observation/ACT feature。推理时teacher RPC不提供能力/拒绝调用。
保存任务事实于单独记录通道/sidecar，明确与Policy输入隔离；原MCAP版本语义不变。

## 示教

输入context/episode/源capture_seq与observation时间；对已保留的不可变capture求解，不能假装用较早图像时刻却读无标识的未来状态。
最小仿真接口返回60×16、50ms相对Observation时间、phase及有界原因/源时间；沿用032 target validation。
左臂保持初始待机，右臂依次pregrasp→grasp→close→确认抓牢/抬升→transport→lower→open→retreat→stable。
阶段过渡基于实测位置/速度和双指同物体稳定接触；禁止只看命令完成或固定睡眠就认定抓牢。
夹爪闭合目标可为0但实测约0.04，不覆盖原目标。lift应验证source真实抬起且双指保持接触。
IK复用mink1.3/DAQP、明确姿态/关节限制及冻结左臂；复用已有reach seed，解不收敛/不安全则显式失败。
规划碰撞检查区别允许的指-source接触与非法机械臂/桌/柱/目标碰撞，不全局关闭碰撞检测。
每段连续时间轨迹用可解释平滑插值/速度加速度约束（可用已验收primitive），不是路点瞬移；
保持60点raw/执行5点同一受限跟随。阶段门控未满足时有界保持/失败，不用无限等待逃避30s。
阶段时长/容差/安全抬升/降落/释放/退开均配置，合理总计划需能覆盖稳定2s且小于30s。
示教可以读真值，推理只能由ACT处理，服务能力与部署清楚隔离。

## 验收与范围

配置非法值启动前拒绝，任务判定正反例：自由静置真实接触支撑/未释放/无接触/姿态或速度不符/抖动重置/超时/reset。
正例可以专门的静态物理fixture在reset时摆叠块；必须标记fixture，不作数据episode，不算任务示教成功。
IK/目标60点时钟和左右顺序/限位/私有状态/碰撞保持的组件测试，不做完整抓放rollout。
新增IDL真CDR和真实ROS查询/禁用能力检查由监督者执行；业务组件可自行host native测试。
之后034才实际从随机初始自由物体完成抓放、三阶段视野检查、标签重放、落盘审计；所有完整尝试进入≤10台账。
报告实际结果与未测项，不把局部测试标为S4完成。不提交Git/Docker/联网安装/修改其他工作树。

## 具体交付边界与接口

增加最少的具名接口：PrepareStackTask（有界只读规划）、StartStackTask（明确30s起点）、
GetTeacherChunk（针对保留的capture产生60点目标）、GetStackTaskState（只读任务事实）；
可用独立StackTaskState topic给运行时/判定记录。避免一个service用任意operation字符串路由。
准备/启动请求携带完整context、lease和幂等request_id；错误epoch/未持lease/非静止prepare拒绝。
准备不能执行动作；Start之后不准重置物体、重写qpos或无限重复Start来延长30s。
reset撤销计划/稳定窗/旧RPC，重复同ID同内容返回原事实，变内容拒绝。
服务关闭、队列、deadline、规划线程均有界并纳入同一worker清理；不能阻塞物理owner求IK。

推荐阈值初值写入panda_stack_task.yaml：source中心位于target支撑域内且边缘余量2mm，
高度差=两块半高之和±5mm，倾角≤10度，源/底块线速度≤0.02m/s、角速度≤0.2rad/s，
法向支撑力≥0.05N、target与桌面真实支撑、无手指接触、右夹爪开度≥0.06m、
TCP与source中心距离≥0.08m，连续2s；只有满足全部条件才success。
这些是实现初值，可根据正确物理量解释小幅调整；不得降低2s/30s或关闭碰撞、用高度独判。

示教计划用已有kinematics_check配置/FrameTask求私有6关键姿态；解算不改活动场景。
规划duration应由配置及速度/加速度上限推导或验证：手爪总开度硬速度0.04m/s，
0.08m全行程不能用1s完成；若用三次smoothstep至少3s，使用其他轨迹需给峰值证明。
右臂home到pregrasp也不能简单1s跳到大角度。默认总流程给稳定2s和门控等待留余量，
计划超30s必须拒绝并报告具体段，不能偷偷延长任务时限。

本项先实现backend/契约/配置和必要runtime接口适配，不实际运行完整任务。
若runtime编排涉及较多代码，可在交付中明确由034完成；不能留下伪装可用的TODO入口。
后续034统一teacher/policy stream调度，复用NativeControlClient；teacher和ACT相同SubmitFollower/
5点执行窗口/采样20Hz，不能在示教单独改用严格MOTION。
现有source-resume8已补按块回执/整体身份/终态事件收尾；复用，别复制旧有错误逻辑。

## 自测与配置规则

使用绝对PROJECT_PYTHON并设置SIM_ASSETS_ROOT/SIM_ASSET_SUMMARY；
Ruckig导入路径可加入主目录.build/follower-probe-venv/lib/python3.12/site-packages，
候选backend/contracts仍从本工作树导入。源码和资产只读，报告写本工作树.build/reports/dsh-033。
不得调用Docker/联网安装/Git提交/修改其他工作树或启动其他代理。
允许更改backend_sim、contracts Python/ROS注册、新task/teacher配置、ros_support转换、
对应native_runtime必要配置适配、测试与文档。不得更改场景尺寸、现有关节限位或模型输入/输出。

所有service/node/task/teacher日志复用robot_run_logging，不打印大量目标/图像；
阶段转换、失败、时间预算、context/源capture及IK残差/耗时有结构化事件，物体状态进数据流。
配置快照与文件hash、固定依赖/资产来源可追溯；旧接口/普通native诊断回归不得受影响。

报告.build/reports/dsh-033/report.md列明实际测试、尚未完成接线和下一项所需字段；完成退出，不提交。
用户现要求先跑通学习闭环，不要求高成功率，但物理成功判定、数据50合格、时间/存储预算保持。
