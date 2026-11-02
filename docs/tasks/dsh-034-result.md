# DSH-034 运行、采集与标签重放验收

2026-11-02，已完成真实teacher→物理堆叠→MCAP审计→逐帧RGB/执行参考导出，以及独立标签重放→物理堆叠。程序示教与策略共用连续follower/ACK/插值/反馈/受控停止链；推理部署不提供teacher能力。本项合入67文件，220个Python冻结一致；主HEAD/暂存区不变，未提交。

- DSH首轮1092.310秒/194步/222工具，唯一返工580.616秒/77步/77工具，均退出0。随后主助手接管。会话session-408448de-9498-4571-908d-c70679a75492。
- 最终全量批次1086项通过，1项旧有限MOTION停止窗口失败；原失败保留。专项修复已发出指令的有界ACK等待、有限MOTION与stream终态规则区分、终态入录制队列先于完成标记；最终32项相关真实ROS/控制/策略批次及确定性反例通过，详见ack-related-final/pytest.xml。
- 真实历史MCAP独立对照40/9/9条执行标签、120/27/27张RGB，参考数值误差0。RGB与载荷hash/320×240、Stop截止、相对时钟、不同身份与profile均校验。
- 私有学习场景仅调整右腕FOV85→68度，独立profile/标定，103静态布局全部满足初始视野，最小源块68像素。旧场景不变。
- 任务部署启用PD参考速度前馈及RELIABLE32帧传感流；旧部署默认关闭。前馈只使用既有参考位置/速度，不改物理模型/限位/跟踪阈值。失败接近段物理对比最大误差0.0629→0.00939rad，连续超差1.992→0秒。
- 制动1ns末段相消导致虚假加速度1.000000027超限，已用整数剩余时间修正，硬限值1.0不变；原25指令重算全部通过。

持久campaign位于var/campaigns/panda_stack_v1，完整pilot共7/10：

| 运行 | 结果及计数 |
| --- | --- |
| teacher1000 | 接近跟踪超时；失败保留 |
| teacher1001 | 真实堆叠成功，初始腕相机像素不足，不合格 |
| teacher1002 | 数值制动拒绝，失败保留 |
| teacher1003 | 真实堆叠成功，一帧三RGB缺失，不合格 |
| teacher1004 | 真实堆叠、完整录制、连续标签、各phase视野全部合格 |
| replay1004 | 首次相对时钟未对齐被拒绝；已修复，仍计一次 |
| replay1004-v2 | 真实堆叠成功，runtime/policy/backend均0，录制完整、连续执行、所属资源清理；非teacher/非ACT，不计训练数据 |

仍保留2次ACT pilot，剩余1次备用。以上均不计正式50条。接续正式采集与DSH-035数据转换/ACT训练工具，然后036真实ACT仿真推理；用户本次以跑通闭环为目标，低成功率如实报告。

证据根var/runs/dsh/dsh-034/20261102T041852Z/：source-ack-final冻结、all-stop-final全量及各次失败、ack-related-final、export-label-fix、servo-comparison.json、learning-camera-static-68、pilot1002-recompute-fixed、integration-review.json。操作说明见docs/native-teacher-runtime.md。
