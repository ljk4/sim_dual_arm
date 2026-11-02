# DSH-032 验收记录

2026-11-02，已验收并合入50个文件，192个Python文件与source-resume8冻结候选一致。
主HEAD/暂存区不变，未提交主工作区改动。旧暂停交接保留于文末，已由本次结果取代。

## 最终行为与证据

- Policy服务与客户端：真实gRPC、三RGB/16测量、每次完整60×16、每5点连续执行；共享有界策略日志，deadline取调用方与服务配置较小值。
- NativeControlClient分别验证整体流首块身份与当前块回执/接受时间；未知ACK查询同块并幂等重发。
- 修复订阅端同类身份混用，以及后端停止时活动块终态状态已更新却漏发事件的问题。
- 录制收尾等待整体终态、所有已接受块终态和对应参考入队；不因跨DDS话题先到的整体反馈而提前关闭。
- MCAP v4保留完整480个原始点、真实参考/实测和来源；v1/v2/v3/default2保留。
- 最终983项通过：871 unit、111原生/真实ROS（含7项实际policy正常/迟到/旧epoch/断连/运动中断连/End/signal）、1项旧CLI超时竞态。
- 另14项最终运行时专项全部通过；旧ROS/MCAP先114通过，唯一CLI清理竞态已修正夹具并最终复验。之前完整115通过的证据仍保留。
- 真实公共入口最终launcher=0/backend=0，8块实际执行、8块应用、480原始点，独立audit=0；物理/规划/渲染均关闭，拥有的容器/网络清空。
- 实际MCAP原始未选尾部/反馈参考/同capture不同图像三类篡改均拒绝；首个探针误用了ROS字段，修正后通过，原输出保留。
- 三个进程共134条结构化事件schema与连续序号通过，summary均complete、无丢弃。
- 正式策略镜像实际构建，导入路径/新descriptor验证，镜像不含rclpy/Torch；PyYAML固定哈希锁。官方生成的Policy SDK已提升到.build/generated/policy-sdk。

证据根：var/runs/dsh/dsh-032/20261101T145730Z/，最终source-resume8、all-final-resume8、
public-entry-final、logging-final.json、tamper-resume6。全部首轮/返工/监督者修复失败均保留。
一次监督者gRPC关闭在负载下超时，最终完整回归及公共服务关闭通过；未把失败抹除或修改退出口径。

接管候选迁到项目内.build/worktrees/dsh-032-supervisor，原兄弟工作树及归档保留。
迁移前两次自动权限审查超时；没有被判定操作不安全，工作树创建成功后在项目授权范围内完成编辑。
集中返工额度已用，后续修复由主助手接管完成；未再次反复派发返工。

S3接口闭环完成；这是无物体任务的小运动诊断，正式数据0、完整抓放试采0、ACT任务训练未开始。
下一阶段任务书为DSH-033，开始独立任务判定与可选teacher；用户已恢复持续执行，成功率不作为本次跑通门槛。

---

# DSH-032 暂停交接记录

2026-11-01，按用户“约15分钟内收尾后暂停”的最新要求停止推进。
**本阶段尚未验收、未合入主工作区。** 上一阶段DSH-031已验收整合；本阶段候选留在独立工作树。

## 候选与实际检查

- 工作树：`/home/ljk/work/sim_dual_arm-worktrees/dsh-032`。
- 会话：`session-f778eb6c-ab88-439b-8622-e263e43392c0`，首轮1226.762秒、184步、183工具；唯一返工56.747秒、12步、11工具。两轮均已退出，不继续后台运行。
- 记录：`var/runs/dsh/dsh-032/20261101T145730Z/`。原始首轮/返工归档、三份冻结源码、官方SDK、镜像构建与失败现场保留。usage缺项，不估算费用或节省比例。
- 官方grpc_tools实际生成新Policy SDK，字段号与observation相对时间已验证；没有采用DSH自测的手写grpc stub作为正式证据。
- 首轮870单测通过；返工871单测通过。首轮冻结候选104项原生/真实ROS和115项旧ROS/MCAP回归通过。
- 返工改变了v4审计分派和块数校验；上述219项回归基于source-first，不能笼统当作最终候选全部已复验。
- 最终监督者接管又修改了policy_server.py/run_fake.py两文件；完整871单测尚未在该两处修复后重跑。
- 最终source-primary的191个Python文件与工作树一致。Ruff/diff在source-rework通过；后续需连同最终修改复核。

候选实现包含真实Policy服务/客户端、连续流、MCAP v4、新配置和公共Compose入口。
它们存在于独立工作树，主工作区仍使用已验收的原生诊断入口。

## 已修复与未通过

1. 实际首轮策略镜像导入失败：缺PyYAML。唯一返工增加6.0.3带哈希锁，实际重建成功；新入口默认后端改为follower-dev。
2. 首次公共运行失败：fake包装配置二次按服务配置加载，报unknown fields fake/service。监督者改为传已验证的PolicyServiceConfig，避免二次读取。原失败证据保留。
3. 修复后公共运行启动了真实服务，仍因`INVALID_FOLLOWER_EXECUTION`失败；记录正确标为partial/failed，未误报任务或记录成功。
   当前native_control_client.py约590行将整体NativeExecution.chunk_seq强制等于新FollowerReference.chunk_seq。
   这是错误的整体/每块语义混用：后继块已接受但仍PENDING时，整体反馈仍可指向前一活动块。
   下次应验证SubmitFollower的每块receipt/QueryNativeChunk、完整context/request/seq与对应reference，
   保留整体反馈独立校验，不可仅删校验或伪造当前块来通过。
4. 两次实际公共运行均非零退出；测试拥有的容器和网络已全部清理。首轮失败关闭还出现backend137，需要下一次补验正常和故障的完整停止路径。

## 恢复顺序

先读本记录和pause-summary.json，再检查工作树是否有人工修改；不要直接合入。
使用source-primary及公共失败日志定位回执语义，补真实active+pending反例和正常8块贯通，
再做v4 MCAP独立重建/源参考捕获篡改、RPC迟到/身份/断连/reset、EndEpisode/信号/日志写盘故障及清洁关闭。
新服务统一日志仍需专门核验，不能把控制台readiness打印当作完整策略日志系统。
集中返工额度已用；后续由监督者接管必要修复或与用户恢复后的执行范围一致地另立任务，不重复无限返工。

未启动S4示教、完整试采、正式采集、ACT任务训练或20种子评估。正式数据0、完整任务试采0。
S4/数据训练提示草案留在`.build/reports/dsh/dsh-033-teacher-task-draft.md`和
`dsh-035-data-training-checks-draft.md`，均未分派。未提交主工作区改动，HEAD/暂存区不变。
