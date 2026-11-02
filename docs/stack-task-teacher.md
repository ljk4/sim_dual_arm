# 独立堆叠任务与可选程序示教（DSH-033）

2026-11-02。本文件记录 S4 第一部分的真实实现边界：只做组件、IDL、配置和 native
局部检查，不启动完整抓放试采，也不把局部测试当作 S4 完成。完整 teacher/policy
stream 调度、三阶段视野检查、标签重放、正式采集/训练/20 种子评估属于 DSH-034。

## 配置与容量

- `configs/simulation/panda_stack_task.yaml` 是任务判定、示教时序和容量的单一来源。
  4 cm source / 6 cm target 的半高必须与冻结场景一致；加载时交叉校验，不在代码中
  再写一遍几何真值。
- `configs/simulation/backend.yaml` 与 `backend_follower.yaml` 保持默认不启用 task；
  新增 `backend_stack_teacher.yaml` 在 follower 部署上同时启用 task+teacher，
  `backend_stack_inference.yaml` 启用 task 但 `task_mode: inference`。推理时
  BackendInfo 不发布 `teacher_chunk`，`GetTeacherChunk` 返回 `TEACHER_DISABLED`；
  Prepare 返回 `prepare_kind=evaluation_only`，不构造 TeacherPlan、不求 IK。
- 30 s、20 Hz 的 0.25 s 窗口对应 120 块；配置要求 `expected_chunks >= 120`、
  `max_episode_chunks >= 120`、`rpc_cache_entries >= max_teacher_requests`、
  `backend_chunk_history >= max_episode_chunks`。本部署使用 256，后端
  `limits.command_history/chunk_history = 256`。事件/引用队列在同一加载路径校验，
  不使用无限内存，也不静默丢弃仍需审计的控制事实。
- 运行时 `NativeControlClient` 的 stream origin 缓存由 `max_chunks` 推导，下限 128、
  上限 512，替换旧的固定 64；runtime `max_chunks` 硬上限从 64 提高到 512。

## 独立任务判定

`stack_task.py` 只消费物理 owner 在 tick 后复制的不可变 `TaskObservation`：

- source/target 的中心、四元数、线速度、角速度；
- source-target 与 target-table 的竖直法向接触力（只累计世界 Z 分量足够大的接触）；
- 四指与两块的任何接触计数；
- 右夹爪总开度与右 TCP 到 source 中心距离。

14 个条件全部为真才进入稳定窗：高度差、XY 支撑域、source-target 支撑、target
真实桌面支撑、source 不靠桌面、两块倾角、两块线/角速度、无手指接触、夹爪释放、
TCP 退开。稳定窗连续 2 sim s 才锁存成功；窗内任一条件失效即清零。30 sim s 和
60 s wall watchdog 是独立失败原因。记录首次满足/完成 sim 时间与完成时的实际条件值。

固定长度 `StackTaskState` topic/query 包含 context、序号、采样 sim 时间、
task_config_digest、plan_digest、两块位姿/速度、接触力、夹爪/TCP、条件位图和
时间事实；没有图像。真值只进入该诊断/评估通道和 teacher，不进入 Policy
Observation/ACT 特征。

## 程序示教

`teacher.py` 从 prepare 时复制的不可变 snapshot 求解六个 private mink key pose，
复用 `kinematics_check.yaml` 的 mink 1.3/DAQP/FrameTask/碰撞预算。碰撞 pair 只允许
指-source 接触；机械臂、桌、柱、target 的碰撞仍检查。每个 joint 段在私有
`mink.Configuration` 上按采样路径检查；close/open 关键姿态另用实测合理的
0.04 m 闭合开度（原始 close 目标仍为 0）和释放开度检查指-桌/指-target 碰撞。每段采用三次 smoothstep，
时长由速度/加速度上限推导；0.08 m 夹爪全行程在 0.04 m/s 硬速度下至少 3 s。
计划总长超过 28.5 s 或 30 s 上限时拒绝并报告具体阶段。

每段推进不是按计划时间自动完成：`TeacherProgress` 使用对应 capture 的不可变
实测关节位置/速度、夹爪开度/速度、双指-source 接触力/指速、source 高度和 TCP
距离做门控。机械臂/夹爪到位并稳定 `gate_stable_duration_s` 才推进；close 必须
双指同 source 接触且力/指速达标；lift 还必须 source 实际升高；open 必须实测释放；
retreat 必须 TCP 退开。阶段等待上限为 nominal duration + `phase_gate_extra_timeout_s`，
失败原因写入 `gate_failure`，全局 30 s 不延长。未来 60 点不会跨越未通过门控的阶段，
只在该阶段末端 C1 保持；stable 阶段在任务 RUNNING 期间持续保持末端目标，直到独立
判定成功/失败或 30 s。

`GetTeacherChunk` 只对 `capture_seq` 对应的已保留 capture 时间求解，要求
`observation_time` 与该 capture 的 sim 时间完全一致且不晚于当前 sim 时间。返回
60×16 raw target、`(i+1)*50 ms` 相对时间、phase、源时间和有界原因。左臂原始目标
保持 initial standby；teacher 不调用 `apply_targets`/`mj_step`、不写物体，也不绕过
`SubmitFollower`。每段 5 点执行窗口和 20 Hz 采样由 034 复用 032 的
`NativeControlClient`/连续 runtime。

## 接口

新增仿真专用 IDL：`StackTaskState.msg`、`PrepareStackTask.srv`、
`StartStackTask.srv`、`GetTeacherChunk.srv`、`GetStackTaskState.srv`。旧 33 个
msg/srv/action 原文未改，只更新 CMake 注册。

- `PrepareStackTask`：完整 context/lease/幂等 request_id；错误 epoch、未持 lease、
  非静止 prepare 拒绝。只读规划，不执行动作。
- `StartStackTask`：明确 30 s sim 起点；重复同 request 返回原起点，变内容或重复
  Start 不延长预算。
- `GetTeacherChunk`：保留 capture + 幂等 request_id；缓存满时拒绝而不是静默丢块。
- `GetStackTaskState`：只读任务事实。

IK/teacher 规划在独立有界 planner 线程；物理 owner 仍是唯一读写 live MjData 的线程。
服务关闭、队列、deadline 和两个 planner 都纳入同一 worker 关闭预算。

## 本项实际验证

host native 测试（设置 `SIM_ASSETS_ROOT`/`SIM_ASSET_SUMMARY`）覆盖：

- 任务配置非法值、容量不足、未知键、几何不一致启动前拒绝；
- 14 条件正例/反例、抖动重置、2 s 锁存、30 sim s/60 s wall 独立超时、reset 撤销
  epoch/plan/RPC、prepare/start 幂等与冲突；
- 真实 MuJoCo 静态叠块 fixture（明确仅 fixture，不是数据 episode/示教成功）通过
  全部条件并在真实 tick 上锁存 2 s；自由初始摆放不通过；
- 真实 mink IK 六关键姿态、私有配置不改活动场景、碰撞通过、22 s 级计划、
  60×16/50 ms/左臂固定/迟到或未来 observation 拒绝、超 30 s 计划拒绝；
- NativeWorker 实测 prepare/start/teacher chunk/幂等/未来时间/未保留 capture/
  推理模式禁用 teacher，以及正常关闭。

未执行：新 IDL 真 CDR/真实 ROS 服务查询（监督者）、EGL 渲染验收、完整抓放
rollout、正式数据、ACT 训练和 20 种子评估。
