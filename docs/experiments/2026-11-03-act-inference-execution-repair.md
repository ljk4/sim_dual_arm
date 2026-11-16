# ACT 推理停滞、执行误停与首次堆叠成功的修复经验

- **收录日期**：2026-11-03
- **关联阶段**：[DSH-045](../history/README.md#dsh-045)～[DSH-051](../history/README.md#dsh-051)（综合验收见 [DSH-048](../history/README.md#dsh-048)）

---

## 1. 核心结论与适用范围

“ACT 推理时机械臂几乎静止”并非因为模型只输出恒定动作。离线诊断表明，模型输出的完整 60 步（3.0 s）预测块包含显著运动，但过短的执行窗口（2 点 / 100 ms）未能推进远端动作；将执行窗口扩大后，又依次暴露出跨块预测跳变、复位后初始非稳态、加权融合浮点舍入、Ruckig 飞秒级制动前缀误拒绝，以及静止速度分类容差不一致等问题。

在**不追加任何训练**的前提下，通过调整流式执行配方并修复上述实现缺陷，原 20K 步模型在种子 `30600` 下完成了一次真实物理堆叠（耗时 **25.85 s**，释放并退开后稳定支撑 **2.0 s**，全程未引入物体真值、Teacher 或 IK 补救）。

> **适用边界**：该结果属于针对种子 `30600` 的**同场景修复回归**，验证了推理与控制执行链路的修复有效性，不能直接等同于跨独立新种子的泛化成功率。同时，右腕相机视角在接近阶段存在 1.85 s 底块遮挡，图像质量门禁（`quality_qualified`）仍记为未通过。

---

## 2. 实验条件与验收结果

### 2.1 实验配置

| 维度 | 配置详情 |
| --- | --- |
| 仿真与控制 | MuJoCo 双 Panda、ROS 2 Jazzy、Ruckig 0.15.3 有界跟随规划 |
| 策略模型 | LeRobot ACT（batch 16、20,000 步训练；本次修复新增训练 0 步） |
| 观测与动作 | 三路 320×240 RGB + 16 维实测关节/夹爪位置 → 60×16 动作块（步长 50 ms，共 3.0 s） |
| 最终执行配方 | 10 点（0.5 s）执行窗口、绝对仿真时间指数融合（系数 0.01）、任务前 2.0 s 稳定等待 |
| 源配置文件 | [policy_rollout.yaml](../../shared/scenarios/panda_stack/configs/runtime/policy_rollout.yaml)、[backend_stack_act_chunked.yaml](../../shared/scenarios/panda_stack/configs/simulation/backend/backend_stack_act_chunked.yaml)、[panda_follower_v1.yaml](../../shared/scenarios/panda_stack/profiles/control/panda_follower_v1.yaml)、[policy_temporal.yaml](../../shared/scenarios/panda_stack/configs/learning/policy_temporal.yaml) |
| 模型包路径 | `var/learning/experiments/act-static-b16-20k-20261103/bundle-best-20k`（摘要 `c1ed8e8f...278e6`） |

### 2.2 回归结果对照

| 验收项 | 实际结果 |
| --- | --- |
| 修复前失败身份 | `evaluation:30600:run-1793710201614369937` |
| 修复后回归身份 | `evaluation:30600:run-1793711210049299089`（独立台账） |
| 物理任务结果 | `SUCCEEDED`（25.85 s 完成，稳定支撑 2.0 s；54 次预测、53 块实际执行） |
| 释放与退开证据 | 夹爪实测开度约 `0.0800 m`，双指零接触，TCP 距源方块约 `0.1523 m` |
| 录制与审计 | MCAP 录制完整；文件哈希、RGB 绑定与原始/准入/融合/传输动作逐块校验通过 |
| 图像质量门禁 | `quality_qualified=false`（右腕视角底块遮挡 1.85 s，超出 0.35 s 门限） |

---

## 3. 关键问题诊断与修复方法

### 3.1 执行窗口过短导致动作推进不足

- **现象与根因**：原配方每次仅截取预测块前 2 点（100 ms）。对 81 次在线预测的分析显示，完整 3.0 s 动作块的右臂最大关节位移中位数达 **1.86 rad**，而前 100 ms 仅约 **0.066 rad**。受限于首段零初速加速约束，2 点窗口生成的参考位移仅约 `0.0039 rad`，导致机械臂在重规划循环中原地微动。
- **修复方法**：将跟随窗口 `target_count` 扩大至 **10 点（0.5 s）**，使 Ruckig 规划器获得足够的前瞻位移。
- **经验总结**：排查策略停滞时，必须同时对比**完整 60 点预测、截取窗口、样条参考与实测轨迹**，切勿仅凭视频表现或首点位移断定模型未学会运动。

### 3.2 跨块预测跳变引发跟随偏差拒绝

- **现象与根因**：扩大执行窗口后，相邻两次预测在切换时刻出现跳变（如种子 `30401` 第 8 次预测相对当前参考的关节偏差达 `0.229 rad`，超出当时的 `0.15 rad` 预算）。单纯缩放窗口长度无法消除相邻预测块的不一致性。
- **修复方法**：在 Policy 服务端（[`temporal_ensemble.py`](../../learning/lerobot/src/dual_arm_learning/temporal_ensemble.py)）对已通过准入的历史预测按**绝对仿真时刻**进行指数加权融合。设当前观测时刻为 $t$、历史观测时刻为 $s$、步长 $P = 50\text{ ms}$，仅当未来目标时刻对齐（$t + (i+1)P = s + (j+1)P$）时按权重 $\exp(0.01 \cdot (t - s) / P)$ 融合并归一化。历史缓冲按会话隔离，仅纳入已成功提交的有效预测。
- **经验总结**：异步推理的查询间隔未必恒等于单步周期 $P$，时间融合必须严格按**绝对目标仿真时间戳**对齐，不能简单按数组下标对齐。

### 3.3 复位后瞬态非稳态与准备竞态

- **现象与根因**：
  1. **服务竞态**：复位刚完成时立即调用 `PrepareTask` 偶发返回 `TASK_NOT_QUIESCENT`。
  2. **分布偏移**：复位后的 PD 阻尼收敛过程使极早期关节状态偏离示教训练集的稳定待机分布，导致首块预测偏差过大。
- **修复方法**：
  1. 针对 `TASK_NOT_QUIESCENT` 在总墙钟期限内进行有界幂等重试，其他错误立即中止。
  2. 在 `runtime` 配置中增加任务开始前的 **2.0 s 仿真时间稳定等待**，待机械臂完全静置后再采样首帧观测并启动任务计时。

### 3.4 策略目标与平滑参考的偏差预算设定

- **现象与根因**：Ruckig 生成的平滑参考受限于最大速度与最大加速度，无法瞬时跳变到策略目标。原 `0.15 rad / 0.02 m` 的目标-参考偏差预算在正常加速与抓取闭合阶段频繁触发拒绝（如夹爪偏差达 `0.0285 m`）。
- **修复方法**：将仿真实验的目标-参考允许偏差预算调整为 **关节 `0.30 rad` / 夹爪 `0.04 m`**（[`panda_follower_v1.yaml`](../../shared/scenarios/panda_stack/profiles/control/panda_follower_v1.yaml)），同时保持机器人物理硬限位、速度/加速度上限与实测跟踪容差不变。
- **经验总结**：严格区分**动作准入修正上限**（夹爪 `0.03 m`）、**策略目标相对平滑参考的允许偏差**（`0.30 rad / 0.04 m`）与**平滑参考相对实测反馈的跟踪容差**，三者物理意义不同，不可混用。

### 3.5 时间融合加权平均的浮点舍入越界

- **现象与根因**：种子 `30402` 中参与融合的所有历史夹爪开度均在合法范围 `[0.0, 0.08]` 内，但浮点加权平均后产生了 `0.08000000000000002 m`，超出 `0.08 m` 严格上限导致整块被拒。
- **修复方法**：在 [`temporal_ensemble.py`](../../learning/lerobot/src/dual_arm_learning/temporal_ensemble.py) 中将每个时间步、每个自由度的加权融合结果钳制在**当次实际参与融合样本的最小值与最大值（凸包）之间**，消除 IEEE 754 浮点尾数舍入溢出（单测见 [`test_temporal_ensemble.py`](../../tests/unit/test_temporal_ensemble.py)）。

### 3.6 Ruckig 飞秒级制动前缀误拒绝

- **现象与根因**：种子 `30502` 夹持方块后触发 `INVALID_RUCKIG_EXPORT`。回放发现起始夹爪速度为 `0.04000000000000001 m/s`（比上限 `0.04 m/s` 多出 `1e-17` 浮点误差），导致 Ruckig 生成了时长约 $2.20 \times 10^{-14}\text{ s}$（22 飞秒）的微制动前缀，被旧版导出器当作非零制动段拒绝。
- **修复方法**：在 [`ruckig_export.py`](../../simulation/mujoco/src/robot_backend_sim/motion/ruckig_export.py) 中，仅当制动前缀在纳秒量化下为 **`0 ns`** 且状态跳变在既有连续性容差内时将其安全折叠；任何可表示为非零纳秒的真实制动段仍严格拒绝。

### 3.7 静止速度容差不一致导致正常夹持被判为跟踪超时

- **现象与根因**：种子 `30600` 在成功夹持源方块（双指接触力各约 `1 N`、指速接近 0）后触发 `TRACKING_TIMEOUT`。原因是闭合参考速度为 $+5.339 \times 10^{-14}\text{ m/s}$：闭合接触判定要求参考速度严格 `<= 0`，而接触卸载逻辑将 `<= 1e-9 m/s` 视为“未张开”，导致该数值静止状态落入两不管地带，跟踪误差累计满 2.0 s 后超时停车。
- **修复方法**：在 [`gripper_contact.py`](../../simulation/mujoco/src/robot_backend_sim/physics/gripper_contact.py) 与 [`motion_controller.py`](../../simulation/mujoco/src/robot_backend_sim/motion/motion_controller.py) 中统一复用 `REFERENCE_VELOCITY_ATOL_M_S = 1e-9` 作为零速度分类容差（单测见 [`test_contact_unload.py`](../../tests/unit/test_contact_unload.py)）。

---

## 4. 标准化排查清单

遇到闭环执行异常时，建议按以下顺序排查：

1. **核对运行身份**：确认模型包摘要、Profile 摘要与挂载源码版本，保留原始失败目录。
2. **定位首发故障事件**：按时间戳查找后端与运行端日志中的首个拒绝或终止原因（如后端的 `TRACKING_TIMEOUT` 常表现为运行端后续报 `STREAM_ANCHOR_UNAVAILABLE`）。
3. **逐层比对动作数据**：在同一观测时间戳下依次核对 `raw_act`（原始预测）→ `admitted`（准入值）→ `execution_targets`（时间融合目标）→ `FollowerReference`（Ruckig 样条参考）→ `ExecutionFeedback`（实测反馈）。
4. **最小化离线复现**：从 MCAP 提取故障时刻的 16 维请求与 Anchor 状态进行单元测试重建，修复后再通过容器公共入口执行完整物理闭环回归。

目录导航：[实践索引](README.md) · [Agent 指导与规范](../../AGENTS.md) · [项目说明](../../README.md)
