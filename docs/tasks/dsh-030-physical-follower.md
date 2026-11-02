# DSH-030：受限跟随接入同一原生物理执行器

DSH-029数学层已验收合入；API已核对。首轮40分钟，最多一次集中返工。
用户已批准受限跟随并先对比；本项不采堆叠试采、不训练、不改场景/硬限位。

## 允许范围和设计

允许改 contracts/python/src/robot_contracts/{follower,follower_math}.py（小型派生属性/共享采样、既有验证不能减弱）、
components/backend_sim/src/robot_backend_sim/{follower_plan,control_state,control,motion,stream_schedule}.py，
必要时新增一个明确的chunk_types.py定义二者的类型联合；新增tests/unit/test_follower_control_state.py、
tests/integration/test_follower_native.py、docs/native-follower.md。
不要修改ROS IDL、protocol.py、worker/node/runtime、teacher、训练、Docker或主工作树，不提交。
Ruckig包路径：`/home/ljk/work/sim_dual_arm/.build/follower-probe-venv/lib/python3.12/site-packages`。
执行测试前追加该路径到PYTHONPATH，PROJECT_PYTHON仍用任务头的native-dev-venv，不修改venv或联网安装。

复用同一个NativeMotion/NativeController/StreamSchedule/lease/history，禁止复制第二套物理循环。
保留严格PreparedChunk和全部旧接口行为。增加明确PreparedChunk | PreparedFollowerPlan联合，
通过具体类型判定；禁止让follower继承严格类型绕过原契约，也不使用无验证的鸭子对象。
现有接口需要settle_at_end时，可在PreparedFollowerPlan加派生只读property返回is_final。
不要为方便添加语义含混的final_velocities别名，使用follower已有actual/restricted端速名。

PreparedControlCommand新增明确kind=FOLLOWER，MOTION/CHUNK原有fingerprint字节稳定。
FOLLOWER指纹覆盖算法版本、全部原始window/anchor、真实系数、实际限制参数、raw/restricted端速、
偏差证据与终态延长；不能只看末目标或把它伪装为旧CHUNK payload。
本项内部packet.request可为None（公共Follower服务下一项接），不生成严格AcceptedReference。
保留PreparedControlCommand中既有NativeRequest行为，不改026的严格转换。

## 执行与时钟

首块controller.submit，后继queue_chunk，都是同一真实物理线程。新增模式必须同样受
lease/context/profile、accept_before、观测年龄、minLead、C1、硬限位、历史容量及停止期限保护。
同一stream中严格/受限模式混用应拒绝；新stream可选择模式。只保留active+1pending。
next_anchor从实际plan末端p/v生成，不能从raw目标或measured状态重基准。
原始60×16和观测时间完整保留；非final固定window接续，不逐块停稳。

终块真实时域须在2ms物理网格上，并到达原最后目标、零端速，然后等真实测量稳定/接触完成。
优先给prepare_follower_plan增加明确可选duration_grid_ns参数：默认连续以保留029行为；
物理调用传2_000_000，让Ruckig InputParameter.duration_discretization=Discrete，
Ruckig(count, delta_time=grid_ns/1e9)在原min_duration上生成离散总时长。
监督者已实测0.15.3：连续.2658921197s的例子离散为.266s（133步），p/v终值正确。
须严格验证grid类型/正值、非final固定handoff本身在网格上，以及最大3s/配置尾段限制。
不手改ruckig_duration元数据，不以延长参考冒充原始预测未变时域，也不放宽C1/grid容差。

只改变目标参考层；物理关节、力/接触、完成、刹车、质量/墙钟期限原实现沿用。
Follower终态CONTACT仍保留raw闭合目标与真实开度/双指接触，不能靠位置到达伪造抓取。

## 验收

- 固定初始seed、完全相同raw source构建严格首窗反例与follower；前者明确拒绝，后者真实执行。
- 至少3连续follower块，活动/待定/退休/整体终态分别核对；每tick真实apply->step->measure，
  独立按保存系数重建实际reference，保留raw、reference、measure和偏差JSON摘要。
- final尾段确有延长且在2ms网格，到达raw末目标后实测稳定；不得把non-final前缀终点当最终目标。
- 迟预测/underrun、超偏差/raw越界、错context/reset、lease过期、修改retry、模式混用、
  同指纹重试、夹爪CONTACT均有行为验证；失败必须真实刹停并给明确原因，不能只测异常字符串。
- 新指纹区分follower参数/raw未选尾部，旧MOTION/CHUNK固定fixture指纹与原实现逐字一致。
- 跟随math和旧全部unit/native物理回归通过，Ruff/diffcheck通过；ROS尚未接入要如实记录。

将对比产物及实际检查写.build/reports/dsh-030/report.md；不声称完成堆叠或学习，完成后退出。

## 已确认的数值接线问题（分派前补充）

029按共享1e-9数值容差验证系数；PandaIO.apply_targets对位置逐值严格检查。
已证实近闭合例的实际系数末端=-2.849e-11m、近全开末端=.08000000002849m，
虽在共享数学容差内，直接下发会被严格I/O拒绝。必须在本项真实边界测试解决，不能忽略。
可以在follower共享采样/参考适配中定义显式、可独立重建的浮点边界规范化：
仅处理共享LIMIT_ATOL/LIMIT_RTOL内的舍入误差，实际下发值始终在硬边界内；
保留原系数采样值、实际应用参考及修正量，采集/推理统一、共享审计可重建。
也可用经证明的系数导出改进消除此误差。不得修改raw预测，不放宽PandaIO硬限位，
不得静默全局clip、修改旧严格模式或把有意义的预测误差当数值误差。
若选择规范化，需明确记录位置/速度/加速度语义和适用边界，正反例及超容差拒绝实测。
