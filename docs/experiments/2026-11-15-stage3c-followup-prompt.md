# P3-C 补修提示词：隔离 Teacher 与修正时间验证

状态：已补修（见任务书 §10.9）。依据为[任务书 §10.8](2026-11-15-inference-repair-taskbook.md#108-p3-c-独立复核待补修2026-11-15)及[独立红测](../../var/_system/diagnostics/p3c-independent-review-runtime.log)。本阶段不扩展臂轴、不开展新闭环。

---

请在 `/home/ljk/work/sim_dual_arm` 亲自完成 P3-C 补修。先读取根 RULES.md、适用 AGENTS.md，以及修复任务书 §10，尤其 §10.8。保留 P1/P2/P3-B 和 P3-C 已有有效补丁；不要重做整个 P3-C。

## 1. 必须修复的范围错误

`runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/chunk_stream.py` 的 `ContinuousStreamRunner` 同时承载 Policy 和 Teacher。当前无条件初始化 `retiming_axes` 并在 `_request` 改写目标，导致 Teacher 也会执行本阶段的 Policy 适配。

独立复现使用真实 `TeacherTargetSource`、`TeacherStreamRunner` 和 Panda `teacher_collect.yaml`。继承右夹爪 `q0=0.022205447806916776 m, v0=0.04 m/s`，原始目标 0.079865 m；首个选中点在交接后 0.05 s，当前会被改成 0.024205447806916774 m。实际结果 **1 failed、23 passed**，失败断言为 Teacher 的完整原目标应逐值不变。尚未证明历史正式采集发生过这种触发，不改写或重标旧数据。

按既有动作来源隔离适配：只有 Policy 可使用当前 P3-C 夹爪目标适配，Teacher 保持 P3-C 前的命令行为。优先使用已有、已验证的 `recipe.action_source` 或把机制放在 Policy 入口，不按任务名、日志字符串或 chunk_digest 猜来源；不添加第二份来源字段、默认开关、纯转发包装或新策略框架。

Teacher 发生原本不可执行的请求时，保留原 backend 拒绝，不把它自动改成新示教目标。既有 Teacher chunk identity/digest、目标、门控、录制标签和来源语义必须保持一致。

## 2. 修正时间验证

`tests/unit/test_transition_retiming.py::test_retimes_a_reverse_switch_from_a_closing_velocity` 当前以 `(index + 21) * 0.05` 调用 `reachable_bounds`；这把源索引对应时间当成交接后的时间。实际 `window.targets` 的时间为 0.05、0.10、…、1.00 s，首点不是 1.05 s。

改用实际选窗的交接相对时间逐点验证，并保留独立手算的首点/早期点边界，不能只验证输出落在一个过宽区间内。生产时间轴和选窗规则无需因此改动。

## 3. 说明算法真实含义

当前算法保持时间戳，改变选中位置值，并把窗口内部路径替换为朝末点的直线再逐点裁入宽松包络。这是显式执行目标整形；原始模型输出保留，并不意味着执行目标不变，也不能宣称保留了原窗口的先开后合时序。

修正文档和必要注释：

- 原始目标偏差与适配执行目标偏差分别报告；对适配目标满足预算不能写成对原始目标满足预算。
- 单点位于可达包络内，只是必要条件，不保证整条曲线存在。后续真实 Ruckig、连续限位、制动和 Follower 偏差检查必须完整保留，失败继续传递。
- 已有 8 次闭环中，Panda 为 4 次 right_joint2 拒绝、1 次 horizon 拒绝、1 次成功且零适配；不泛称 6 次都由臂轴拒绝。Panda 使用 window20，该结果不能替代默认 target_count=5 的闭环验证。
- 不改变原录制结果、不将 Panda 零适配成功归因于本机制，不把补修通过记成 P3 总体验收通过。

本阶段不重设计整形算法，也不把相同方法扩展到臂轴；模型相位不一致继续作为后续 P4/P5 输入。

## 4. 验证与范围

先加回归，在当前代码上复现 Teacher 范围失败，再修复。至少覆盖：

1. Teacher 配方 + 真实 Teacher runner/source 路径，使用上述能够触发 Policy 适配的输入，完整 60 点与原预测一致，无适配标记/事件，chunk identity 不变。
2. 同类 Policy 输入仍触发原适配，目标和事件数值保持；正常 Policy/Teacher 请求均保持不变。
3. 反向速度测试使用正确时间轴，首点和早期点在相应包络内。
4. 两条原始不可达请求仍拒绝；已有显式适配请求的真实 Ruckig 回归保持；原始与执行目标偏差继续区分。
5. backend 拒绝不被吞掉，不改变成功判据、限位、加速度、速度、偏差预算或继承 anchor。

扩展既有测试文件，运行相关 runtime、纯数学与真实 Ruckig 测试即可；不无差别重跑全套，不删除负例、不增加跳过、不放宽断言。

本次授权修改上述实现、相关回归、必要文档及只读证据复核。**新闭环预算为零**；原 8 次预算已用完。不启动模型训练、不补采、不修改权重、正式数据、旧 Bundle、方案窗口或摩擦等物理配置。

## 5. 保护、环境与交付

- 不执行 Git 写操作；只读 Git 使用 `GIT_OPTIONAL_LOCKS=0`。
- 修改前按原相对路径备份到唯一 `/tmp/sim_dual_arm-repair-P3C-followup-<标识>/original/`，新文件另列；不新增源码 hash、证明清单或兼容层。
- 复用已有镜像，不 build/pull；容器管理仍走 env.sh。临时启动改动立即恢复，测试视图放 `/tmp`，确认导入当前源码，结束恢复原容器状态。
- 更新原任务书 §10.8 后的补修结果：具体行为变化、红测/绿测、实际命令与数量、未验证项、修改文件、备份/恢复和容器状态。
- 补修完成只代表消除本次范围错误和测试缺口，不自动推进 P4/P5、增加闭环或宣布成功率改善。
