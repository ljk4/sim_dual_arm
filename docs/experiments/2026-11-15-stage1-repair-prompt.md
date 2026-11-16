# 第一阶段执行提示词

将下面代码块完整复制到项目中的编码 AI，或让它读取本文件并执行代码块内容。本提示词只授权 P1；当前文档的创建不代表已经开始实施。

```text
在 /home/ljk/work/sim_dual_arm 实施修复任务书的 P1，只完成两项确定性评价错误修复并验证，然后停止，不自动继续后续阶段。

先读：
1. /home/ljk/work/sim_dual_arm/RULES.md
2. /home/ljk/work/sim_dual_arm/docs/experiments/2026-11-15-inference-repair-taskbook.md
3. /home/ljk/work/sim_dual_arm/var/_system/diagnostics/2026-11-15-inference-failure-analysis.md
4. /home/ljk/work/sim_dual_arm/var/_system/diagnostics/2026-11-15-code-and-artifact-audit.md

以当前代码和原始证据为准。若文件漂移，先核实目标函数及调用方，不能按旧行号盲改。

本阶段范围：
A. B01：runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/offline/evaluation_metrics.py::_apply_task 把 stable_elapsed_s>=2.0 当作所有任务的成功条件，导致合法的 Marvin 1 秒稳定成功被判失败。
B. B02：simulation/mujoco/src/robot_backend_sim/tasks/stack/evaluator.py::_support_xy_error 把世界偏移投影到目标局部系时用了 R，应该用 R.T。

约束：
- 不执行 Git 写操作，包括 add、commit、reset、checkout、stash、clean、分支和 worktree。只读检查使用 GIT_OPTIONAL_LOCKS=0。
- 改文件前记录用户现有修改，将本阶段原文件按相对路径备份到 /tmp/sim_dual_arm-repair-P1-<唯一标识>/original/，列出新文件。不要新增哈希/校验框架。通过后保留修复，给出精确复原方法；临时实验配置在结束时恢复。
- 不训练、不采集、不修改权重或数据集，不运行新的批量闭环；不改成功阈值、限位、摩擦、控制增益、Follower、准入、融合和训练配置。
- 不修改 ROS/protobuf/schema，不按 Panda/Marvin 或 task_type 在通用代码中选择常数，不增加兼容分支、纯包装层或配置快照副本。
- 不删除或覆盖原 240 次结果和既有诊断证据。不实施 B03–B08 或顺手清理其他系统。
- 如需容器测试，先核实当前环境：已有容器默认停止，源码挂载按端隔离，tests 不一定在容器可见；不要假装某条 pytest 命令可以直接运行。通过 env.sh 使用现有环境，临时测试视图放 /tmp，确保导入的是当前修改源码，并在结束后清理、恢复原容器状态。不启动/重建/拉取 learning 镜像；当前 learning:current 已由原容器恢复，不能用 latest 再覆盖。环境修复若确有必要，按任务书作为独立子项报告，不混入评价器补丁。

执行步骤：
1. 检查工作区和两条调用链，确认原失败能由最小测试复现。先运行相关现有测试，记录原有失败，避免把既有问题算作本次回归。
2. 修 B01。采用任务评价器拥有稳定窗口判定、离线汇总消费冻结成功事实的最小设计：去掉全局硬编码 2 秒的重复门槛，保留现有正式 Policy/非测试来源、任务确实被评价、终态/条件一致、时间关系以及配置/计划身份等完整性约束。
   不把 2.0 换成 1.0；不从当前 YAML 重建旧录制的时长要求；不只凭 outcome=succeeded 字符串或 conditions_passed 就判成功。在线任务仍按各自已有配置执行持续时间门槛，离线不要再维护一份。
3. 修 B02。实现 R.T @ (source.center-target.center)，保持原支撑边界和 margin；独立 upright 条件保持不变，不改为六面对称或圆形支撑区域。
4. 优先在现有测试文件补充有意义的回归：
   - 完整合法、非 test-only 的 Policy 交接 SUCCEEDED 事实，稳定 1 秒应被离线识别为成功；同一通用逻辑无需识别任务名。
   - 两任务在线评价仍在各自稳定窗口不足时 RUNNING，达到原窗口才 SUCCEEDED。
   - 缺少真实任务事实、只有成功字符串、RUNNING/FAILED、条件未通过及既有 teacher/test-only 负例继续拒绝。
   - 目标 yaw=10°，目标局部偏移 (0.027,0.027,0.05)，目标半尺寸 0.03、margin=0.002，正确 XY over-edge 为 (-0.001,-0.001) m。旧实现有一轴约 +0.006606 m。
   - 正负 yaw、identity、边界内/外都正确，90° 翻转仍被原倾角条件拒绝。用独立几何预期测试，不能仅检查实现写了转置。
   相关文件：tests/unit/test_stack_evaluation.py、test_stack_task.py、test_handover_task.py、test_task_result_sidecar.py。需要新增测试模块时放 tests/unit，不复制一套私有评分器。
5. 运行有变更覆盖意义的测试与相关回归；按项目风格检查改动文件。禁止删负例、降低断言、添加 importorskip 或伪造成功来通过。
6. 如本机原始证据仍在，做只读历史复核：
   - closure120-panda-b16-ep-0002、closure120-panda-b23-ep-0001 为原成功代表。
   - closure120-panda-b14-ep-0003 的 XY 项应由失败变通过，但其余失败项仍存在，不得宣称任务成功。
   - 原 Marvin 120 局均未完成任务，时长评分修复不能把它们改标为成功。
   只重算相关事实，不改写原 task_result、closure audit 或 MCAP；不用为了 P1 再运行完整 240 局仿真。
7. 更新任务书 P1 状态与结果；阶段摘要写清实现选择、测试命令/环境/通过失败跳过数、历史复核边界、实际修改与新建文件、备份位置和恢复方法。运行结果使用既有 diagnostics 目录，不新增产物类型。相关原始数据不存在或环境不能验证时，明确哪项未做，不能记为完全验收通过。

验收目标是“评分逻辑和几何计算正确”，不是“成功率提高”。正常物理失败应继续失败；P1 不保证解决 Panda 停滞或 Marvin 越界。

完成后先报告两个修复的结果与证据、剩余限制，保留可审查的补丁。不要自动进入 P2，不要求提交 Git，也不要只交计划而不实施本阶段代码和测试。
```
