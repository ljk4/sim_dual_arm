# P2 补修提示词

本轮补修已完成并验证，结果见[任务书 §7.8](2026-11-15-inference-repair-taskbook.md#78-p2-补修结果2026-11-15)。下方保留具体实施范围供复核或在其他工作区执行；当前工作区无需重复修改，P3 尚未启动。

```text
在 /home/ljk/work/sim_dual_arm 核对 P2 补修。先读取 RULES.md、docs/experiments/2026-11-15-inference-repair-taskbook.md 的 §7.7–7.8，以及原 P2 提示词。若已具备 §7.8 的修复，保留现状，仅复核；尚未修复的工作区按下面实施。不要进入 P3，不改模型、数据、准入、Follower、融合、物理参数或任务阈值，不训练/采集/批量推理，不执行 Git 写操作。

保留现有 P1/P2 补丁，记录已有修改，按相对路径备份本轮要改的文件到 /tmp/sim_dual_arm-repair-P2-followup-<唯一标识>/original/。恢复时只处理本轮文件，不覆盖用户后续修改。复用现有 simulation/runtime 镜像，不 build/pull，不启动 learning；临时测试视图放 /tmp 并清理，结束恢复容器初态。

R2-01，必须修复：
tasks/handover/evaluator.py::_update_sequence 在首次 holding 时保存 _sequence_references，失去有效抓持时只清计时，不清候选相对位姿；_remember_sequence_reference 又在已有条目时直接返回。右手尚未完成共持证明前短暂断触，再换一个有效相对位置重新抓稳，会永远被第一次参考锁死。

最小复现可加入现有 tests/unit/test_handover_task.py，复用它已有的 helper（replace 已导入）：

def test_candidate_reference_recovers_after_unestablished_right_regrasp():
    runtime = _runtime(())
    time_ns = _advance_policy_left_phase(runtime)

    def feed(offset_ns, observation):
        t = time_ns + offset_ns
        return runtime.tick(replace(observation, sim_time_ns=t), t)

    feed(100_000_000, _dual_airborne(0))
    feed(200_000_000, _left_airborne(0))
    for index in range(3, 16):
        observation = _airborne(
            0, left_open=CLOSE, left_holding=True,
            right_open=CLOSE, right_holding=True,
            right_tcp_offset=(0.0, -0.05, 0.035),
        )
        state = feed(index * 100_000_000, observation)
    assert state.phase == "RUNNING", state.metrics
    assert state.metrics["left_holding"] is True
    assert state.metrics["right_holding"] is True
    assert state.metrics["handover_sequence"]["dual_airborne_stable"] is True

该回归已在当前真实模块的 simulation 环境运行，结果 1 failed。抓持断触 0.1 秒，发生在右手共持证明建立前；之后连续稳定超过原持稳窗口，仍 dual_airborne_stable=false、dual_hold_elapsed_s=0。新旧参考差 10 mm，原相对位置容差 5 mm。

实现要求：
- 明确“尚未建立阶段证明的候选抓持”与“已经建立的持物/交接证明”的生命周期。在候选失去有效抓持、新尝试开始时，允许重新建立该手参考并从新时间起计时；只改清楚的受支持路径，不引入重试框架。
- 稳定窗口内的参考必须固定，不能每 tick 跟随当前姿态，否则持续滑移会被掩盖。条件中断不能拼接累计时间。
- 不清掉另一只手已经有效的证明来掩盖错误；已建立持物后的丢失仍使用原 hold_loss_tolerance，外部支撑、释放、顺序和跨 epoch 清理继续生效。
- 同类候选参考生命周期要检查左手、右手及 Teacher/Policy 共用路径，避免只为这一个 helper 的参数打补丁。不得提高位置容差、放宽接触力或持稳时长。

补充针对性回归：
1. 上述右手候选断触后异位重抓，重新持续满足窗口应通过共持阶段。
2. 候选初次出现但尚未完成证明时，失去接触/重新建立后按新窗口计时，不能继承旧时长；检查适用的左手候选路径。
3. 持续滑移或反复抖动不得因为重新取参考而凑够稳定窗口；瞬时不足窗口仍不能通过。
4. 已建立证明后的长时持物丢失仍失败；短时允许恢复的路径、外部借力拒绝和跨 epoch 清理不退化。
5. 最终 1 秒稳定窗口、完整序列成功正例、直接入箱拒绝及 P1 投影回归保留。

R2-02，修正本轮测试要求：
移除 tests/unit/test_handover_contact_evidence.py:18 和 test_stack_contact_evidence.py:17 两处本次新增的 pytest.importorskip("mujoco")。这些是 P2 必需物理验证，缺依赖应显式失败并说明环境问题。直接在项目正确 simulation 环境运行，不新增另一种跳过条件，不扩大到历史其他测试的清理。

验证与交付：
- 先记录新回归在修改前失败，再实现最小修复，运行新回归与 P2 相关测试、必要的 P1 回归。准确报告通过/失败/跳过，既有 9 项宽子集问题独立列出，不能混作新失败或靠跳过掩盖。
- 只更新任务书的一份补修结果及状态、必要索引。说明候选参考何时失效/重新建立、窗口内如何固定、修改文件、测试命令/环境/数量、备份和复原、临时视图与容器清理。
- 没有完整闭环或真机证据就明确未验证；不声称提高成功率。补修通过后才把 P2 改为已完成。停止在 P2，不启动 P3，不提交 Git。
```
