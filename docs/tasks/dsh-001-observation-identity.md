# DSH-001：固定观测 ID 对应的样本引用

状态：已执行并通过独立验收，见 [试验结果](dsh-001-result.md)。任务类型：阶段 2R 的首个委派试验。下文保留实际执行时的任务要求；监督者已在运行快照中补充真实工作目录、Python/Ruff 可执行路径、报告目录和时间限制。

## 目标与基线

在基线 `c2c3e939116f9926f37a256b3b315cd80ce7fe53` 上修复 ObservationBuffer：同一运行上下文中的一个 observation ID，一经分配，其 state 与相机样本引用保持不变。实现一个局部修复并提交测试证据，保持代码易读。

监督者应先核对当前 HEAD 与任务基线。若基线不同、工作区已有未说明改动或运行路径不明确，报告阻碍，停止修改；不要自行重置分支。

## 阅读范围与允许修改

允许修改且只允许修改：

- `components/runtime/ros/dual_arm_runtime/dual_arm_runtime/observation.py`
- `tests/unit/test_observation.py`

必要阅读：上述文件、`tests/unit/conftest.py`、`contracts/python/src/robot_contracts/models.py`；为理解调用方，可只读 `node.py` 中 `_publish_observation` 和 `_wait_observation`。根 `pyproject.toml` 定义检查入口。

日志等可重建产物仅写入监督者指定目录；测试工具的常规缓存除外。不修改 ROS 消息、node.py、Docker、依赖、profiles、配置、任务书或测试发现规则。不执行 Git 提交、重置、推送或分支操作；不启动其他代理。本任务不需要网络、Docker、ROS 或包安装。

## 已知缺陷

使用同一 RunContext，配置 `max_skew_ns=20`、`max_age_ns=100`：

1. 添加 state(seq=1, stamp=100) 和 front(seq=1, stamp=90)。
2. `assemble(now_ns=100)` 返回 observation_id=1，front.frame_seq=1。
3. 添加迟到但满足因果约束的 front(seq=2, stamp=95)。
4. `assemble(now_ns=105)` 仍返回 observation_id=1，但 front.frame_seq 变成 2。

调用方仅在 ID 变化时发布和记录索引，却每次更新最新观测。相同 ID 对应不同图像会破坏后续推理与记录的一致性。

## 明确的预期行为

- 对当前 state 第一次成功组成观测时分配 ID，并固定全部样本引用；相同 state 的后续组装返回相同 ID 和相同内容。
- 迟到的合法相机帧可以进入缓冲，供后续 state 选择，但不得改写已分配观测的引用。
- 新 state 第一次成功组装时获得递增 ID；仍选择时间不晚于该 state 且满足 max_skew 的最新相机帧。
- 组装失败时不消耗 observation ID；首次缺少相机、稍后相机到达时仍能正常生成观测。
- 每次 assemble 仍检查当前 state 的新鲜度；存在已组装观测不能使过期或未来时间的 state 绕过检查。
- RunContext 变化清空相关状态与引用，ID 从该上下文重新计数；旧上下文的样本仍被拒绝。
- 返回对象的引用集合不能通过调用方修改而污染后续返回结果。可采用适合现有只读调用方式的不可变表示或防御性复制，并解释选择。
- 保持有界存储、既有锁保护、顺序/时间检查和 dropped_samples 行为。不引入整段观测历史或通用缓存框架。
- 保持现有公共方法参数和调用方使用方式兼容。当前 `frames` 的名称查找、遍历、values 操作须继续可用。

## 实施与验证

先补充至少一个暴露“迟到帧改变同一 ID 内容”的行为回归测试，在未修复实现上运行并记录失败；随后实现局部修复，再完成其余关键边界验证。测试应断言可观察结果，不依赖新增私有字段名或特定实现结构。

应覆盖：迟到帧不改旧观测、新 state 使用合适的新帧、缓存不绕过过期检查、上下文切换、首次缺帧后恢复，以及调用方无法污染已分配引用。可复用已有测试覆盖，避免重复案例。

监督者提供 `PROJECT_PYTHON` 和 `PROJECT_RUFF` 的实际可执行路径。在试验 worktree 根目录依次执行：

```bash
"$PROJECT_PYTHON" -m pytest -q tests/unit/test_observation.py
"$PROJECT_PYTHON" -m pytest -q
"$PROJECT_RUFF" check .
"$PROJECT_RUFF" format --check .
git diff --check
git status --short
```

当前主工作区 `.build/dev-venv/bin/` 可作为监督者评估的工具来源，但新 worktree 不会自动包含该目录。不要自行安装缺失工具，也不要从主工作区运行测试后将结果归于候选代码。实际工作目录与源码导入位置必须对应试验 worktree。

## 停止条件

以下情况立即报告，不扩大范围：需要修改白名单外文件；依赖或工具不可用；发现必须改变协议/其他组件才可继续；无法说明线程与缓存行为；达到监督者规定的时限。首轮建议 15 分钟，返工由监督者按具体审查意见另行下发一次。

## 最终交付格式

用简短中文返回：

1. 实现摘要，以及观测引用采用的所有权/不可变策略。
2. 修改文件清单。
3. 旧代码失败的回归测试及失败原因，修复后的实际测试命令和结果。
4. 未解决项、未执行验证；若有阻碍，明确写出。

代码保留在该 worktree 的未提交差异中。自测通过后停止，等待监督者独立审查；不要继续处理 R1、R3 或 runtime 重构。
