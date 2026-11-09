# `sim_dual_arm` 全量架构瘦身、解耦与性能优化任务书（防套壳一步到位版）

本目录包含面向执行模型（Executor Agent）、直接从干净的 Git `HEAD` (`48b4fc9`) 起步分阶段执行的 **6 份独立重构任务书**。

针对此前重构尝试中暴露的偷懒与缺陷（如**“定义私有 `_XxxModel` 再拷回 `@dataclass` 双层套壳”**、**“保留百行手写 `__post_init__`”**、**“只改文件名不改 `Native*` 类名”**、**“用 Mixin 假拆分 `motion.py`”**、**“漏删 SciPy `chunk_plan.py` 死代码”**、**“误删 `.py` 源文件靠 `__pycache__/*.pyc` 僵尸缓存假通过测试”**），本版任务书内置了**严格的防作弊红线（Anti-Shortcut Redlines）与自动化 AST / 行数 / 导入完整性验收脚本**。

---

## 重构前准备（回退至干净的 Git HEAD）

在投喂 Phase 1 之前，请先在宿主机或容器中将工作区彻底回退到最近一次提交 `48b4fc9`（保留本 `docs/refactoring-tasks/` 目录）：

```bash
# 备份任务书 -> 硬重置工作区与清理未跟踪文件 -> 恢复任务书
cp -r docs/refactoring-tasks /tmp/refactoring-tasks-backup
git reset --hard HEAD
git clean -fd
rm -rf docs/refactoring-tasks && mv /tmp/refactoring-tasks-backup docs/refactoring-tasks
find . -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
```

---

## 六阶段重构路线图与硬性验收指标

| 阶段 | 任务书文件 | 核心目标（一步到位，不留返工） | 硬性量化红线 |
| :--- | :--- | :--- | :--- |
| **Phase 1** | [phase-01-cleanup-legacy-and-dead-splines.md](phase-01-cleanup-legacy-and-dead-splines.md) | **彻底清除历史死代码**：一次性删除 S1 Mock 链路、S2/S3 非受限样条接口、**以及仿真端残留的 SciPy `chunk_plan.py`、`reference_compiler.py`、`motion.py:start/_step_legacy`**、旧版 Schema (v2~v4) 与废弃配置 | 删除 45+ 个历史文件；全仓净减 ~19,000 行；无任何非流式控制死分支 |
| **Phase 2** | [phase-02-single-layer-pydantic-and-contracts.md](phase-02-single-layer-pydantic-and-contracts.md) | **单层 Pydantic v2 声明式模型与契约层下沉**：公开 Config 与 Contract DTO **直接继承 `StrictContractModel`**（严禁 `_XxxModel` + `@dataclass` 双层套壳！严禁保留 `__post_init__`！）；将 2,800 行样条与 Ruckig 数学下沉至 `robot_contracts/trajectory/`；松绑单测对自然语言文案的死板断言 | 全仓 `grep "class _.*Model"` 为 0；`contracts/` 顶层 $\le 1,100$ 行；10 个配置文件合计 $\le 1,680$ 行 |
| **Phase 3** | [phase-03-config-ssot-and-eliminate-native.md](phase-03-config-ssot-and-eliminate-native.md) | **配置单一数据源（SSOT）与全仓彻底去 `Native*` 正名**：控制配置收敛至 `profiles/control/`；**不仅模块去 `native_` 前缀，且 Python 类名（`StreamRequest`、`ExecutionFeedback`、`MotionController`、`ControlSession`）与 ROS IDL（`StreamCommand.msg` 等）同步正名** | 全仓源码与 ROS IDL 中 `Native*` 类名/消息名残留为 0；`target_count` 仅一处定义 |
| **Phase 4** | [phase-04-task-plugin-and-motion-composition.md](phase-04-task-plugin-and-motion-composition.md) | **任务插件解耦、运动控制器显式组合拆分与仿真瘦身**：ROS 任务接口泛化为 `TaskState.msg`；抽离 `SimTaskPlugin`（注意 `ros_conv.py` 延迟导入 ROS 消息）；用 `BrakingController` + `GripperContactTracker` **显式组合（严禁 `*Mixin`）** 将 `motion.py` 压至 $\le 550$ 行；离线检查工具移入 `tools/`；瘦身 `control.py` / `node.py` / `worker.py` | `motion_controller.py` $\le 550$ 行；无 `*Mixin` 类；`simulation/`（除 `tools/` 外）无 $> 720$ 行文件 |
| **Phase 5** | [phase-05-learning-pipeline-and-hash-unpinning.md](phase-05-learning-pipeline-and-hash-unpinning.md) | **学习端配置契约合一、取消 YAML 手写 SHA-256 锁死与推理流水线收口**：删除 `act_contract.py` 合并入 `policy_config.py`；将 YAML 静态 SHA-256 改为运行时自动计算写入 Manifest；将会话锁、RGB 解码、准入、时间融合收归 `policy_service.py` + `ChunkedPolicyPipeline` | `adapters/lerobot_act.py` $\le 200$ 行；`act_contract.py` 彻底删除；YAML 配置无需手填 `sha256` |
| **Phase 6** | [phase-06-training-speedup-and-split-monoliths.md](phase-06-training-speedup-and-split-monoliths.md) | **训练提速（`PyAV` + 默认批处理 Collate + `pin_memory` + `AMP`）与千行巨石模块/超长函数拆分**：拆分 `training.py`、`evaluate.py`、`episode_audit.py`、`episode_runner.py`；完成全仓最终验收 | 全仓非测试 Python 总行数 $\le 35,000$ 行；无任何 $> 750$ 行文件；核心模块无 $> 85$ 行函数 |

---

## 投喂给执行模型的标准提示词模板

```text
请严格按照 docs/refactoring-tasks/phase-01-cleanup-legacy-and-dead-splines.md 中的任务书要求执行重构。
特别注意遵守文件顶部的【执行模型必读指令（System Rules & Anti-Shortcut Redlines）】，只完成本阶段要求，不要跨阶段修改。完成后逐条运行本阶段末尾的量化验收命令，确保所有检查和单元测试全部通过后再汇报结果。
```
# `sim_dual_arm` 全量架构瘦身、解耦与性能优化任务书（防套壳一步到位版）

本目录包含面向执行模型（Executor Agent）、直接从干净的 Git `HEAD` (`48b4fc9`) 起步分阶段执行的 **6 份独立重构任务书**。

针对此前重构尝试中暴露的偷懒与缺陷（如**“定义私有 `_XxxModel` 再拷回 `@dataclass` 双层套壳”**、**“保留百行手写 `__post_init__`”**、**“只改文件名不改 `Native*` 类名”**、**“用 Mixin 假拆分 `motion.py`”**、**“漏删 SciPy `chunk_plan.py` 死代码”**、**“误删 `.py` 源文件靠 `__pycache__/*.pyc` 僵尸缓存假通过测试”**），本版任务书内置了**严格的防作弊红线（Anti-Shortcut Redlines）与自动化 AST / 行数 / 导入完整性验收脚本**。

---

## 重构前准备（回退至干净的 Git HEAD）

在投喂 Phase 1 之前，请先在宿主机或容器中将工作区彻底回退到最近一次提交 `48b4fc9`（保留本 `docs/refactoring-tasks/` 目录）：

```bash
# 备份任务书 -> 硬重置工作区与清理未跟踪文件 -> 恢复任务书
cp -r docs/refactoring-tasks /tmp/refactoring-tasks-backup
git reset --hard HEAD
git clean -fd
rm -rf docs/refactoring-tasks && mv /tmp/refactoring-tasks-backup docs/refactoring-tasks
find . -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
```

---

## 六阶段重构路线图与硬性验收指标

| 阶段 | 任务书文件 | 核心目标（一步到位，不留返工） | 硬性量化红线 |
| :--- | :--- | :--- | :--- |
| **Phase 1** | [phase-01-cleanup-legacy-and-dead-splines.md](phase-01-cleanup-legacy-and-dead-splines.md) | **彻底清除历史死代码**：一次性删除 S1 Mock 链路、S2/S3 非受限样条接口、**以及仿真端残留的 SciPy `chunk_plan.py`、`reference_compiler.py`、`motion.py:start/_step_legacy`**、旧版 Schema (v2~v4) 与废弃配置 | 删除 45+ 个历史文件；全仓净减 ~19,000 行；无任何非流式控制死分支 |
| **Phase 2** | [phase-02-single-layer-pydantic-and-contracts.md](phase-02-single-layer-pydantic-and-contracts.md) | **单层 Pydantic v2 声明式模型与契约层下沉**：公开 Config 与 Contract DTO **直接继承 `StrictContractModel`**（严禁 `_XxxModel` + `@dataclass` 双层套壳！严禁保留 `__post_init__`！）；将 2,800 行样条与 Ruckig 数学下沉至 `robot_contracts/trajectory/`；松绑单测对自然语言文案的死板断言 | 全仓 `grep "class _.*Model"` 为 0；`contracts/` 顶层 $\le 1,100$ 行；10 个配置文件合计 $\le 1,680$ 行 |
| **Phase 3** | [phase-03-config-ssot-and-eliminate-native.md](phase-03-config-ssot-and-eliminate-native.md) | **配置单一数据源（SSOT）与全仓彻底去 `Native*` 正名**：控制配置收敛至 `profiles/control/`；**不仅模块去 `native_` 前缀，且 Python 类名（`StreamRequest`、`ExecutionFeedback`、`MotionController`、`ControlSession`）与 ROS IDL（`StreamCommand.msg` 等）同步正名** | 全仓源码与 ROS IDL 中 `Native*` 类名/消息名残留为 0；`target_count` 仅一处定义 |
| **Phase 4** | [phase-04-task-plugin-and-motion-composition.md](phase-04-task-plugin-and-motion-composition.md) | **任务插件解耦、运动控制器显式组合拆分与仿真瘦身**：ROS 任务接口泛化为 `TaskState.msg`；抽离 `SimTaskPlugin`（注意 `ros_conv.py` 延迟导入 ROS 消息）；用 `BrakingController` + `GripperContactTracker` **显式组合（严禁 `*Mixin`）** 将 `motion.py` 压至 $\le 550$ 行；离线检查工具移入 `tools/`；瘦身 `control.py` / `node.py` / `worker.py` | `motion_controller.py` $\le 550$ 行；无 `*Mixin` 类；`simulation/`（除 `tools/` 外）无 $> 720$ 行文件 |
| **Phase 5** | [phase-05-learning-pipeline-and-hash-unpinning.md](phase-05-learning-pipeline-and-hash-unpinning.md) | **学习端配置契约合一、取消 YAML 手写 SHA-256 锁死与推理流水线收口**：删除 `act_contract.py` 合并入 `policy_config.py`；将 YAML 静态 SHA-256 改为运行时自动计算写入 Manifest；将会话锁、RGB 解码、准入、时间融合收归 `policy_service.py` + `ChunkedPolicyPipeline` | `adapters/lerobot_act.py` $\le 200$ 行；`act_contract.py` 彻底删除；YAML 配置无需手填 `sha256` |
| **Phase 6** | [phase-06-training-speedup-and-split-monoliths.md](phase-06-training-speedup-and-split-monoliths.md) | **训练提速（`PyAV` + 默认批处理 Collate + `pin_memory` + `AMP`）与千行巨石模块/超长函数拆分**：拆分 `training.py`、`evaluate.py`、`episode_audit.py`、`episode_runner.py`；完成全仓最终验收 | 全仓非测试 Python 总行数 $\le 35,000$ 行；无任何 $> 750$ 行文件；核心模块无 $> 85$ 行函数 |

---

## 投喂给执行模型的标准提示词模板

```text
请严格按照 docs/refactoring-tasks/phase-01-cleanup-legacy-and-dead-splines.md 中的任务书要求执行重构。
特别注意遵守文件顶部的【执行模型必读指令（System Rules & Anti-Shortcut Redlines）】，只完成本阶段要求，不要跨阶段修改。完成后逐条运行本阶段末尾的量化验收命令，确保所有检查和单元测试全部通过后再汇报结果。
```

