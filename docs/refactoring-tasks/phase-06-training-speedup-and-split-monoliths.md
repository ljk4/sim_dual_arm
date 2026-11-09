# Phase 6 任务书：训练提速（Batch Collate + AMP）、拆分千行巨石模块与最终总验收

## 【执行模型必读指令（System Rules & Anti-Shortcut Redlines）】
1. **前置条件**：确认 **Phase 1 ~ Phase 5** 已完成且测试通过。你当前只负责执行收官阶段 **Phase 6**。
2. **【严禁保留 `collate_fn=list` 抵消 `pin_memory` 红线】**：
   - 在重构 `training.py` 的 `DataLoader` 时，**必须去掉 `collate_fn=list`**，使用 PyTorch 默认的 `default_collate` 让多进程 Worker 直接拼好 Batch Tensor 并由 `pin_memory` 锁页传输到 GPU！
3. **【严禁机械剪切或删 `.py` 留 `.pyc` 红线】**：
   - 拆分千行巨石文件时，所有新模块（如 `training_support.py`）必须真实保留在磁盘上；
   - **硬性红线**：本阶段完成后，**全仓（除 `simulation/.../tools/` 外）不得有任何 $> 720$ 行的 `.py` 文件**，核心模块中**不得有任何 $> 85$ 行的函数**，全仓非测试 Python 总行数 **必须 $\le 35,000$ 行**！

---

## 1. 训练管线性能优化（`PyAV` + 默认 Batch Collate + Multi-Worker + `pin_memory` + `AMP`）

1. **数据集加载器切换至 `pyav`**：
   - 在 [`learning/lerobot/src/dual_arm_learning/dataset.py`](../../learning/lerobot/src/dual_arm_learning/dataset.py) 的 `load_lerobot_dataset` 中显式传入 `video_backend="pyav"`。
2. **在 [`config.py`](../../learning/lerobot/src/dual_arm_learning/config.py) 的 `TrainingConfig` 中新增性能参数**：
   - `num_workers: NonNegativeInt = 4`
   - `pin_memory: bool = True`
   - `prefetch_factor: PositiveInt = 2`
   - `use_amp: bool = True`
   - `amp_dtype: Literal["bfloat16", "float16"] = "bfloat16"`
   - 同步更新 `act_training.yaml` 与 `act_training.container.yaml`（设置 `batch_size: 32`、`num_workers: 4`、`pin_memory: true`、`use_amp: true`）。
3. **重构 [`training.py`](../../learning/lerobot/src/dual_arm_learning/training.py) 的 `DataLoader` 与 AMP 训练循环**：
   - **去掉 `collate_fn=list`**，让 `DataLoader` 使用默认 `default_collate` 直接产出 batched Tensor 字典，并启用 `num_workers`、`persistent_workers=(num_workers > 0)`、`pin_memory`、`prefetch_factor`。
   - 修改预处理函数 `_process_batch(pre, batch, keys)` 直接支持对 batched Tensor 字典做预处理与 GPU 传输。
   - 在前向计算与 Loss 求解中接入 `torch.amp.autocast(device_type="cuda", dtype=..., enabled=use_amp_on_cuda)` 与 `torch.amp.GradScaler("cuda", enabled=(use_amp_on_cuda and amp_dtype == "float16"))`。

---

## 2. 拆分并瘦身 Learning 与 Runtime 七大千行巨石模块（全部压至 $\le 700$ 行，单函数 $\le 85$ 行）

1. **拆分 [`learning/lerobot/src/dual_arm_learning/training.py`](../../learning/lerobot/src/dual_arm_learning/training.py)（1,070 行）**：
   - 新建 [`training_support.py`](../../learning/lerobot/src/dual_arm_learning/training_support.py)（**目标 $\le 400$ 行**）：承载模型与优化器构建、预训练 ResNet 权重加载校验、验证集评估（`evaluate_validation`）与训练报告组装（`build_training_report`）。
   - 将 `training.py` 压缩至 **$\le 480$ 行**，并将原 410 行的 `train()` 函数拆分为 `_prepare_training_context()`、`_run_training_steps()`、`_finalize_training()`，确保每个函数 **$\le 80$ 行**。
2. **瘦身 [`dataset.py`](../../learning/lerobot/src/dual_arm_learning/dataset.py)（1,007 行 $\rightarrow$ **目标 $\le 680$ 行**）与 [`sample_filter.py`](../../learning/lerobot/src/dual_arm_learning/sample_filter.py)（987 行 $\rightarrow$ **目标 $\le 620$ 行**）**：
   - 删除手写的配置/JSON/Parquet 字段重复校验与冗余字典拷贝，改用 `StrictContractModel` 校验过滤规则。
3. **瘦身 [`episode_audit.py`](../../runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/episode_audit.py)（1,024 行 $\rightarrow$ **目标 $\le 600$ 行**）**：
   - 删除对已通过 `robot_contracts` 校验对象的重复手写检查，将超长审计函数按主题拆分为 $\le 80$ 行的子函数。
4. **瘦身 [`episode_runner.py`](../../runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/episode_runner.py)（1,012 行 $\rightarrow$ **目标 $\le 620$ 行**）与 [`runtime_node.py`](../../runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/runtime_node.py)（977 行 $\rightarrow$ **目标 $\le 680$ 行**）**：
   - 提取 Teacher 与 Policy 模式共用的 Episode 生命周期执行骨架，消除双分支复制粘贴。
5. **拆分 [`runtime/ros2/scripts/evaluate.py`](../../runtime/ros2/scripts/evaluate.py)（1,022 行 $\rightarrow$ **目标 $\le 550$ 行**）、[`campaign.py`](../../runtime/ros2/scripts/campaign.py)（822 行 $\rightarrow$ **目标 $\le 520$ 行**）、[`export_episode.py`](../../runtime/ros2/scripts/export_episode.py)（784 行 $\rightarrow$ **目标 $\le 580$ 行**）与 [`control_cli.py`](../../runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/control_cli.py)（903 行 $\rightarrow$ **目标 $\le 520$ 行**）**：
   - 将 `evaluate.py` 中 412 行的 `_metric_for_seed()` 拆分为 $\le 80$ 行的子函数；
   - 清理 `control_cli.py` 顶部的 mock 注释与过时死代码。

---

## 3. 更新架构文档与项目状态说明

同步更新 [`README.md`](../../README.md)、[`docs/project-status.md`](../../docs/project-status.md) 及各子包 `README.md`，准确反映重构后的架构。

---

## 4. 全仓最终收官总验收脚本（必须逐条执行并 100% 达标）

1. **清空字节码缓存并运行全仓 AST 导入完整性检查（必须 0 错误）**：
   ```bash
   find . -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
   python3 -c '
   import ast, pathlib, sys
   roots = ["contracts/python/src", "libraries", "simulation/mujoco/src", "runtime/ros2", "learning/lerobot/src"]
   pkg_roots = {
       "robot_contracts": pathlib.Path("contracts/python/src/robot_contracts"),
       "robot_ros_support": pathlib.Path("libraries/ros_support/src/robot_ros_support"),
       "robot_run_logging": pathlib.Path("libraries/run_logging/src/robot_run_logging"),
       "robot_backend_sim": pathlib.Path("simulation/mujoco/src/robot_backend_sim"),
       "dual_arm_runtime": pathlib.Path("runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime"),
       "dual_arm_learning": pathlib.Path("learning/lerobot/src/dual_arm_learning"),
   }
   errors = []
   for r in roots:
       for py in pathlib.Path(r).rglob("*.py"):
           tree = ast.parse(py.read_text())
           for node in ast.walk(tree):
               if isinstance(node, ast.ImportFrom) and node.module:
                   if node.level > 0:
                       base = py.parent
                       for _ in range(node.level - 1): base = base.parent
                       rel = pathlib.Path(*node.module.split("."))
                       if not (base / f"{rel}.py").exists() and not (base / rel / "__init__.py").exists():
                           errors.append(f"Broken relative import in {py}:{node.lineno} -> {node.module}")
                   elif node.module.split(".")[0] in pkg_roots:
                       parts = node.module.split(".")
                       if len(parts) > 1:
                           sub = pkg_roots[parts[0]].joinpath(*parts[1:])
                           if not sub.with_suffix(".py").exists() and not (sub / "__init__.py").exists():
                               errors.append(f"Broken absolute import in {py}:{node.lineno} -> {node.module}")
   for e in errors: print(e)
   sys.exit(1 if errors else 0)
   '
   ```
2. **全仓单文件 720 行红线、单函数 85 行红线与总行数（$\le 35,000$ 行）检查**：
   ```bash
   python3 -c '
   import ast, pathlib, sys
   roots = ["contracts/python/src", "libraries", "simulation/mujoco/src", "runtime/ros2", "learning/lerobot/src"]
   total = 0
   over_720 = []
   for r in roots:
       for p in pathlib.Path(r).rglob("*.py"):
           lines = len(p.read_text().splitlines())
           total += lines
           if "/tools/" not in str(p) and lines > 720:
               over_720.append((lines, str(p)))
   print(f"Total non-test Python lines: {total}")
   for l, p in sorted(over_720, reverse=True): print(f"OVER 720 LINES: {l} {p}")
   assert total <= 35000, f"Total lines {total} > 35000!"
   assert not over_720, "Found files > 720 lines!"

   targets = [
       "learning/lerobot/src/dual_arm_learning/training.py",
       "runtime/ros2/scripts/evaluate.py",
       "runtime/ros2/scripts/campaign.py",
       "runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/episode_audit.py",
       "runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/episode_runner.py",
   ]
   long_funcs = []
   for t in targets:
       tree = ast.parse(pathlib.Path(t).read_text())
       for node in ast.walk(tree):
           if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
               length = node.end_lineno - node.lineno + 1
               if length > 85: long_funcs.append((length, f"{t}:{node.name}"))
   for l, loc in sorted(long_funcs, reverse=True): print(f"LONG FUNC (>85 lines): {l} {loc}")
   assert not long_funcs, "Found functions > 85 lines!"
   '
   ```
3. **全仓静态检查、Dockerfile 检查与三端全量测试通过**：
   ```bash
   ruff check .
   python3 infrastructure/scripts/check_dockerfiles.py
   ./simulation/mujoco/scripts/run.sh test
   ./runtime/ros2/scripts/run.sh test
   ./learning/lerobot/scripts/run.sh test
   ```
