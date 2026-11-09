# Phase 5 任务书：学习端配置契约合一、移除 YAML 静态 SHA-256 锁死与推理流水线彻底收口

## 【执行模型必读指令（System Rules & Anti-Shortcut Redlines）】
1. **前置条件**：确认 **Phase 1 ~ Phase 4** 已完成且测试通过。你当前只负责执行 **Phase 5**。
2. **【严禁把 `ChunkedPolicyPipeline` 写成无人调用的死代码红线】**：
   - `policy_service.py` 在处理 `Predict` 请求时，**必须真实实例化并调用 `ChunkedPolicyPipeline.predict()`**！
   - **严禁在 `adapters/lerobot_act.py` 中保留 `_Session` 会话锁、相机 RGB 解码、准入校验、时间融合或超时判断**！`adapters/lerobot_act.py` 只允许保留纯粹的模型推理适配器 `BundleActModel`（**硬性红线：行数必须 $\le 200$ 行**）。
3. **【严禁删除 `.py` 源文件留 `.pyc` 红线】**：
   - 新建的 `pipeline.py`、`adapters/base.py`、`adapters/lerobot_act.py` 必须真实存在于磁盘上，严禁误删！

---

## 1. 合并 `policy_config.py` 与 `act_contract.py`（彻底删除 891 行的 `act_contract.py`）

当前 `learning/lerobot/src/dual_arm_learning/` 下同时维护了 `policy_config.py`（395 行）和 `act_contract.py`（891 行，`PolicySettings` + `PolicyContract`），启动一个推理服务要把相同字段来回转换 4 次。

### 重构要求：
1. **物理删除 `learning/lerobot/src/dual_arm_learning/act_contract.py`（891 行）**：
   - 将其中核对 Bundle 目录（`bundle_manifest.json`）与 `RobotProfile` 一致性的核心函数（约 70 行）合并入 [`policy_config.py`](../../learning/lerobot/src/dual_arm_learning/policy_config.py)。
2. **用单层 `StrictContractModel` 重写 [`policy_config.py`](../../learning/lerobot/src/dual_arm_learning/policy_config.py)（目标 $\le 260$ 行）**：
   - 统一使用 `PolicyServiceConfig`（删除 `PolicySettings` / `PolicyContract` 重复类）。
   - 加载配置时，若指定了 `robot_profile_path` 与 `bundle_dir`，自动从 `robot_profile_path` 读取关节/相机规格与 `profile_hash`，自动从 `bundle_dir/bundle_manifest.json` 读取 `model_bundle_digest` 并核对 `profile_hash` 是否匹配。
3. **重命名配置文件与 CLI 入口**：
   - `learning/lerobot/configs/act_policy_temporal.yaml` $\rightarrow$ `learning/lerobot/configs/policy_temporal.yaml`
   - `learning/lerobot/src/dual_arm_learning/act_service.py` $\rightarrow$ `learning/lerobot/src/dual_arm_learning/policy_service_cli.py`
   - `learning/lerobot/src/dual_arm_learning/act_admission.py` $\rightarrow$ `learning/lerobot/src/dual_arm_learning/chunk_admission.py`
   - `learning/lerobot/src/dual_arm_learning/act_temporal.py` $\rightarrow$ `learning/lerobot/src/dual_arm_learning/temporal_ensemble.py`

---

## 2. 移除 YAML 配置文件中的静态 SHA-256 锁死机制（改为运行时自动记录 Provenance Logging）

当前在 `act_dataset.yaml`、`act_training.yaml`、`policy_temporal.yaml` 中强制要求手填 `source_archive_sha256`、`weights.sha256`、`model_bundle_digest` 等静态 SHA-256，导致任何微调都会引发全链路配置文件连环报错。

### 修改要求：
1. 在 `DatasetConfig`、`TrainingConfig`、`PolicyServiceConfig` 中，将 `source_archive_sha256`、`weights.sha256`、`model_bundle_digest`、`observation_schema_hash`、`action_schema_hash` 等字段全部设为**可选字段（默认 `None` 或 `""`）**，并从默认 YAML 配置文件中移除这些硬编码哈希。
2. 在运行 `dataset.py`、`training.py`、`policy_config.py` 时：**自动计算实际输入文件/Bundle 的 SHA-256 摘要并写入生成的 `dataset_manifest.json`、`training_report.json` 与运行日志**（仅当用户在 YAML 中显式填写了非空哈希值时才进行比对拦截）。

---

## 3. 提炼通用 `ChunkedPolicyPipeline` 与精简 `adapters/lerobot_act.py`

1. **新建 [`learning/lerobot/src/dual_arm_learning/adapters/base.py`](../../learning/lerobot/src/dual_arm_learning/adapters/base.py)（$\le 40$ 行）**：
   - 定义 `PolicyModelAdapter` Protocol：`policy_family`、`chunk_size`、`action_dim`、`reset()`、`predict_raw_chunk(rgb_images, joint_positions) -> np.ndarray`。
2. **新建 [`learning/lerobot/src/dual_arm_learning/pipeline.py`](../../learning/lerobot/src/dual_arm_learning/pipeline.py)（$\le 190$ 行）**：
   - 实现 `decode_rgb()`、`PipelineResult` 与 `ChunkedPolicyPipeline`（封装 `adapter.predict_raw_chunk` $\rightarrow$ `admit_action_chunk` $\rightarrow$ `TemporalEnsemble` $\rightarrow$ `save_prediction`）。
3. **将 `act_provider.py` 移至 [`adapters/lerobot_act.py`](../../learning/lerobot/src/dual_arm_learning/adapters/lerobot_act.py) 并大幅瘦身（从 675 行压至 **$\le 200$ 行**）**：
   - 只保留 `BundleActModel`（实现 `PolicyModelAdapter`）与 `load_act_adapter()`。
   - 将会话字典锁、观测单调性检查、图像解码、`ChunkedPolicyPipeline.predict()` 调用、产物落盘和预测超时判断全部收归 [`policy_service.py`](../../learning/lerobot/src/dual_arm_learning/policy_service.py)（使 `policy_service.py` 控制在 **$\le 580$ 行**）。
4. **合并 `learning_config_sections.py` 并修复 2 个学习端单测细节**：
   - 将 `learning_config_sections.py` 合回 `learning/lerobot/src/dual_arm_learning/config.py`（删除 `learning_config_sections.py`）；
   - 修复原 `test_act_provider.py`（重命名为 `test_lerobot_act_adapter.py`）子进程测试拼接 `PYTHONPATH` 时遗漏 `contracts/python/src` 的问题，以及对齐 `test_temporal_ensemble_is_off_by_default` 与 `PolicyServiceConfig` 默认值的断言。

---

## 4. 本阶段量化验收标准（必须逐条执行并全部达标）

1. **文件存在性与行数硬指标检查**：
   ```bash
   find . -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
   test ! -f learning/lerobot/src/dual_arm_learning/act_contract.py
   test ! -f learning/lerobot/src/dual_arm_learning/policy_contract.py
   test -f learning/lerobot/src/dual_arm_learning/pipeline.py
   test -f learning/lerobot/src/dual_arm_learning/adapters/base.py
   test -f learning/lerobot/src/dual_arm_learning/adapters/lerobot_act.py

   wc -l \
     learning/lerobot/src/dual_arm_learning/policy_config.py \
     learning/lerobot/src/dual_arm_learning/pipeline.py \
     learning/lerobot/src/dual_arm_learning/policy_service.py \
     learning/lerobot/src/dual_arm_learning/adapters/lerobot_act.py
   ```
   - `policy_config.py` **必须 $\le 260$ 行**；
   - `adapters/lerobot_act.py` **必须 $\le 200$ 行**；
   - `policy_service.py` **必须 $\le 580$ 行**。
2. **验证 `policy_service.py` 真实调用了 `ChunkedPolicyPipeline`**：
   ```bash
   grep -rn "ChunkedPolicyPipeline" learning/lerobot/src/dual_arm_learning/policy_service.py
   ```
3. **代码检查与测试全部通过**：
   ```bash
   ruff check .
   ./learning/lerobot/scripts/run.sh test
   ./runtime/ros2/scripts/run.sh test
   ```
