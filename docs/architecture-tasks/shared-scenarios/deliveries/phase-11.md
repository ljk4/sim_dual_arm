# 阶段 11 交接：代码精简

- 日期：2026-11-12（Asia/Shanghai）。状态：完成。未进入阶段 12。
- 依据：总要求、阶段 11、progress、阶段 10 交接。未修改 binding、配方、Profile、模型、历史数据、台账和第三方资产；未 commit/push/stash/reset/clean。

## 选择记录复用

- [`robot_contracts/binding.py`](../../../../shared/common/python/contracts/src/robot_contracts/binding.py) 的 `ResolvedReference.record()` 统一生成 `field/declared/path/sha256`（有语义摘要时附 `semantic_sha256`）。runtime collection、learning 转换/训练/服务、simulation/runtime 的 prepare 不再各自拼同一份来源记录；`scenario_selection._recipe_record` 与 `_profile_records` 的重复构造随之删除。
- prepare 的选择结果不再携带与 `recipe` 完全相同的 `template` 记录。`inputs.template` 仍记录直接模式与方案模式的真实路径和摘要。
- 方案模式直接复用 binding 已计算的文件摘要：`prepare_backend`、`prepare_run`、`prepare_policy` 的 `inputs.template_sha256` 与选择记录一致，不再对同一文件二次哈希。直接模式仍计算一次并在 manifest 和契约文件中复用。`prepare_policy` 的 `policy.yaml` 摘要也只计算一次。
- `scenario_selection.campaign_config_path` 只返回导出实际使用的 campaign 路径；此前返回的 selection 没有任何消费者。`campaign` 中写入 `argparse` 命名空间却从未读取的 `args.selection` 一并删除。

## 采集编排

- [`record_workers.py`](../../../../runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/offline/record_workers.py)：方案模式下，`select_collection(require_parallel=True)` 已在同一进程校验 action source、task mode、Teacher 能力、render profile 和 `render_quality`，准备阶段不再重复读取同一配方做同一组判定。直接模式保留这些模板检查。每个 worker 不再复制顶层 selection 的 `profiles`；顶层 `selection.profiles` 仍是唯一来源。domain id 只计算一次。
- [`record_pool.py`](../../../../runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/offline/record_pool.py)：删除无调用的 `load_workers`；合并两轮相同的 per-worker 身份校验；控制器直接从 `record` CLI 提供的 `args.selection/scenario/shared_root` 读取，不再重复 `getattr` 兜底。
- [`record_parallel.py`](../../../../runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/offline/record_parallel.py)：方案模式的解析参数由同一个 `_scenario_selection_arguments` 提供给 prepare 与 controller；`run` 中重复的一份已删除。prepare 需要 `--backend-template`、controller 需要 `--config` 的差异仍按各自入口保留。
- [`collect_attempt.py`](../../../../runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/offline/collect_attempt.py) 提供唯一的 `sdk_check_command`；[`record_preflight.py`](../../../../runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/offline/record_preflight.py) 复用它并只负责捕获输出。预检、worker 与 pool 的跨进程 SDK 检查保持不变。

## SDK 与历史读取

- [`sdk_packages.py`](../../../../shared/common/tools/scripts/sdk_packages.py)：`PackageSource` 携带 `package.xml` 中解析出的依赖，`package_source` 每个包只解析一次清单；依赖闭包使用同次解析结果，不再单独重读 `package.xml`。删除了仅重复该解析的 `package_name`/`package_dependencies`。
- [`dev_workspace.py`](../../../../shared/common/tools/scripts/dev_workspace.py) 的 `digest` 输出直接复用 `package_record` 的结果，不再逐字段复制一遍。
- marker 校验仍通过 `selection_from_marker` 检查记录中的来源路径，保留“核心包集不能冒充完整包集 / 外来来源必须拒绝”的边界。
- [`feedback_definition.py`](../../../../runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/offline/feedback_definition.py) 提供 `recorded_definition`/`installed_definition`；[`follower_definition.py`](../../../../runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/offline/follower_definition.py) 复用同一查找与错误包装，不再重复读取消息定义。

## 测试辅助

- `sha()` 与 `ROOT` 集中到 [`scenario_fixtures.py`](../../../../tests/unit/scenario_fixtures.py)，collection/learning fixtures 复用，删除重复定义。
- 适配精简后的清单形状：worker 测试不再断言已被删除的 per-worker `profiles`；`campaign_config_path` 测试只断言实际路径；`load_workers` 的调用改为 `load_worker_plan`。行为断言（binding/recipe 身份、Teacher 能力、bundle 来源、SDK 包集、历史布局拒绝）保持不变。

## 保留的必要边界

- 预检（独立进程）→ worker 准备（独立容器进程）→ pool（控制器进程）之间的 binding sha、recipe、Teacher 能力和 backend 就绪检查未合并；worker manifest 仍保留 per-worker binding/recipes/teacher_api，pool 逐项校验。
- binding 加载、marker 来源校验、SDK 重建判断、Profile/模型/轨迹摘要与 learning 的真实 bundle 校验均保留原有作用。
- 方案选择仍决定实际 recipe 和有效 bundle；不支持的 backend/runtime/learning 实现、错误模型来源、缺少 bundle 的 inference 仍在创建输出前失败。

## 验证

runtime 容器（`cd /workspace`，`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`）：

- `test_scenario_binding.py test_scenario_tool.py test_prepare_scenario.py test_record_scenario.py test_record_workers_scenario.py test_record_pool.py test_record_parallel_host.py test_dev_workspace.py test_mcap_feedback_guard.py test_feedback_capability.py test_legacy_feedback.py` → 178 passed。
- `test_prepare_backend.py test_prepare_run.py` → 4 passed。
- `test_record_workers.py test_stack_campaign.py` → 14 passed。
- `test_record_scenario.py test_record_workers_scenario.py test_record_workers.py test_record_pool.py`（在 SDK 命令复用改动后复跑）→ 48 passed。
- `test_stack_export_build.py test_stack_export_quality.py test_stack_label_export.py test_offline_audit.py` → 15 passed。
- ruff 对全部改动文件与 `collect_attempt.py`/`record_preflight.py` → all checks passed。

learning 容器：`test_prepare_policy.py test_learning_scenario.py` → 26 passed。

有界运行（未启动 backend、未预留预算）：

- runtime 容器对真实 `panda_stack` 执行 `python -m dual_arm_runtime.offline.record_workers --workers 1 --domain-base 200 --scenario panda_stack --shared-root /shared`，exit 0；生成选择为 scenario/panda_stack、Teacher 能力正确、worker 配置可读、无 per-worker `profiles`，临时目录已删除。
- simulation 容器对真实方案执行 teacher 与 policy 两次 `prepare_backend`，两次 `inputs.template_sha256` 均与 `selection.recipe.sha256` 相同，选择记录不再含 `template`，policy bundle 来源为显式 `/data/learning/bundles/stack-act`；临时输出已删除。
- runtime 容器内直接检查 host launcher 的 scenario/direct 参数构造，与 prepare/controller 两个入口各自的预期一致。

未改生成接口，未重建 ROS/Policy SDK。完整 Teacher RPC、并行录制与中断、Policy 闭环留阶段 12。

## 已知问题

- 新生成 manifest 的 `selection` 不再含与 `recipe` 重复的 `template` 键；阶段 03/04 交接描述的是当时的产物形状。旧产物不迁移，直接读取不受影响。
- 新 worker manifest 的 per-worker 条目不再复制 `profiles`；顶层 `selection.profiles` 保留完整记录。
- `select_collection` 仍加载 campaign recipe 作为选择校验，`record_preflight` 随后再加载同一文件取得 status 所需配置。保留 selection 的校验职责，避免把非序列化文档塞进预检计划；该文件很小，重复读取未影响业务门禁。
- 本轮只执行了准备与参数构造的有界运行，没有启动并行 backend 或录制进程；整体闭环由阶段 12 完成。
