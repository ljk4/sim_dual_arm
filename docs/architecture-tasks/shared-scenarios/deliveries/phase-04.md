# 阶段 04 交接：学习端与模型绑定接入

- 日期：2026-11-11（Asia/Shanghai）。状态：完成（代码与验证）；未进入阶段 05。
- 依据：总要求、阶段 04、progress、阶段 03 交接。沿用既有 binding API，未改数据划分、标签、归一化、训练参数、融合或夹爪准入。

## 入口与参数规则

新增 `learning/lerobot/src/dual_arm_learning/scenario.py`：包装 `robot_contracts.binding.load_binding`，只做本端支持校验、recipe 选择与一致性检查；不新增解析格式，不含方案名分支，ROS-free 且不加载模型。

- `convert` / `verify-data` / `train` / `budget-status` / `bundle`：新增 `--scenario <id>`、`--shared-root`，与 `--config` 互斥（argparse）。选中 `recipes.dataset`（convert）或 `recipes.training`（其余），要求 binding 满足 `learning` 阶段，因此不要求已有模型。
- 训练类命令要求 `training.data.config` 与选中的 `recipes.dataset` 为同一真实文件（`samefile`，兼容 `/shared` ↔ `/workspace` 挂载别名），且数据 recipe 的 profile 与 binding 的 `profiles.robot` 一致；不一致在创建 run 目录/写台账前拒绝。
- `prepare-policy`：新增 `--scenario`、`--shared-root`，与 `--template` 互斥；方案模式再拒绝显式 `--profile/--observation-schema/--action-schema`。选择 `recipes.policy_service` + binding 的三个档案，`--bundle` 可覆盖 `models.bundle`，要求 `inference` 阶段（有效模型）。
- `policy`：新增 `--scenario`、`--shared-root`，与 `--config` 互斥；方案模式拒绝显式 profile 参数，读取 `recipes.policy_service`、binding 档案与有效模型（`--bundle-dir` 覆盖）。
- 本端只声明实现 `learning.implementation=lerobot`、`learning.adapter=act`；其他声明显式返回 `unsupported learning.*`，不继续执行 ACT。
- `learning/lerobot/scripts/run.sh`：检测到 `--scenario` 或用户显式 `--config` 时不再注入 `act_dataset.container.yaml`、`act_training.container.yaml`、`policy_temporal.yaml`（含 env 默认）；直接配置模式不变。`--help` 不解析 binding、不加载模型。

## 命令示例（learning 容器内）

```bash
# 方案选择：learning 阶段，不要求已有模型；不能再给 --config
learning/lerobot/scripts/run.sh train --scenario panda_stack --mode smoke
learning/lerobot/scripts/run.sh convert --scenario panda_stack \
  --select /data/campaigns/.../formal-select.json --output-root /data/learning/new-run
learning/lerobot/scripts/run.sh bundle --scenario panda_stack \
  --run /data/learning/runs/<run> --output /data/learning/bundles/<new>

# 推理准备：inference 阶段，binding 无模型时必须给 --bundle
learning/lerobot/scripts/run.sh prepare-policy --scenario panda_stack \
  --bundle /data/learning/bundles/stack-act --run-id <run> --output /data/prepared/<run>/policy

# 直接配置模式保持不变（不得与 --scenario 同用）
learning/lerobot/scripts/run.sh train --config <training.yaml> --mode smoke
learning/lerobot/scripts/run.sh prepare-policy --template <policy.yaml> --bundle <dir> ...
```

## 记录字段（沿用既有 manifest/快照，不新建记录体系）

- convert `manifest.json`、bundle `manifest.json`、训练 `training_report.json`：方案模式新增 `selection{mode,scenario_id,stage,binding{path,sha256},recipe{field,declared,path,sha256},template}`（训练另含 `dataset_recipe`）与 `overrides`。直接模式字节/字段不变。
- prepare-policy `run_manifest.json`：新增 `selection`（policy role 含 `bundle{declared,effective,source,path,exists,digest,validated_by}`）、`model_source{source,declared,path,bundle_digest}`、`overrides`。
- policy `resolved_contract.json`：新增 `config{path,sha256}`；方案模式另含 `selection`、`model_source`、`overrides`。
- `--bundle`/`--bundle-dir` 覆盖时，加载、真实校验、生成的 `policy.yaml`、`policy_binding.json.model.bundle`、manifest/snapshot 均指向同一 effective 目录。

## 验证结果

- 新增 `tests/unit/test_learning_scenario.py` 24 项：方案 recipe 实际进入 `convert`/`train`/`bundle`/`verify-data` 的配置对象与调用；显式/默认 config 区分；参数冲突；不同 cwd；缺模型；空 bundle；不支持的 implementation/adapter；training 数据引用不一致；prepare-policy 真实 bundle/档案身份及 `--bundle` 覆盖；policy 服务分发（test double，不启动服务）；run.sh 注入规则。
- 回归：learning 相关 `122 passed, 1 skipped`；runtime prepare/binding `68 passed`；`ruff check` 全通过；learning 入口导入无 ROS/runtime/simulation/MuJoCo 模块。
- 真实端到端（只读模型、临时诊断输出）：
  - `learning/lerobot/scripts/run.sh prepare-policy --scenario panda_stack --bundle /data/learning/bundles/stack-act --run-id phase04-policy --output /data/diagnostics/phase-04-20261111/prepared-policy` 成功；bundle digest `952bd7…6b76f`，`selection.bundle.source=override`，`policy_binding.model.bundle`/`inputs.bundle`/`model_source.path` 一致。
  - `runtime/ros2/scripts/run.sh prepare --scenario panda_stack --role policy --policy-binding <上一步> --bundle /data/learning/bundles/stack-act --seed 20001 --run-id phase04-policy --output …/prepared-runtime` 成功；runtime 身份链与阶段 03 检查保持。
  - 真实 binding 选择 `act_dataset.container.yaml`/`act_training.container.yaml`，`data.config` 与 dataset recipe `samefile=True`。
  - 真实 binding 无 `models.bundle` 时 prepare-policy 明确报 `BINDING_STAGE … models.bundle`；空目录 bundle 报无法读取 manifest；两者均未创建输出。
- 证据：`.build/phase-04-20261111/`（pytest、ruff、真实 prepare/runtime manifest、真实 recipe 选择、缺模型/空 bundle 输出）；本轮 `/data/diagnostics/phase-04-20261111` 已清理。

## 收尾（阶段 05 开始时完成）

- `tests/unit/test_learning_scenario.py` 739 → 561 行；binding/recipe/tree 构造、CLI 参数构造与 `run.sh` 替身迁入 `tests/unit/learning_scenario_fixtures.py`（203 行）。
- 保留全部 24 个用例与覆盖；拆分前后 `pytest --collect-only` 的 node id 集合逐行相同（`.build/phase-05-20261111/phase04-test-selection-*.txt`）。learning 容器 `24 passed`。
- 未改 learning 生产代码、recipe、数据或模型来源。

## 未完成与下一阶段注意

- 未运行正式转换、完整训练或常驻推理服务；convert/train 的端模型验证使用测试替身，训练报告/转换 manifest 的写入路径为静态审查加参数级测试。
- `policy --scenario` 只验证到真实配置加载与分发（测试替身），未启动 gRPC 服务；完整闭环留阶段 11。
- `filter-anchors` 仍为直接配置模式（binding 无 sample-filter recipe 字段），未新增绑定。
- 旧数据、bundle、prepared、预算台账与 checkpoint 未改写；未 commit/push/stash/reset/clean。
- 阶段 05 接入采集端 `--scenario` 时可复用 `scenario.py` 的 selection/阶段校验思路，但不要复制本模块的 learning 专属字段。
