# 阶段 02 交接：方案绑定模型与只读工具

- 日期：2026-11-11（Asia/Shanghai）。状态：完成（代码与验证）；未进入阶段 03。
- 依据：总要求、阶段 02、progress、阶段 01 交接。未迁移四端 recipe、未接入四端入口、未创建真实 `panda_stack` binding。
- 环境：使用既有三容器，未重建镜像/容器/SDK；仅执行读取与 pytest。runtime pydantic 2.10.6，learning 2.14.3（同一测试集均通过）。

## API

模块：`shared/common/python/contracts/src/robot_contracts/binding.py`。纯公共契约，无 ROS/MuJoCo/PyTorch/端实现导入。

- `ScenarioBinding`：严格 v1 模型（`extra="forbid"`）。顶层 `schema_version=1`、`scenario_id`、可选 `description`、`backend{kind,implementation,task_type?}`、`runtime{implementation}`、`learning{implementation,adapter?}`、`profiles{robot,observation,action}`、可选 `recipes`（8 项可空）、`interfaces{common_ros[],extension_ros[],policy_api?}`、`models{bundle?}`、`source_roots[]`、`artifact_roots[]`、`references[]`。未知字段/版本/重复 YAML 键均失败。
- `load_binding(path, *, shared_root, bundle=None) -> ResolvedBinding`。`shared_root` 必须显式给出（Schema 从 `<shared_root>/common/interfaces/schemas` 读取）；`bundle` 为显式调用方路径，覆盖 `models.bundle`。
- `ResolvedBinding`：`path/root/sha256/binding/source_roots/artifact_roots/references/dependencies/bundle`；方法 `requirements(stage)`、`missing(stage)`、`validate_stage(stage)`、`effective_bundle()`、`summary()`。
- `ResolvedReference`：`field/declared/path/kind(file|directory)/sha256?/semantic_sha256?`，由 `show` 的 `references` 字段按点号路径输出。
- `ResolvedBundle`：`declared/effective/path/source(binding|override|none)`；`summary()` 中 `digest` 恒为 `null`，`validated_by="learning.verify_bundle"`。
- `BindingStage`：`collection|learning|inference`。`BindingError(ContractError)` 的 `reason` 固定为 `BINDING_SCHEMA`、`BINDING_REFERENCE`、`BINDING_CYCLE`、`BINDING_STAGE`；消息含绑定路径、字段/引用路径或循环链。

## 路径与摘要规则

- 所有 binding 内路径相对 `binding.yaml` 所在目录解析，与 cwd 无关。binding 目录始终是源根；`source_roots` 必须为相对路径并显式声明跨方案/现有端配置范围；`artifact_roots` 可为相对或绝对部署路径。
- profile/recipe/extension/`references` 是源引用，必须相对且落在 binding 目录或 `source_roots` 内；`models.bundle` 与 `--bundle` 可绝对，但必须落在 binding 目录、`source_roots` 或 `artifact_roots` 内。越界/绝对源引用/类别错误（文件/目录）明确失败。CLI 的 `--binding`/`--bundle` 本身按调用方 cwd 解析。
- 显式跨方案复用：声明 `source_roots: [".."]` 等后引用相邻方案。`references` 仅表示独立 binding 依赖，递归加载并检测环，不做字段合并、默认回退或继承。
- 文件摘要为原始字节 SHA-256；profile/observation/action 另附 `semantic_hash` 语义摘要。目录不做内容摘要；模型目录只查存在性/类型，不加载权重、不伪造 digest，模型路径不进入 profile 语义摘要。
- 不自动发现方案、不默认 Panda、不执行 shell、不启动进程、不写文件/台账/种子。

## 阶段条件

| 阶段 | 必需（始终含三个 profiles） |
| --- | --- |
| collection | `recipes.runtime_teacher`、`recipes.campaign`；simulation 另需 `recipes.simulation_teacher` |
| learning | `learning.adapter`、`recipes.dataset`、`recipes.training` |
| inference | `learning.adapter`、`recipes.runtime_policy`、`recipes.policy_service`、`interfaces.policy_api`、`models.bundle`；simulation 另需 `recipes.simulation_policy` |

`collection`/`learning` 允许 `models.bundle: null`；`inference` 缺模型明确报 `BINDING_STAGE`（可用 `--bundle` 满足并记录 `source: override`）。

## CLI

```text
python3 shared/common/tools/scenario.py validate --binding <binding.yaml> --stage <collection|learning|inference> [--bundle <dir>] [--shared-root <dir>]
python3 shared/common/tools/scenario.py show --binding <binding.yaml> [--shared-root <dir>]
```

- `--shared-root` 优先级：参数 > `SHARED_ROOT` 环境变量 > 工具自身 `shared/` 布局（`<tool>/../../..` 推导为 shared 根）；输出为 JSON（stdout），错误为 `error: <reason>: ...`（stderr），成功 0/失败 2。
- `show` 输出 `binding{path,sha256,...}`、`declarations`、`roots`、`references`、`models`、`dependencies`、`stages{requirements,missing,satisfied}`；不输出可 `eval` 的 shell。
- `validate` 只读输入并执行一个阶段的条件；不创建台账、不预留种子、不加载权重、不连接 ROS、不启动训练/后端。

## 验证

- runtime：`runtime/ros2/scripts/run.sh test -q tests/unit/test_scenario_binding.py tests/unit/test_scenario_tool.py tests/unit/test_profile_cameras.py tests/unit/test_contract_consistency.py tests/unit/test_offline_audit.py` → **130 passed**。
- learning：`learning/lerobot/scripts/run.sh test -q tests/unit/test_scenario_binding.py tests/unit/test_scenario_tool.py tests/unit/test_profile_cameras.py tests/unit/test_contract_consistency.py` → **125 passed**。
- 覆盖：正确输入、缺必填字段、重复键、未知字段/版本、缺文件、类别错误、越界/绝对源引用、显式跨方案复用、引用环/自环、Profile 绑定不一致、阶段条件、`--bundle` 覆盖、不同 cwd 同结果、只读/无进程/无运行端导入。
- CLI 证据 `.build/phase-02-20261111/`：`show.json` 与从 `/tmp` 运行的输出字节一致；`inference-missing-model.exit=2` + `BINDING_STAGE`；`inference-bundle-override.json` 记录真实 override 来源且 `digest=null`；`profile-semantic-hashes.txt` 与阶段 01 基线一致（`9e0b5b5f…`、`452d8a15…`、`93689bc5…`）。`ruff check` 新增文件全通过。
- 既有 runtime 3 个失败（phase-01 基线：`dual_arm_learning` 不在 runtime PYTHONPATH）本阶段未复跑、未修复；本阶段未跑完整套件。

## 未验证 / 限制

- 未创建真实 `shared/scenarios/panda_stack/binding.yaml`；目标 recipe/扩展包尚不存在，测试全部使用临时合成目录。不能据此声称示例方案可运行。
- 所有已声明的 recipe/extension 在加载时都必须存在，不会因当前阶段未选中而放宽；阶段 03 建立真实 binding 时只声明已落地项，后续阶段再补齐。
- recipe 仅校验存在性与原始摘要，私有 YAML 完整语义仍由四端校验；extension ROS 包仅校验目录存在。
- `references` 不提供继承/合并/默认值；循环检测只覆盖显式绑定引用，不猜测私有 recipe 字符串。
- bundle 内容真实性、摘要与 60x16 契约仍由 learning `verify_bundle`/加载端校验；模型目录为空也会通过本阶段存在性检查。
- `--scenario` 与同类 `--config`/`--template` 互斥属后续端入口，本阶段未实现选择器；未跑 ROS/MuJoCo/训练/推理/历史数据。

## 下一阶段注意

- 阶段 03 建立真实 binding 时，路径保持相对 binding 目录；跨端现有配置用显式 `source_roots`，不要按 cwd 猜根。
- 推理侧传入 `--bundle` 或写入 `models.bundle` 后，运行记录应保存 `effective_bundle()` 的真实路径与 `source`，digest 待 learning 校验后填写。
- 不要在本层加入方案名分支、隐式 Panda 默认、自动发现或深层继承。
