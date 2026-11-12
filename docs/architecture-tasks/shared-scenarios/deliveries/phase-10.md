# 阶段 10 交接：方案模板与替换验证

- 日期：2026-11-12（Asia/Shanghai）。状态：完成。未进入阶段 11。
- 依据：总要求、阶段 10、progress、阶段 09 交接。未修改 `panda_stack` 的 binding、档案、配方或业务参数，未改动模型、数据、台账，也未 commit/push/stash/reset/clean。

## 模板入口

新增两个文件：

- [`shared/scenarios/_template/README.md`](../../../../shared/scenarios/_template/README.md)：目录结构、路径基准、权威配置与模型引用、校验/prepare 命令，以及 checkpoint、学习算法、机器人、仿真后端、任务、真机、新能力的修改位置。
- [`shared/scenarios/_template/binding.example.yaml`](../../../../shared/scenarios/_template/binding.example.yaml)：完整字段形状与占位值 `replace_me` / `REPLACE_ME`。

模板目录没有 `binding.yaml`。实测：

- `python3 /shared/common/tools/scenario.py validate --binding /shared/scenarios/_template/binding.example.yaml --shared-root /shared --stage collection` → exit 2，`BINDING_REFERENCE: ... profiles.robot: file does not exist: .../profiles/REPLACE_ME/profile.json`。
- `scripts/run.sh prepare --scenario _template --role teacher ...` → exit 1，`scenario '_template' has no binding file: /shared/scenarios/_template/binding.yaml`。

因此未填写的模板不会被入口读到，也不会回退到 Panda。

## 替换验证

临时方案放在 `.build/phase-10-20261112/replacement/`（`.build` 已被忽略，不属于方案目录）。`scenarios/test_only_replacement/binding.yaml` 明确标注 `TEST ONLY`，通过 `source_roots: ["../../../shared/scenarios"]` 引用真实 Panda 档案，并选择另一份既有配方：

- 基线 `panda_stack`：`recipes.simulation_policy` = `configs/simulation/backend/backend_stack_act_chunked.yaml`。
- test_only 方案：`recipes.simulation_policy` = `configs/simulation/backend/backend_stack_inference.yaml`。
- 两份方案使用同一 robot/observation/action 档案、同一 seed（17）、同一 bundle 路径（`/data/learning/bundles/stack-act`，只作为路径选择，未重新校验模型内容）。

实际入口在 simulation 容器执行（`docker compose ... run --rm --no-deps`，额外只读挂载临时方案根），两次均成功：

```bash
/app/scripts/run.sh prepare --scenario panda_stack --role policy --shared-root /shared \
  --bundle /data/learning/bundles/stack-act --seed 17 --run-id panda-policy \
  --output /out/panda-policy
/app/scripts/run.sh prepare --scenario test_only_replacement --role policy --shared-root /scenario-root \
  --bundle /data/learning/bundles/stack-act --seed 17 --run-id replacement-policy \
  --output /out/test-only-policy
```

输入改变：scenario id、binding 路径、`recipes.simulation_policy` 文件。输出随之改变：

| 生成配置字段 | panda_stack | test_only_replacement |
| --- | --- | --- |
| `run.seed` | 17 | 17 |
| `run.realtime_speed` | 0.5 | 1.0 |
| `limits.chunk_history` | 512 | 256 |
| `robot.task_mode` | inference | inference |
| manifest `selection.scenario_id` | `panda_stack` | `test_only_replacement` |
| manifest `selection.recipe.path` | `.../backend_stack_act_chunked.yaml` | `.../backend_stack_inference.yaml` |

`python3 /shared/common/tools/scenario.py validate --stage inference --bundle /data/learning/bundles/stack-act` 对 test_only binding 返回 `ok: true`，检查项包括 profiles、`learning.adapter`、runtime/policy 配方、`interfaces.policy_api`、`models.bundle` 与 `recipes.simulation_policy`。

公共生产代码没有增加方案名分支：`simulation/mujoco/scripts/prepare_backend.py` 不出现 `panda_stack` 或场景名比较；第二个 id 直接通过现有 `--scenario` 路径完成选择。输出与日志在 `.build/phase-10-20261112/out/`。

## 测试与检查

- 新增替换测试：`tests/unit/test_prepare_scenario.py::test_second_scenario_selects_another_recipe_without_name_branch`，用同一解析与 `prepare_backend` 选择第二个方案的另一份配方，断言生成配置差值与 manifest 选择记录；同时新增 `_template` 不可被 `prepare --scenario` 选中的用例。
- 新增模板测试：`tests/unit/test_scenario_binding.py::test_template_example_is_unfilled_and_not_runnable`，断言示例无 `binding.yaml`、占位值未填写时以 `BINDING_REFERENCE` 失败。
- 受影响四个测试文件在 runtime 容器：68 passed；宿主机同一集合：68 passed。ruff 通过。
- runtime 容器内按模板命令执行 `python3 shared/common/tools/scenario.py validate --binding shared/scenarios/panda_stack/binding.yaml --stage collection` → `ok: true`。未执行 `build-ros-sdk`（本阶段未改 SDK）。
- 未在 runtime/learning 容器重跑 prepare 全链路；其方案选择行为已有阶段 04/05 测试，模板命令参数与解析器一致。

## 依赖边界

- `shared/common` 生产包不导入 `dual_arm_learning`、`dual_arm_runtime`、`robot_backend_sim`、`mujoco`、`torch` 或 `lerobot`；`ros_support` 对 `rclpy` 的导入是其既定的 ROS 适配职责。contracts、trajectory 与 binding 保持 ROS-free。
- 生产代码不导入 `tests/` 或 Fake Policy。
- `tests/fakes/policy/fake_provider.py` 依赖 `dual_arm_learning.policy_config` 与 `policy_types`，其消费者是 `test_policy_config.py`、`test_policy_contract.py`、`test_policy_rpc_server.py`、`test_policy_service.py`。这些是 learning 端测试，Fake Policy 保持 `tests/fakes/policy/` 归属；未发现需要调整的反向依赖，未新建协议替身或依赖审计工具。

## 已知问题

- `.build/phase-10-20261112/replacement/` 是临时验证夹具，不在版本控制内；需要复现时按本交接第三节重建 binding 并只读挂载到容器。正式方案目录没有新增 test_only 方案。
- 替换验证只覆盖 simulation 的 `prepare` 入口；runtime 与 learning 的 `--scenario` 替换行为沿用阶段 04/05 测试。
- bundle 由 `prepare` 按目录存在性选择，模型内容校验仍属于 learning 的 `verify-bundle`/`prepare-policy`，本次未重跑。
- 模板中的 `hardware` 行只是修改位置说明；仓库仍没有已验证的真实驱动，runtime 采集也仍拒绝非 simulation 后端。第二机器人、第二后端或新学习算法均未实现。
