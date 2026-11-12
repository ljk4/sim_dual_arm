# 阶段 03 交接：方案配方与仿真、运行端接入

- 日期：2026-11-11（Asia/Shanghai）。状态：完成（代码与验证）；未进入阶段 04。
- 依据：总要求、阶段 03、progress、阶段 02 交接。环境：既有三容器，未重建镜像/SDK；未运行完整训练、正式采集或完整评价。

## 配方迁移
22 个文件（18 个 YAML + 4 个 README）从四端迁入 `shared/scenarios/panda_stack/configs/`：

- `simulation/{backend,scene,checks}/`（4+3+2 个）、`runtime/`（teacher_collect、policy_rollout、experiments/stack_campaign）、`learning/`（6 个）。

`container.env`、Dockerfile、依赖锁、`fake_policy.yaml` 与历史测试夹具保持原归属。学习端与采集端只更新现有引用/默认路径；其 `--scenario` 接入分别留阶段 04/05。

迁移前后 18 个 recipe 的路径无关业务参数完全一致（`.build/phase-03-20261111/move-verification.json`）；11 个文件仅因内部相对路径或注释变化而原始 SHA 改变，7 个原始字节不变。三个 Profile 语义哈希保持 `9e0b…`、`452d…`、`9368…`。

## 真实 binding
`shared/scenarios/panda_stack/binding.yaml`：

- `backend.kind=simulation/mujoco/task_type=stack`；`runtime.implementation=ros2`；`learning.implementation=lerobot,adapter=act`。
- profiles 选择 `panda_stack_learning_v1` 与 `panda_rgb_joint_v1`、`panda_joint_position_v1`。
- recipes 八项均指向已迁入文件；`simulation_policy` 选择 `backend_stack_act_chunked.yaml`（`backend_stack_inference.yaml` 保留为直接模式变体）。
- `interfaces.common_ros=[robot_api_msgs, sim_api_msgs]`，`extension_ros=[]`，`policy_api=bimanual.policy.v1`；未提前引用扩展包。
- `models.bundle=null`；`source_roots=["../.."]`（共享根）；`artifact_roots=["/data"]`（三容器实际可写产物根）。因此真实 binding 是部署态文件，应在项目容器内校验；宿主机无 `/data` 时按设计失败。
- collection/learning 阶段通过；inference 缺模型报 `BINDING_STAGE … missing … models.bundle`；`--bundle /data/learning/bundles/stack-act` 后通过并记录 `source=override`。

## 入口与命令
simulation `scripts/run.sh prepare`：
```bash
# 方案选择（与 --template 互斥）
scripts/run.sh prepare --scenario panda_stack --role teacher \
  --seed 0 --run-id prep-teacher --output /data/prepared/prep-teacher
scripts/run.sh prepare --scenario panda_stack --role policy \
  --bundle /data/learning/bundles/<verified-bundle> \
  --seed 20001 --run-id prep-policy --output /data/prepared/prep-policy
# 直接配置模式保留
scripts/run.sh prepare --template /shared/scenarios/panda_stack/configs/simulation/backend/backend_stack_inference.yaml ...
scripts/run.sh backend --config /data/prepared/prep-policy/backend.yaml --run-id backend-1
```

runtime `scripts/run.sh prepare`：
```bash
runtime/ros2/scripts/run.sh prepare --scenario panda_stack --role policy \
  --policy-binding /data/prepared/<policy-run>/policy/policy_binding.json \
  --bundle /data/learning/bundles/<verified-bundle> \
  --seed 20001 --run-id policy-run --output /data/prepared/policy-run
runtime/ros2/scripts/run.sh prepare --scenario panda_stack --role teacher \
  --seed 1000 --run-id teacher-run --output /data/prepared/teacher-run
runtime/ros2/scripts/run.sh policy --config /data/prepared/policy-run/runtime.yaml
```

`--scenario <id>` 解析 `<shared-root>/scenarios/<id>/binding.yaml`；`--shared-root` 缺省取 `SHARED_ROOT` 或布局推导。`--template`/`--scenario`、Teacher+`--policy-binding`、`--bundle` 用于 Teacher、缺 Policy binding 等冲突均有明确错误。运行入口（backend/viewer/policy/interactive）只消费 prepared 配置，不提供绕过准备的 `--scenario`。

## Prepared Manifest 字段

simulation 与 runtime 新写入的 `run_manifest.json`（`schema_version=1`，kind 不变）新增：
- `selection`：`mode=scenario|direct`、`scenario_id`、`binding{path,sha256}`、`recipe{field,declared,path,sha256}`；policy role 另含 `bundle{declared,effective,source,path,exists,digest,validated_by}`。
- `overrides`：`seed{value,source,target}`、`run_id`、`output`；scenario policy 另含 `bundle{source,path}`。
- `inputs`：原有 `template`/`template_sha256`（policy 另有 `policy_binding`/sha），新增实际 `bundle` 摘要对象。
- runtime Teacher 的 `policy` 为 `null`；Policy 的既有身份字段不变。旧 `var/prepared/` 未改写。

## 模型来源一致性（补充）
- scenario policy prepare 要求 `policy_binding.model.bundle` 存在，并与 binding 的有效 bundle（`--bundle` 或 `models.bundle`）指向同一真实目录：`resolve(strict=True)` 后用 `os.path.samefile` 比较，符号链接、相对路径与 `/workspace/var` ↔ `/data` bind-mount 别名均可。
- 不同目录一律拒绝；Profile/观测/动作相同不构成放行理由，runtime 不重新校验模型内容。缺 `model.bundle`、目录不存在、scenario 无有效 bundle 均明确失败，且全部发生在创建输出目录之前。
- 保持 ROS-free：不导入 learning/Torch、不加载权重、不新增模型格式；直接配置模式与旧 prepared 不变。

## 验证结果
- simulation：direct/scenario teacher/scenario policy prepared 后均被真实 `load_backend_config` 接受；替代 recipe 将 `realtime_speed` 1.0 改为 0.4 后 prepared `backend.yaml` 实际变为 0.4。
- runtime：direct/scenario policy 与 scenario teacher prepared 后均被真实 `load_runtime_configuration` 接受；替代 recipe 将 `lease_duration_ns` 2 s 改为 3 s 后 prepared `runtime.yaml` 实际变为 3 s。
- 身份链：learning `prepare-policy --bundle /data/learning/bundles/stack-act` 成功；新 `policy_binding.json` 的 profile/observation/action 哈希与 binding 一致；篡改 semantic hash 后 runtime prepare 拒绝。
- cwd：simulation `/app` vs `/tmp`、runtime `/workspace` vs `/tmp` 的 prepared YAML 字节一致；manifest 除 output 路径外一致。
- 故障：参数冲突（template+scenario、缺 role、Teacher+bundle/binding、Policy 缺 binding）、未知 scenario、缺失 recipe 文件、profile 不一致、缺模型均返回明确错误。
- 模型来源补充回归：runtime 68 passed、learning 10 passed；真实 A binding + B bundle 复现被拒且无输出；同目录/符号链接/bind-mount 别名成功；缺字段/缺目录失败且无输出（`model-identity-verification.json`）。
- 测试：runtime 213 passed；learning 180 passed；simulation 配置单测 107 passed（runtime 容器内执行，`tests/unit/test_backend_config.py` 等）；`ruff check` 变更 Python 文件全通过。
- 旧引用：活跃源码/测试/README 中无旧 recipe 路径；学习/采集默认路径均可解析；Markdown 链接检查仅剩既有 `docs/architecture-tasks/implementation-2026-11-10.md` 缺失。

## 决定、未完成项与下一阶段
- `runtime_config.schema.json` 无代码调用且已漂移，已从公共 Schema 撤下；Runtime 由 `RuntimeRecipe` 严格校验，未新增重复私有 Schema。
- 真实 binding 的 `/data` artifact root 使其仅适用于部署容器；测试用替代 shared root 不受影响。
- `policy_binding.json` 仍由学习端真实 bundle 校验生成，scenario binding 不替代其 digest；修正 phase-01 遗留的 simulation 检查默认路径；campaign GetTeacherChunk 包漂移留阶段 05。
- 未实现学习端/采集端 `--scenario`，未跑 ROS/MuJoCo 闭环、训练、正式采集与完整评价。
- 证据：`.build/phase-03-20261111/`（`before/`、`move-verification.json`、`verification-summary.json`、`model-identity-verification.json`、`binding-*.json`、`evidence/`、pytest/ruff 日志）；本轮 `/data/diagnostics/phase-03*` 已清理。
