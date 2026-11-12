# 阶段 05 交接：采集、campaign 与并行 worker 接入

- 日期：2026-11-11（Asia/Shanghai）。状态：完成（代码与验证）；未进入阶段 06。
- 依据：总要求、阶段 05、progress、阶段 04 交接。未改物理、任务、训练、质量门禁、预算数值或 SDK 包集；正式台账、数据、模型与 prepared 未动。

## 入口与选择规则

容器侧新增 `runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/offline/scenario_selection.py`：复用 `robot_contracts.binding.load_binding`，选择 `recipes.runtime_teacher`、`recipes.simulation_teacher`、`recipes.campaign` 与三个 profiles，校验 collection 阶段、profile 身份与当前实现能力（`simulation/mujoco`、`ros2`、`robot_api_msgs`+`sim_api_msgs`、`GetTeacherChunk.srv`、runtime `action_source: teacher`、backend `task_mode: teacher`；并行另要求 `render_quality.enabled`）。无方案名分支。

新增 `offline/record_preflight.py`：容器内只读预检 CLI，输出单个 JSON 计划。入口改动：

- `record.py` / `record.sh`：新增 `--scenario`、`--shared-root`、`--binding-sha256`；`--config`/`--runtime-config` 默认改为 None，方案模式不再注入 Panda 默认；冲突退出 2。
- `campaign.py`：`--scenario`/`--shared-root`；`run --runtime-config` 与 `--scenario` 冲突；直接模式仍回退原 Panda recipe。
- `export_episode.py`：新增 `--scenario`/`--shared-root`，与 `--campaign-config` 冲突；`collect_attempt` 显式把选中的 campaign recipe 传给导出，质量规则不再固定 Panda 默认。
- 宿主机 `record_parallel.py`：新增 `--scenario`、host `--shared-root`；`--config`/`--runtime-config`/`--backend-template` 默认 None；方案模式冲突退出 2，且不触发 Docker。
- `record_workers.py`：新增 `--scenario`/`--shared-root`/`--binding-sha256`，与显式模板冲突；直接模式要求显式模板。
- `campaign.required_paths` 的 `GetTeacherChunk.srv` 已从 `robot_api_msgs` 修到 `shared/common/interfaces/ros/sim_api_msgs/srv/GetTeacherChunk.srv`；`sdk_marker` 仍为 `.build/ros-dev/contracts.json`。
- `campaign_ledger.tree_sha256` 源码根白名单改为 `simulation/runtime/learning/hardware/scripts/shared`；新尝试指纹不再跳过公共层，旧台账不改写。

```bash
# runtime 容器内，单路
runtime/ros2/scripts/record.sh --scenario panda_stack --count 1
# campaign 只读查询
runtime/ros2/scripts/run.sh campaign --scenario panda_stack status
# 导出
runtime/ros2/scripts/run.sh export --scenario panda_stack --episode-dir /data/raw/<id> --out /data/exports/<id>
# 宿主机并行
runtime/ros2/scripts/record_parallel.sh --scenario panda_stack --target-total 50 --workers auto
# 只读诊断预检（runtime 容器内，输出 JSON plan）
python3 -m dual_arm_runtime.offline.record_preflight --scenario panda_stack \
  --shared-root /shared --project-root /workspace --parallel
```

## 预检与资源顺序

宿主 `record_parallel` 只做 Docker 编排：先 `docker exec` runtime 执行 `record_preflight`，成功后才创建 pool 目录、生成 worker 配置、启动 simulation、进入预算/种子分配。预检顺序为 binding 与能力 → recipe/profile 身份 → campaign `required_paths`/SDK marker/磁盘 → 台账只读状态 → ROS SDK；失败返回 2，不创建 pool 目录、不预留预算、不分配种子、不启动 worker。`campaign_ledger.read_records` 使状态读取不创建目录；独立诊断仅可另行保留。`--target-total` 超正式上限在预检失败；已达到目标在预检后直接返回 0，不启动 worker。

## 宿主/容器路径规则

| 宿主 | runtime | simulation |
| --- | --- | --- |
| `<project>` | `/workspace` | — |
| `<project>/shared` | `/shared` | `/shared` |
| `<project>/var` | `/data`（也见 `/workspace/var`） | `/data` |
| `<project>/.build` | `/build`、`/workspace/.build` | `/opt/ros-dev`（仅 `.build/ros-dev`） |

宿主只接受项目内路径或项目相对路径；显式 `/shared`、`/data`、`/workspace` 作为宿主参数会被拒绝。`record_parallel` 用 `host_path_to_container`/`container_path_to_host`/`shared_root_to_container` 转换。worker manifest 混合保存 runtime 路径（`runtime_config`、`backend_log`、`output_root`）和 simulation 路径（`backend_config`、`run_root`），宿主读取 backend 日志前必须转换。

## worker 契约（`workers.json`）

顶层：`schema_version: 1`、`selection{mode,scenario_id,binding{path,sha256},recipes,…profiles,overrides}`。每个 worker：`id`、`domain_id`、`namespace`、`run_id_prefix`、`output_root`、`runtime_config`、`backend_config`、`backend_log`、`run_root`、`binding{path,sha256,scenario_id}`、`recipes{runtime_teacher,simulation_teacher}`、`profiles`、`overrides{domain_id,namespace,realtime_speed,visualization_enabled,render_quality_enabled,task_mode,action_source}`。pool 校验同一 binding 与 recipe、唯一 domain/namespace/run_id_prefix/output_root；RUNNING 记录写入这些身份，子进程环境增加 `CAMPAIGN_WORKER_ID`、`CAMPAIGN_WORKER_RUN_ID_PREFIX`、`CAMPAIGN_WORKER_OUTPUT_ROOT`、`CAMPAIGN_SCENARIO_ID`、`CAMPAIGN_BINDING_PATH`、`CAMPAIGN_BINDING_SHA256`。

count/target-total、purpose、种子分配、续采、失败计数、超时/中断取消与计费语义未变；取消项仍写 INTERRUPTED 且不删除失败记录。单路与并行失败路径仍分别报告启动、导出与停止结果。

## 验证结果

- recording/learning 相关：`108 passed, 8 skipped`（runtime 容器；skip 为无 torch）。
- runtime 全量单测：`906 passed, 25 skipped`；仅 3 个 learning 控制台测试因 runtime 无 `dual_arm_learning` 失败，同一文件在 learning 容器 `7 passed`；本次 learning 容器相关 `31 passed`。
- ruff：全部通过。
- 真实 binding 只读验证：宿主预检成功，`scenario_id=panda_stack`、binding sha256 `964a…17d`、SDK ok、campaign status 50/50；`--target-total 51` 预检返回 2 且无新 pool 目录。
- 真实两 worker 仅运行配置准备（domain 170/171，独立 namespace/run_id_prefix/output_root，binding/recipe/profile 身份一致），未启动 backend、未预留种子；诊断目录已清理。
- 证据：`.build/phase-05-20261111/`（pytest、ruff、真实 preflight JSON、两 worker manifest 与摘要、阶段 04 测试集合对比）。本轮 `/data/diagnostics/phase-05-20261111` 已清理。

## 未验证与下一阶段注意

- 未运行正式采集、真实双 worker 录制或中断回归（留阶段 11）；真实正式台账已是 50/50，任何正式补采都会停止，本阶段未触碰。
- 方案能力校验只覆盖当前 `simulation/mujoco` + `ros2` + `sim_api_msgs` Teacher；新增 backend/runtime 必须显式实现后再扩展声明。
- 替代方案通过临时 synthetic binding 验证，没有第二个真实场景。
- `interfaces.extension_ros` 真实 binding 仍为空；阶段 06 才拆 `panda_stack_api_msgs` 并更新 `GetTeacherChunk` 包路径。
- 单路直接配置模式保持原行为（仅文件/SDK 检查）；`--scenario` 模式执行完整能力与身份校验。
- 未 commit/push/stash/reset/clean。
