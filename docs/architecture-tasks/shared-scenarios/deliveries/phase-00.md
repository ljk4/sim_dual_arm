# 阶段 00 交接：基线清点与迁移归属

- 日期：2026-11-11（Asia/Shanghai）。状态：完成（只读清点）；未执行新业务、训练、完整评价或协议/路径修改。
- 本阶段交付：`baseline-files.tsv`（177 行：131 个 `contracts/libraries/infrastructure/profiles` 文件 + 22 个方案 recipe/说明 + 24 条保护模式行）、`progress.md`、本交接；证据目录 `.build/architecture-baseline-20261111/`。

## 基线

- HEAD `c8215e0f260f81e7cbdc049cf026fe0c21bf8b09`（2026-11-11 12:16:33 +08，提交信息“修改后本状态可运行闭环”）；快照时刻 `2026-11-11T12:22:40+08:00`。
- 工作树已有变更：`docs/README.md`、`docs/architecture-tasks/README.md` 修改，旧两份任务书删除，`docs/architecture-tasks/shared-scenarios/` 未跟踪。本轮只新增 `progress.md`、`baseline-files.tsv`、`deliveries/phase-00.md`；未改源码/数据/部署，未 commit/push/stash/reset/clean。
- 保护对象：`var/`（campaign 19G、数据集 205M、bundle 72M、prepared、台账）、`third_party/`、`.build/`、`tests/` 夹具、模型/数据/历史产物。`git status --ignored` 仅见 `infrastructure/scripts/__pycache__/`。

## 关键决定

- 阶段 01 目标按任务书映射：`contracts/policy|ros|schemas` → `shared/common/interfaces/...`；`contracts/python` → `shared/common/python/contracts`；三个 `libraries/*` → `shared/common/python/{trajectory,ros_support,run_logging}`；`infrastructure/{docker,scripts}` → `shared/common/tools`；`profiles` → `shared/scenarios/panda_stack/profiles`。包名、`pyproject + src` 结构和导入名保持不变。
- 聚合 README 需重写：`contracts/README.md` → `shared/common/README.md`，`libraries/README.md` → `shared/common/python/README.md`；其余包/目录 README 随目录移动。
- `contracts/schemas/v1/runtime_config.schema.json` 唯一归属为 Runtime 私有格式，建议阶段 03 撤下：Schema 仍要求旧 `script_targets`，缺 `control_constraints/reset/observation/control/action_source/follower_config/policy/teacher`，无代码调用方（仅 README/任务书引用）。
- `GetTeacherChunk.srv` 最终归属 `shared/scenarios/panda_stack/extensions/ros/panda_stack_api_msgs`（阶段 06）；阶段 01 随 `sim_api_msgs` 整体暂存，阶段 06 才拆包，固定 `60x16`、`phase_codes` 属堆叠方案，不得冒充通用 Teacher 协议。
- 方案 recipe 阶段 01 暂留四端（本阶段未搬移），阶段 03 统一迁入 `shared/scenarios/panda_stack/configs/{simulation,runtime,learning}`；阶段 04、05 分别接入学习与采集入口。`container.env`、Dockerfile/Compose、端 `run.sh` 与 `scripts/README.md` 保持端私有。`fake_policy.yaml` 保留测试归属，不进入正式方案 recipe。
- Profile/observation/action/control 的数值与语义摘要作为纯移动对照基线：`panda_stack_v1`=a95b0f81…、`panda_stack_learning_v1`=9e0b5b5f…、observation=452d8a15…、action=93689bc5…；具体文件 SHA-256 见 TSV。

## 待核对事实（全部已按当前源码复核）

1. `contracts`：1 个 Policy proto、2 个 ROS 包、7 份 JSON Schema、15 个 Python 契约模块。2. `robot_api_msgs`=15 msg/9 srv；`sim_api_msgs`=1 msg/5 srv。
3. 公共 Python 包未导入四端实现；唯一工具级耦合是 `dev_workspace.py` 检查 `dual_arm_runtime` 路径，`profiles/control/*.yaml` 仅在注释中提 `robot_backend_sim` 名称。
4. `GetTeacherChunk.srv`：`float64[960]`、`int64[60]`、`uint8[60]`，phase 0–8 固定为堆叠阶段。5. `ExecutionFeedback.msg` 为 `GripperFeedback[2]`，契约 `grippers` min=max=2、`side` 仅 left/right、finger 1/2 固定。
6. `FOLLOWER_ALGORITHM_VERSION = "bounded-target-follower/ruckig-0.15.3/v1"`，`FollowerReference` 对 algorithm/sampler/normalization 版本做严格相等校验。
7. `runtime_config.schema.json` 与 `RuntimeRecipe`/模板不一致（见上）。8. `dev_workspace.py` 固定 `CONTRACT_SOURCE=contracts/ros`、两个必需包与 `dual_arm_runtime` 检查路径。
9. `check_dockerfiles.py` 要求 simulation 使用 `/app`、`/contracts`、`/profiles` 等固定挂载，且 `runtime: nvidia`/`gpus: all`。10. 根 `pyproject.toml`、三份 `container.env`、`prepare_policy.py`、`record_parallel.py`、`record_workers.py` 均含旧公共路径。
11. `stack_campaign.yaml.required_paths` 仍写 `contracts/ros/robot_api_msgs/srv/GetTeacherChunk.srv`（实际文件在 `sim_api_msgs`）；`evaluate.py` 默认 `configs/experiments/campaign.yaml` 不存在。12. `hardware/` 仅一份边界 README，无真实驱动，不能声称真机验证。
- 文档漂移：根 `README.md:308` 与 `libraries/trajectory/README.md:7` 仍指向不存在的 `docs/architecture-tasks/implementation-2026-11-10.md`；本轮未改，留阶段 12。

## 环境与资源

- 三个容器 `sim-dual-arm-{simulation,runtime,learning}-1` 已运行约 2 小时但空闲：仅 `sleep infinity` 与交互 bash，无业务进程/监听端口。挂载仍为旧路径：simulation `/app`、`/contracts`、`/profiles`、`/deps/*`、`/config/*`；runtime/learning 仓库根 `/workspace:ro`、`var→/data`、`.build→/build`。
- 容器可用环境：runtime=ROS 2 Jazzy、rosbag2/mcap、pytest 8.3.5、pydantic 2.10.6，`robot_*` 与两 ROS 包导入通过；simulation=MuJoCo 3.17.0、pytest 7.4.4、pydantic 2.10.6，导入通过；learning=torch 2.15.0+cu132（CUDA 可用）、pytest 9.2.0、pydantic 2.14.3（高于 runtime 锁定版本，阶段 04/11 注意）。
- 宿主 Python 3.12.3（无 pydantic）；`/data` 不存在（仅容器内映射）；磁盘剩余 29G（88% 已用）；GPU RTX 4050 6GB。Compose 三份静态解析通过，九个相关 shell 脚本 `bash -n` 通过。
- 可用产物：`var/learning/bundles/stack-act`（72M，ACT 5000 步，60×16，三相机）、`var/learning/panda_stack_learning_v1`（205M，complete）、`var/prepared/infer-5k-server` 与 `teacher-backend`。可隔离使用三个空闲容器、GPU 与新建诊断目录；不复制/覆盖产物，不挪用正式台账/预算/评价集。

## 测试基线（现有证据，非当前通过）

- `.pytest_cache` 时间 2026-11-11 10:43，158 个 nodeids，`lastfailed={}`；早于 HEAD 提交 12:16，命令与选择集未记录，且共享 `test.sh` 使用 `-p no:cacheprovider`，不能据此声称当前 HEAD 通过。
- `.build/ros-dev/contracts.json`（2026-11-10 19:09）SHA-256 `b9092f8f…` 与当前 `contracts/ros` 41 文件重算值一致；`.build/generated/policy-sdk/source.sha256` 与当前 proto SHA-256 `22955bd8…` 一致。仅证明 SDK 相对当前契约源码不陈旧，不是测试通过。
- 后续基线命令（本阶段未运行）：`runtime/ros2/scripts/run.sh test -q tests/unit`；`learning/lerobot/scripts/run.sh test -q tests/unit`；simulation 标准挂载不含 `tests/`（公共工具已归入 `shared/`），需额外完整仓库只读挂载后再跑，不能直接用 `scripts/run.sh test` 声称覆盖。

## 历史 episode（读取验证留阶段 11）

- 选中 `var/campaigns/panda_stack_v1/attempts/formal-10014-record-1794304035694470020-worker-03/`：manifest schema_version=1，任务 succeeded，profile/observation/action hash 分别为 9e0b5b5f/452d8a15/93689bc5；原始 MCAP 351,799,521 B，`recording/recording_0.mcap` + `metadata.yaml`。
- 已有导出在 `stack/record-1794304035694470020-worker-03/export/`：summary 记录 observations=495、labels=470、raw_predictions=48、quality_qualified=true；未在本阶段重新读取或修改。
- 读取所需环境：runtime 容器（ROS 2 Jazzy + rosbag2_py/mcap + 两 ROS 包 SDK + `robot_contracts`/`robot_ros_support`），入口 `runtime/ros2/scripts/run.sh export --episode-dir <raw> --out <new-dir>`；新读取验证留阶段 11。

## 各阶段入口与风险

- 01：`pyproject.toml` pythonpath、三份 `container.env`、三份 Compose、三端 `run.sh` 根推导、`prepare_backend.py`/`prepare_run.py`/`prepare_policy.py`、`dev_workspace.py`/`policy_sdk.py`/`generate_policy_sdk.py`/`test.sh`、`runtime_config.py:312`、`export_episode.py:353/357`、`record_workers.py:21-23`、`campaign_ledger.py`/`collect_artifacts.py` 的根目录白名单、`tests/` 路径。
- 02：现有加载/校验入口 `contracts/python/src/robot_contracts/{profiles,validation}.py`；新 binding API 尚无实现。03：`simulation/mujoco/scripts/prepare_backend.py`、`run.sh prepare/backend/viewer/scene/kinematics`、`runtime/.../offline/prepare_run.py`、`runtime_config.py`、配方文件。
- 04：`learning/lerobot/scripts/prepare_policy.py`、`dual_arm_learning/{policy_config,policy_service,bundle,cli}.py`。05：`record.sh`/`record_parallel.sh`、`offline/{record,record_pool,record_workers,campaign*}.py`、`stack_campaign.yaml`。
- 06：`dev_workspace.py`、`contracts/ros/*`、runtime `task_client.py`、simulation `ros/node.py`。07：`execution_feedback.py`、`ExecutionFeedback/FingerFeedback/GripperFeedback.msg`、`stream_conversions.py`。08：`robot_contracts/trajectory/*`、`robot_trajectory/*`、`FOLLOWER_ALGORITHM_VERSION`。
- 09：`check_dockerfiles.py` 与三份 Compose/`container.env`。10：尚需 `shared/scenarios/_template` 与 `binding.example.yaml`。11：`offline/mcap_reader.py`、`export_episode.py`、集成测试与上述历史 episode。12：根 `README.md`、docs 索引、旧路径清理。
- 既有失败/风险：5K ACT 的 `FOLLOWER_DEVIATION_EXCEEDED`/`TRACKING_TIMEOUT`、右腕遮挡质量门禁属业务问题，不得借迁移修复；旧容器/SDK 在移动后失效；磁盘仅 29G，禁止复制 19G campaign；`/data` 宿主不存在；标准 simulation 挂载不能跑全仓测试。

## 未运行项与下一阶段注意

- 未运行：pytest（单元/集成）、ruff、SDK 重建、prepare、训练/推理、历史 MCAP 重读、任何业务容器启停。原因：阶段 00 只读清点；现有证据不足以声称当前 HEAD 通过，均已标注“未验证”。
- 下一阶段先读本交接与 `baseline-files.tsv` 相关行；保持工作树已有 docs 修改和未跟踪任务书；阶段 01 移动后必须同步更新全部活跃路径、重建/重挂容器与 SDK，并记录过渡项，不得用软链接或隐藏 fallback。
