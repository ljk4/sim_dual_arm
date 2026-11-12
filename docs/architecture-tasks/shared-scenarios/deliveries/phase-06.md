# 阶段 06 交接：可选 SDK 与 Panda Teacher 扩展

- 日期：2026-11-11（Asia/Shanghai）。状态：完成（代码与验证）；未进入阶段 07。
- 依据：总要求、阶段 06、progress、阶段 05 交接。未改物理、任务判据、训练/质量/预算数值、正式台账、历史 MCAP、模型或 prepared。

## 接口边界

- `GetTeacherChunk.srv` 从 `sim_api_msgs` 迁至 `shared/scenarios/panda_stack/extensions/ros/panda_stack_api_msgs/srv/`；载荷、60×16、50 ms、阶段码 0–8 与 IK 诊断字段未改。
- 新包 `panda_stack_api_msgs`（0.1.0，ament_cmake）含 `GetTeacherChunk.srv` 与 `TeacherCapability.msg`；能力标识 `panda_stack_api_msgs.teacher_chunk.v1`。
- `sim_api_msgs` 保留 `ResetSimulation`、`PrepareTask`、`StartTask`、`GetTaskState` 与 `TaskState`。服务名仍为 `<ns>/get_task_chunk`；旧同名类型不再生成。
- 旧能力 `teacher_chunk` 不再被接受：runtime 要求 binding/recipe 的 `teacher_api`，并校验 backend `BackendInfo.capabilities` 含同一标识。

## 适配与选择

- simulation：`ros/teacher_extension.py` 只在 `worker.teacher_enabled` 时导入扩展并注册服务；`node.py` 顶层不再导入专用包。
- runtime：`task_client.py` 只保留 `sim_api_msgs` 的 prepare/start/state；新 `teacher_client.py` 按能力标识延迟解析 `panda_stack_api_msgs` 并实现 chunk 客户端；`teacher_source.py` 改用该适配。
- runtime recipe 新增 `teacher_api`（teacher 模式必填）；`prepare_run --scenario` 从 binding 注入并校验 recipe 一致。runtime `package.xml` 移除 `sim_api_msgs` 强制 `exec_depend`，可选包不再由安装声明强制。
- binding：`interfaces.extension_ros: [extensions/ros/panda_stack_api_msgs]`、`interfaces.teacher_api: panda_stack_api_msgs.teacher_chunk.v1`；`robot_contracts.binding.InterfacesSpec` 增加可选 `teacher_api`，公共解析器无方案名分支。
- `scenario_selection` 按扩展包 `package.xml`/`TeacherCapability.msg`/`srv/GetTeacherChunk.srv` 验证能力，不按 scenario id 分支；worker manifest 每项新增 `teacher_api`，RUNNING 记录与子进程环境同步。
- `campaign.required_paths` 指向扩展包 service 与 capability 文件；`record`、`record_workers`、`record_pool`、`collect_attempt`、`record_preflight` 的 SDK 检查在 scenario 模式按选中包集精确校验。

## SDK 工具

- 新 `shared/common/tools/scripts/sdk_packages.py`：binding/显式包选择、package.xml 依赖闭包、重复包名/缺包/目录名不符拒绝、README 排除的稳定摘要。`sdk_marker.py` 负责 marker 生成、包集/摘要/模块来源校验与状态输出；`dev_workspace.py` 保留所有权、锁、colcon 构建和 CLI（363/363/429 行）。
- `dev_workspace.py`：无选择构建仅核心 `robot_api_msgs`（`build_ros_sdk.sh` 默认写 `/build/ros-core`）；`--scenario`/`--package` 选择完整包集写 `/build/ros-dev`。`check/status` 无选择时校验 marker 记录的包集，不把完整 SDK 降级成核心。
- marker（`contracts.json`，schema 2）：`kind`、`selection{kind,scenario_id,binding{path,sha256},packages}`、`packages{<name>:{role,source,files,sha256,module_root}}`、`package_set_sha256`、`environment{python,ros,image,workspace_version}`、`install{directory,setup}`、`finished_at`。
- 摘要覆盖 `.msg/.srv/.action`、`CMakeLists.txt`、`package.xml` 等非 README 文件；README 变化不失效。包集、IDL、构建元数据或源码变化使 marker 失效。校验拒绝旧 schema、包集不符、缺模块/旧模块、重复模块来源、绝对或越界 source。
- `build_ros_sdk.sh`：无 `--scenario/--package` 写 `ROS_CORE_SDK_BUILD_ROOT`（默认 `/build/ros-core`）；有选择写 `ROS_SDK_BUILD_ROOT`（默认 `/build/ros-dev`）。

## 命令与结果

```bash
# runtime 容器
python3 /shared/common/tools/scripts/dev_workspace.py build \
  --project-root /workspace --shared-root /shared --build-root /build/ros-core
python3 /shared/common/tools/scripts/dev_workspace.py build \
  --project-root /workspace --shared-root /shared --scenario panda_stack \
  --build-root /build/ros-dev
runtime/ros2/scripts/run.sh build-ros-sdk --scenario panda_stack --shared-root /shared
python3 -m dual_arm_runtime.offline.record_preflight --scenario panda_stack \
  --shared-root /shared --project-root /workspace --parallel
python3 /shared/common/tools/scripts/policy_sdk.py build --project-root /workspace \
  --sdk-root /build/generated/policy-sdk
```

- 核心构建：1 包 `robot_api_msgs`，`package_set_sha256=69cabb7c…`，`.build/ros-core`。
- 完整构建：3 包 `robot_api_msgs`、`sim_api_msgs`、`panda_stack_api_msgs`，`package_set_sha256=b4a45ebf…`；runtime 与 simulation 容器对 `/build/ros-dev`/`/opt/ros-dev` 校验一致。
- 核心隔离：只 source `/opt/ros/jazzy` 与 `/build/ros-core` 的 Python 路径，导入 `robot_api_msgs`、`robot_control_client` 与 `teacher_client` 模块成功；`sim_api_msgs`、`panda_stack_api_msgs` 均不可导入，普通控制 7 个客户端不要求可选包。日志 `.build/phase-06-20261111/core-isolation.log`。
- 核心冒充完整：`check --scenario panda_stack` 对 core marker 返回 1（包集不符）；预检在 `ROS_SDK_ROOT=/build/ros-core` 下返回 2，`var/record-pools` 数量 5→5，未创建目录、未预留预算/启动后端。
- 真实预检：exit 0，SDK ok，`teacher= panda_stack_api_msgs.teacher_chunk.v1`，campaign 50/50；输出 `.build/phase-06-20261111/preflight-real.json`。
- Teacher 类型：runtime `tests/integration/test_teacher_extension_ros.py` 2 passed；simulation 容器内 `register_teacher_service` 与 `GetTeacherChunk` 类型断言通过；旧 `teacher_chunk` 与 `sim_api_msgs.GetTeacherChunk` 明确拒绝。
- 单测：runtime 全量 `915 passed, 25 skipped, 3 failed`；3 个失败均为 `test_learning_console_progress.py` 在 runtime 无 `dual_arm_learning`，同文件 learning 容器 `7 passed`；learning 相关 29 passed。ruff 全部通过。
- Policy SDK：proto 配方性注释迁移后旧 marker 报 `POLICY_SDK_STALE`，正常重建后 `source_sha256=f1f2f6ee…` 校验通过；wire 字段/编号/主版本未变。
- 证据目录：`.build/phase-06-20261111/`（核心/完整构建日志、核心隔离、marker、预检 JSON、pytest、ruff、历史 MCAP topic 清单）。本轮 `/data/diagnostics/phase-06-20261111` 已清理；未运行正式采集。

## 历史兼容边界

- 旧 MCAP 不含 Teacher 类型（实测 topic 仅 `robot_api_msgs/*` 与 `sim_api_msgs/msg/TaskState`），`TaskState` 未迁移，旧 `offline/mcap_reader` 读取路径不受影响；未改写任何历史数据。
- 旧 schema 1 marker 被新工具拒绝（旧 marker 副本在证据目录）；旧 `sim_api_msgs.srv.GetTeacherChunk` 模块不再生成，旧客户端/服务端同名类型无法互连。
- 旧 prepared teacher `runtime.yaml` 若缺 `teacher_api` 会被新 runtime 拒绝；当前 `var/prepared` 无此类 teacher 配置。旧 policy prepared 配置不受影响。
- `sim_api_msgs` 不再由 runtime `package.xml` 强制安装；使用任务/Teacher 的配方必须按 binding 构建完整 SDK。

## 未验证与下一阶段注意

- 未启动真实 simulation teacher 后端执行 `get_task_chunk` 端到端调用；仅完成生成类型、能力标识、SDK 与预检验证。建议阶段 07/11 在仿真容器内做一次 teacher 后端启动与 service 类型匹配。
- 核心 SDK 默认根 `/build/ros-core` 未加入 Compose 挂载；服务启动仍使用 `/build/ros-dev`。核心验证通过显式隔离 PYTHONPATH 完成。
- 直接配置模式（无 `--scenario`）的 SDK 检查仍只校验 marker 记录包集；完整能力由 recipe `teacher_api` 与运行时类型导入保证。
- 未 commit/push/stash/reset/clean。
