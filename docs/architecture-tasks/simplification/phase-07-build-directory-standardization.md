# P07：`build/` 目录瘦身与 SDK 构建机制去冗余

总要求见 [README.md](README.md) 与 [`RULES.md`](../../../RULES.md) 第二节，进度跟踪见 [progress.md](progress.md)。

## 背景与痛点

当前 `build/` 目录下存在大量编译中间垃圾与过度防御（HERO）代码：
1. **`build/ros-dev/` 占 75 MB，其中 54 MB（72%）是无用中间文件**：
   - `build/ros-dev/build/`（53 MB CMake/C++ `.o` 中间文件）：`dev_workspace.py` 每次构建前都会先执行 `_clean_outputs` 把它删掉，既不做增量编译，运行时也完全不用，白白占用 53 MB；
   - `build/ros-dev/log/`（1.1 MB colcon 日志）和 `build/ros-dev/colcon-home/`：运行时完全不用；
   - `build/ros-dev/workspace.json`：典型的 HERO 过度防御文件（`_ensure_workspace_owned` / `_read_ownership`），为了防止“误删非本工具创建的目录”而发明了一套所有权校验。
2. **`build-ros-sdk` 按 `--scenario` 拆分构建与 `ros-core` 分支纯属多余**：
   - 全仓仅有 2 个公共接口包（`robot_api_msgs`、`sim_api_msgs`）和 2 个方案扩展包（`panda_stack_api_msgs`、`marvin_handover_api_msgs`）；
   - 当前设计导致不传 `--scenario` 时编进没人用的 `build/ros-core`，传 `--scenario panda_stack` 后跑 `marvin_handover` 又要重编；
   - 默认一次性编译 `shared/` 下全部公共与方案扩展 ROS 接口包（仍保留可选 `--scenario` 兼容参数），即可做到**构建一次、全方案通用**，并彻底删除 `ros-core` 分支。
3. **`policy-sdk` 中的冗余文件与 legacy 分支**：
   - `generate_policy_sdk.py` 把 `policy.proto` 拷贝到 `_proto/` 再编译并留在输出目录，还生成了从不被安装的 `pyproject.toml`；
   - `policy_sdk.py` 写了 `workspace.json` 所有权校验和一段 90～101 行的 `legacy` 布局兼容代码。
4. **`pyproject.toml` 中的 `build/ruff-cache` 与失效配置**：
   - `pyproject.toml` 将 `cache-dir = "build/ruff-cache"` 塞进 `build/`，且 `exclude` 中留有不存在的 `docs/archive`。

## 实施任务

### 1. 精简 `build/ros-dev/` 构建产物（砍掉 54 MB 中间垃圾与 `workspace.json`）
修改 `shared/common/tools/scripts/dev_workspace.py`、`sdk_marker.py`、`sdk_packages.py`、`build_ros_sdk.sh`：
1. **将 colcon 的 `--build-base`、`--log-base` 与 `COLCON_HOME` 放到 `/tmp` 临时目录**（并在构建结束后自动清理临时目录），或构建成功后立即删除 `build/`、`log/`、`colcon-home/`，使 `build/ros-dev/` 下**只保留 `install/` 和 `contracts.json`**。
2. **删除 `workspace.json` 所有权守卫**：
   - 从 `dev_workspace.py` 中删除 `OWNERSHIP_NAME`、`OWNERSHIP_KIND`、`_ownership_document`、`_read_ownership`、`_ensure_workspace_owned`；构建时直接清理并写入 `install/` 与 `contracts.json`。
3. **默认构建全量 ROS 接口包（一次构建，双方案通用）**：
   - 在 `sdk_packages.py` / `dev_workspace.py` / `build_ros_sdk.sh` 中：删除 `ROS_CORE_SDK_BUILD_ROOT`（`build/ros-core`）分支；当运行 `run.sh build-ros-sdk`（不传 `--scenario`，或传 `--scenario`）时，统一输出到 `build/ros-dev`，且默认收集 `shared/common/interfaces/ros/*` 与所有 `shared/scenarios/*/extensions/ros/*` 包一次性编译。`check` 时只要所需包均在已编译集合内且摘要一致即通过。

### 2. 精简 Policy SDK 构建与校验（砍掉 `_proto/`、`pyproject.toml`、`workspace.json` 与 legacy 兼容）
修改 `shared/common/tools/scripts/generate_policy_sdk.py` 与 `policy_sdk.py`：
1. `generate_policy_sdk.py`：编译完成后清理临时的 `_proto/` 目录（或在 `TemporaryDirectory` 中暂存 proto 编译），不再生成无人使用的 `pyproject.toml`；输出目录内只保留 `dual_arm_policy/` 与 `source.sha256`。
2. `policy_sdk.py`：删除 `OWNER_FILE`（`workspace.json`）、删除 `pyproject.toml` / `_proto` 相关的 `legacy` 兼容分支，`GENERATED_NAMES` 简化为 `{"dual_arm_policy", "source.sha256"}`。

### 3. 清理 `pyproject.toml` 与现有 `build/` 目录
1. `pyproject.toml`：移除 `cache-dir = "build/ruff-cache"`（或改为 `/tmp/ruff-cache`），移除 `exclude` 中不存在的 `"docs/archive"`。
2. 重新执行一次精简后的 `build-ros-sdk` 与 `build-policy-sdk`，确认宿主机 `build/` 下只包含干净的 `ros-dev/{install,contracts.json}` 与 `generated/policy-sdk/{dual_arm_policy,source.sha256}`，总体积从 75 MB 降至 ~21 MB。
3. 同步更新 `README.md`（`build-ros-sdk` 无需再绑定单一方案）与相关 SDK 单元测试。

## 完成条件与交付

- `build/ros-dev/` 下不再存在 `build/`（53 MB）、`log/`、`colcon-home/`、`workspace.json`。
- `build/generated/policy-sdk/` 下不再存在 `_proto/`、`pyproject.toml`、`workspace.json`。
- `build-ros-sdk` 一次构建即同时覆盖 `panda_stack` 与 `marvin_handover` 所需全部 ROS 接口包，不再产生 `build/ros-core`。
- 全仓 `ruff check` 零警告，Markdown 链接扫描 `0 dead link(s)`，相关单元测试与集成测试全部通过。
- 更新 [progress.md](progress.md) 中 P07 状态为“已完成”，并在 `deliveries/phase-07.md` 记录精简前后的目录与体积对比后停止。

目录导航：[总要求](README.md) · [上一阶段 P06](phase-06-var-directory-standardization.md) · [进度索引](progress.md)
