# P07 交付：`build/` 目录瘦身与 SDK 构建机制去冗余

任务书见 [phase-07-build-directory-standardization.md](../phase-07-build-directory-standardization.md)，总规范见 [`RULES.md`](../../../../RULES.md) 第二节。

## 结果

宿主机 `build/` 从 75 MB 降至约 22.7 MB，只保留运行时直接加载的生成 SDK：

```text
build/
├── ros-dev/
│   ├── install/           # 4 个接口包的编译产物
│   └── contracts.json     # 包集与源码摘要
└── generated/
    └── policy-sdk/
        ├── dual_arm_policy/
        └── source.sha256
```

`build/ros-dev/` 下不再出现 colcon 的 `build/`、`log/`、`colcon-home/`，也不再生成 `workspace.json`；`build/generated/policy-sdk/` 下不再出现 `_proto/`、`pyproject.toml` 或 `workspace.json`。

## 精简前后对比

| 路径 | 精简前 | 精简后 |
| --- | --- | --- |
| `build/ros-dev/build/` | 53 MB（2603 个 colcon 中间文件） | 不存在 |
| `build/ros-dev/log/` | 1.1 MB | 不存在 |
| `build/ros-dev/colcon-home/` | 存在 | 不存在 |
| `build/ros-dev/workspace.json` | 存在 | 不存在 |
| `build/ros-dev/install/` | 21 MB（3 个接口包） | 22.7 MB（4 个接口包） |
| `build/ros-dev/contracts.json` | 存在 | 存在 |
| `build/generated/policy-sdk/` | — | 48 KiB（`dual_arm_policy/` + `source.sha256`） |
| **`build/` 总计** | **约 75 MB** | **约 22.7 MB（23268 KiB）** |

`install/` 体积略增是因为一次构建同时纳入了 `marvin_handover_api_msgs`，不再需要为第二个方案重复编译。

## 实现要点

- `dev_workspace.py`：colcon 的 `--build-base`、`--log-base` 与 `COLCON_HOME` 统一放入 `tempfile.TemporaryDirectory`，构建结束（无论成败）自动删除；输出根在构建前只清理 `install/` 与旧版本可能遗留的中间目录，成功标记仍原子写入 `contracts.json`。删除 `OWNERSHIP_NAME`、`_ownership_document`、`_read_ownership`、`_ensure_workspace_owned` 与 `write_json_atomic` 的所有权写入路径。
- `sdk_packages.py`：新增 `full_selection`，收集 `common/interfaces/ros/*` 与全部 `scenarios/*/extensions/ros/*`；删除仅核心包的 `core_selection`。
- `build_ros_sdk.sh`：删除 `ROS_CORE_SDK_BUILD_ROOT` 与 `build/ros-core` 分支，所有调用统一写入 `build/ros-dev`；构建时即使传入 `--scenario` 也编译全量包集，避免用方案子集覆盖通用 SDK。
- `sdk_marker.py`：`check` 改为子集校验——所需包必须全部存在于 `contracts.json` 且角色、源码来源与源码摘要一致，允许全量构建覆盖单个方案的需求；`BUILD_COMMAND` 不再提示绑定方案。
- `generate_policy_sdk.py`：`policy.proto` 在临时目录中编译，输出目录只写 `dual_arm_policy/` 与 `source.sha256`，不再生成 `pyproject.toml`。
- `policy_sdk.py`：删除 `WORKSPACE`/`OWNER_FILE` 所有权校验与 `pyproject.toml`、`_proto` 的 legacy 识别分支，`GENERATED_NAMES` 收敛为 `{"dual_arm_policy", "source.sha256"}`。
- `pyproject.toml`：ruff 缓存改到 `/tmp/ruff-cache`，`exclude` 移除不存在的 `docs/archive`。
- 根 `README.md`、`runtime/ros2/scripts/README.md`、`shared/common/tools/scripts/README.md`、`shared/common/interfaces/ros/README.md`、`runtime/ros2/docker/README.md`、`simulation/mujoco/docker/README.md`、两个方案与模板 README 以及 `run.sh` 帮助文本同步为“一次构建、全方案通用”。
- `tests/unit/test_dev_workspace.py` 与 `tests/unit/test_container_build.py` 移除所有权守卫、保留中间目录、`workspace.json` 与 `OWNER_FILE` 的旧断言，改为验证全量选择、子集检查和临时目录清理。

## 验证

| 检查 | 结果 |
| --- | --- |
| `ruff check --no-cache .`（runtime 容器） | All checks passed |
| 全仓单元测试（runtime 容器，`tests/unit`） | 963 passed, 23 skipped |
| 全仓集成测试（simulation 容器，`tests/integration`） | 93 passed, 2 skipped |
| Markdown 相对链接扫描（排除 `third_party/`、`build/`、`var/`） | 346 links, 0 dead link(s) |
| `run.sh build-ros-sdk`（runtime 容器） | 4 个包编译成功：`robot_api_msgs`、`sim_api_msgs`、`panda_stack_api_msgs`、`marvin_handover_api_msgs` |
| `run.sh build-policy-sdk`（runtime 容器） | 输出仅 `dual_arm_policy/` 与 `source.sha256` |
| `dev_workspace.py check --scenario panda_stack` | contracts ok（全量构建覆盖方案子集） |
| `dev_workspace.py check --scenario marvin_handover` | contracts ok（`marvin_handover/binding.yaml` 的 `models.bundle` 已与 `panda_stack` 对称设为 `null`） |
| `build/ros-dev` 顶层条目 | `contracts.json`、`install/` |
| `build/generated/policy-sdk` 顶层条目 | `dual_arm_policy/`、`source.sha256` |
