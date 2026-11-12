# 阶段 01 交接：公共目录、档案与容器路径迁移

- 日期：2026-11-11（Asia/Shanghai）。状态：完成（迁移与验证）；未进入阶段 02。
- 依据：总要求、阶段 01、progress、阶段 00 交接。先更正阶段 00 两处计划偏差：Teacher 最终路径为 `shared/scenarios/panda_stack/extensions/ros/panda_stack_api_msgs`（阶段 06 拆包）；实例 recipe 阶段 03 统一迁移、阶段 04/05 分别接入学习/采集入口。基线源路径、内容摘要与 SHA 未改。
- 环境：HEAD `c8215e0` + 工作树 docs 变更；三容器迁移前空闲（仅 `sleep infinity`），迁移中未运行其他业务；宿主无 `/data`，未创建。

## 实际完成

- 物理归并 129 个文件，无软链接；除下述 README 与工具脚本外，内容字节一致：
  - `contracts/policy|ros|schemas` → `shared/common/interfaces/{policy,ros,schemas}`
  - `contracts/python` → `shared/common/python/contracts`
  - `libraries/{trajectory,ros_support,run_logging}` → `shared/common/python/...`
  - `infrastructure/{docker,scripts}` → `shared/common/tools/{docker,scripts}`
  - `profiles` → `shared/scenarios/panda_stack/profiles`
  - 聚合 README → `shared/common/README.md`、`shared/common/python/README.md`、`shared/common/tools/README.md`
  - 旧 `infrastructure/scripts/__pycache__/` 两个生成缓存按基线 `skip-generated` 未迁移并删除；非数据/模型/台账。
- 路径修复：根 `pyproject.toml` pythonpath；三份 `container.env`（`SHARED_ROOT=/shared`、`PROJECT_SETUP`、`PROJECT_SOURCE_PATHS`）；三份 Compose（新增 `shared → /shared:ro`，删除 `/contracts`、`/profiles`、`/deps/*`、`/config/*`）；三端 `run.sh`、`record.sh`、`record_parallel.py`、SDK 工具、`check_dockerfiles.py`、runtime 的 config/export/collect/record_workers、learning `prepare_policy.py`、四端 recipe 公共引用、测试与夹具、活跃文档、`.gitignore`/`.dockerignore`。
- 容器：三端 `up -d --force-recreate --no-build --pull never` 显式重建；镜像未重建（依赖未变）。`/shared` 只读可见，旧公共挂载不存在。
- SDK：ROS SDK 用新入口显式重建（41 文件，digest `d9b1fcf8…`，marker source=`shared/common/interfaces/ros`）；Policy SDK 源 proto 内容未变，校验通过（`22955bd8…`），未重建。

## 关键入口

- 容器内：simulation `/app` + `/shared`；runtime/learning `/workspace` + `/shared`；三端 `SHARED_ROOT=/shared`。
- simulation：`scripts/run.sh test` → `/shared/common/tools/scripts/test.sh`；后端启动前 SDK 校验 → `/shared/common/tools/scripts/dev_workspace.py`。
- runtime：`run.sh build-ros-sdk|build-policy-sdk|test` 均走 `/shared/common/tools/scripts`；Python 源码含 `/shared/common/python/*` 与 `/workspace/runtime/ros2/ros/dual_arm_runtime`。
- learning：`run.sh test` 同上；`PROJECT_SOURCE_PATHS` 仅本端、contracts、run_logging、SDK，未加载 ROS/轨迹。
- 宿主：`shared/common/interfaces/...`、`shared/common/python/...`、`shared/common/tools/...`、`shared/scenarios/panda_stack/profiles/...`。

## 验证（迁移前 → 迁移后，同一选择集）

| 环境 | 命令 | 迁移前 | 迁移后 |
| --- | --- | --- | --- |
| runtime | `runtime/ros2/scripts/run.sh test -q tests/unit` | 793 passed / 3 failed / 17 skipped | 相同 |
| learning | `learning/lerobot/scripts/run.sh test -q tests/unit` | 862 passed / 6 skipped | 相同 |
| simulation | 临时只读全仓挂载（`repo→/workspace:ro`、`shared→/shared:ro`）+ `scripts/run.sh test -q`（18 个迁移相关测试文件，见 `selection.txt`） | 377 passed / 1 skipped | 相同 |

- runtime 的 3 个失败为既有环境差异（`dual_arm_learning` 不在 runtime PYTHONPATH），迁移前后一致，未修改业务。
- 静态：3 份 Compose `config -q`、9 个 shell `bash -n`、`check_dockerfiles.py`（3 Dockerfiles/3 env）、`ruff check shared simulation runtime learning tests` 全部通过。
- 旧路径扫描覆盖 Python/shell/YAML/JSON/IDL/Proto/CMake/package.xml/文档；剩余匹配仅为新 `shared/common/python/contracts` 链接与 `collect_artifacts` 历史清单前缀兼容，不是旧根入口。证据：`post-active-path-scan.txt`。
- 语义：四个 Profile/observation/action 哈希与阶段 00 完全一致：`a95b0f81…`、`9e0b5b5f…`、`452d8a15…`、`93689bc5…`；recipe 与 HEAD 的 diff 仅路径/注释行，业务数值未变。
- 解析：4 个 simulation backend 模板 `prepare` 后所有引用存在；runtime `teacher_collect.yaml` 加载通过，`policy_rollout.yaml` 替换两个 `FROM_VERIFIED_ACT_BUNDLE` 占位符后加载通过；learning dataset/training 加载通过，`policy_temporal.yaml` 三个档案路径解析存在（`/opt/act-bundle` 缺失属模板占位行为）。
- campaign `required_paths` 8 项，仅已知 `robot_api_msgs/...GetTeacherChunk.srv` 包漂移缺失（阶段 05 修）。
- 保护：`var/` 27G 在迁移期间无新文件；未复制数据；未 commit/push/stash/reset/clean；旧目录已移除，`shared/` 无软链接。

## 未验证 / 过渡项

- 未运行 ROS/MuJoCo 集成测试、backend 启停、record/campaign、训练/推理、历史 MCAP 重读；留阶段 11。
- 未重建镜像；未跑 simulation 完整单元集（标准容器不挂 `tests/`，仅用临时只读全仓挂载跑迁移相关选择集）。
- recipe 仍留四端，阶段 03 统一迁移，阶段 04/05 分别接学习/采集入口。
- `GetTeacherChunk.srv` 仍随 `sim_api_msgs`，阶段 06 拆至 `extensions/ros/panda_stack_api_msgs`；本阶段未改载荷。
- `collect_artifacts.contracts_digest` 同时识别历史 `contracts/` 与新 `shared/common/interfaces/` 清单前缀，仅兼容旧 manifest，不是路径转发。
- `runtime_config.schema.json` 仍在原 Schema 目录（阶段 03 处理）；`shared/scenarios/panda_stack` 暂无 binding/configs/extensions（后续阶段）。
- 文档仅更新活跃入口；根 README 的旧 `implementation-2026-11-10.md` 链接等最终收口留阶段 12。

## 证据与下一阶段注意

- 证据：`.build/phase-01-20261111/`（前后 SHA、`selection.txt` 选择集与命令、三端测试日志、静态检查、SDK/容器/recipe 解析记录）。
- 阶段 02 先读本交接；Profile/Schema 已位于 `shared/`，recipe 仍在四端，新 binding API 尚无实现。不要恢复旧挂载、旧路径转发包或软链接。
