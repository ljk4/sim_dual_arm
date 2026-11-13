# P00 交付记录：基线与实施准备

日期：2026-11-12。实施开始时间：14:46 +08:00。代码基线为 `main` / `55bf341983b985c4746ae3ba245ee00c4f25e6dd`；开始工作树干净。本记录描述本轮实际运行的命令与结果；未跟踪文件、构建产物和运行产物均未提交。

## 代码与工作树

- 仓库：`/home/ljk/work/sim_dual_arm`，分支 `main`，HEAD `55bf341983b985c4746ae3ba245ee00c4f25e6dd`。
- 开始状态：无修改、无删除、无未跟踪文件。
- 本轮新增/修改保留在工作树，不暂存、不提交、不推送。P00 结束时未改动 Panda 业务参数。
- 会话日志：`var/marvin-handover/`；Panda 检查报告：`var/reports/sim-check/`；试采数据：`var/campaigns/panda_stack_v1/`。

## 容器、挂载与依赖

三个容器均由现行 Compose 启动，镜像为 `sim-dual-arm/{simulation,runtime,learning}:current`，运行用户 `1000:1000`。

| 容器 | 关键挂载 | 说明 |
| --- | --- | --- |
| `simulation` | 仓库 `simulation/mujoco` → `/app:ro`；`shared` → `/shared:ro`；`var` → `/data`；`build/ros-dev` → `/opt/ros-dev:ro` | NVIDIA runtime，`MUJOCO_GL=egl` |
| `runtime` | 仓库 → `/workspace:ro`；`shared` → `/shared:ro`；`var` → `/workspace/var` 与 `/data`；`build` → `/workspace/build` 与 `/build` | ROS 2 Jazzy |
| `learning` | 仓库 → `/workspace:ro`；`shared` → `/shared:ro`；`var` → `/workspace/var` 与 `/data`；`build` → `/workspace/build` 与 `/build`；`third_party` → `/third_party:ro` | ROS-free |

宿主机 `build/`、`var/` 在本轮按根 README 要求由用户创建。实际依赖：

- MuJoCo `3.17.0`、Python `3.12.3`、NumPy `1.26.4`、SciPy `1.17.1`、Pydantic `2.10.6`。
- EGL：`NVIDIA GeForce RTX 4050 Laptop GPU/PCIe/SSE2`，驱动 `4.6.0 NVIDIA 580.178.04`；`nvidia=True`。
- ROS 2 Jazzy；本轮在 runtime 容器内生成 `panda_stack` ROS SDK：
  `b79e8a7e83034be345057b965814c75b03e67bcf96f0080f58b790c3503740da`，输出 `/build/ros-dev`。
- LeRobot 源码与学习镜像已存在；本轮未重建镜像。

## Panda 回归基线

### 环境、场景与运动学

```bash
# simulation 容器
scripts/run.sh environment --run-id p00-panda-env-20261112
scripts/run.sh scene --run-id p00-panda-scene-20261112
scripts/run.sh kinematics --run-id p00-panda-kin-20261112
```

| 检查 | 结果 | 证据 |
| --- | --- | --- |
| 环境 | 成功；单 Panda 编译 `nq=9, nv=9, nu=8`；EGL NVIDIA；20 步有限 | `var/reports/sim-check/p00-panda-env-20261112/report.json` |
| 场景 | 成功；`nq=32, nv=30, nu=16, ncam=3`；67/67 布局通过；201 张布局 PNG | `var/reports/sim-check/p00-panda-scene-20261112/` |
| 运动学 | 成功；402/402 IK 目标收敛；67/67 布局通过；最小净空 `0.013054 m` | `var/reports/sim-check/p00-panda-kin-20261112/report.json` |

### Panda 程序示教对照

按根 README 第 2.2～2.4 节执行：

```bash
# simulation 容器
scripts/run.sh prepare --scenario panda_stack --role teacher --seed 0 \
  --run-id teacher-backend-p00 --output /data/prepared/teacher-backend-p00
scripts/run.sh backend --config /data/prepared/teacher-backend-p00/backend.yaml \
  --run-id teacher-backend-p00

# runtime 容器
runtime/ros2/scripts/record.sh --scenario panda_stack --purpose pilot --count 1 \
  --max-consecutive-failures 1 \
  --backend-log /data/runs/backend/teacher-backend-p00/log
```

实际结果：退出码 0；`task.outcome=succeeded`，`runtime.reason=TASK_SUCCEEDED`，
`recording.complete=true`，`quality_qualified=true`，480 条标签，`raw_bytes=393487311`，
`wall_s=41.005`；最终事件 `recording.finished` 的 `reason=target_reached`、`qualified=1`。
证据：`var/marvin-handover/p00-panda-pilot.log`、`var/marvin-handover/p00-teacher-backend.log`；
试采产物在 `var/campaigns/panda_stack_v1/attempts/pilot-01000-record-1794469128399392201/`。

### 测试

下表保留当时测试范围和日志结果；命令列是范围摘要，部分文件路径及容器启动参数未完整记录，不作为逐字复现命令。后续交付记录须保存完整命令、工作目录和容器入口。

| 环境/范围 | 范围摘要 | 历史结果 |
| --- | --- | --- |
| simulation 镜像，全仓单元测试（除 learning） | `pytest tests/unit --ignore-glob="tests/unit/test_learning_*.py"` | 1002 passed, 1 failed, 7 skipped |
| simulation 容器，MuJoCo 集成测试 | `pytest tests/integration/test_sim_scene.py test_panda_io.py test_scene_visibility.py test_scene_kinematics.py test_follower_execution.py` | 51 passed, 2 failed |
| runtime 容器，单元测试 | `runtime/ros2/scripts/run.sh test -q tests/unit` | 944 passed, 3 failed, 28 skipped |
| learning 容器，全仓单元测试 | `learning/lerobot/scripts/run.sh test -q tests/unit` | 1002 passed, 1 failed, 10 skipped |

既有失败，本轮未修改：

- `test_mcap_feedback_guard.py::test_export_decodes_legacy_feedback_without_the_current_type`：
  该环境未提供 `robot_ros_support`；在 runtime 容器具备 ROS SDK 时属于跨环境依赖，非本次接入回归。
- `test_scene_kinematics.py::test_real_segmentation_fails_an_impossible_visibility_threshold`：
  测试对 `VisibilityRule` 使用 `dataclasses.replace`，而当前类型不是 dataclass。
- `test_scene_kinematics.py::test_report_write_failure_is_nonzero_and_loud`：
  测试补丁 `check_kinematics.write_json_atomic`，当前模块没有该属性。
- `test_learning_console_progress.py` 三条：runtime 容器内失败，learning 容器内通过。已保存的 `p00-runtime-unit.log` 中至少两条是子进程无法导入 `dual_arm_learning`（`ModuleNotFoundError`），属于对应测试进程的依赖/导入路径缺口；其他失败原因按各自日志定位。

P01 回归使用受影响的 Panda 场景/I/O/运动学子集，见 `phase-01.md`。两条运动学测试的失效接口在 P02B 修正；反馈 MCAP 在具备 ROS SDK 的 runtime 环境验证，learning 不引入 ROS。后续结果与本节历史基线分别记录。

## 新方案命名与产物归属

- 方案 id：`marvin_handover`。
- 仿真任务身份：`handover`。
- 新产物根：`var/marvin-handover/`（容器 `/data/marvin-handover`）；本轮检查使用 `var/reports/sim-check/`。
- 未创建 `var/campaigns/marvin_handover_diag/` 或 learning 产物目录；P04/P05 建立时再按实际运行分配。
- 既有 `panda_stack` 目录、历史产物和删除状态保持不变。

## 职责与后续接入点

- 端内机制：`simulation` 的 `physics/scene_container.py` 承担编译与复位；`render/` 继续承担三路相机渲染。
- 机器人适配：P01 新增 `physics/marvin/assembly.py`，底座固定、具名关节/执行器/夹爪/TCP 绑定。
- 任务环境：P01 新增 `physics/marvin/environment.py`，桌面、长方块、开口箱体、相机安装座。
- 方案层：`shared/scenarios/marvin_handover/` 保存源资产、档案、标定和场景配方。
- P02 接入点：owner/controller 通过装配入口选择 Marvin I/O；P03 接入任务插件与 Teacher；P04 接入 runtime 采集；P05 以现有 ACT 档案/配方兼容性为起点；hardware 保持边界说明。
