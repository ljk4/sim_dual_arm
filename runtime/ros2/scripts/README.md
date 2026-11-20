# Runtime 容器命令入口（Scripts）

本目录只保留 Shell 入口。Python 业务逻辑位于 [`dual_arm_runtime/offline/`](../ros/dual_arm_runtime/dual_arm_runtime/offline/README.md)，由 `python3 -m` 调用，不依赖脚本目录的 `sys.path`。

运行入口接受 `--robot <robot> --task <task>`，从 `<shared-root>/scenarios/<robot>/<task>/binding.yaml` 在内存中选择 campaign、Teacher 与 Policy 配方。`--robot/--task` 与显式配置模式互斥。运行端通过 gRPC 访问 Policy Service。

## 示教录制 `record.sh`

示教录制使用独立入口 `record.sh`，在已经启动 teacher backend 的 `runtime` 容器内执行：

```bash
# 首次按根 README 完成手动后端和可选 viewer 后，单路试采一次
runtime/ros2/scripts/record.sh --robot marvin_sim --task stack \
  --purpose pilot --count 1 --max-consecutive-failures 1

# 本次新增 10 条合格 episode，或连续 3 次失败时结束
runtime/ros2/scripts/record.sh --robot marvin_sim --task stack \
  --count 10 --max-consecutive-failures 3
```

`--count` 默认为 1，只计本次新增的合格 episode；任务成功、运行结果、录制完整性与导出质量必须全部合格。所有不合格尝试都增加连续失败计数，合格一次后清零。`--max-consecutive-failures` 默认为 3，达到后返回码为 1；预算/种子不足或配置无效返回非零，`Ctrl-C` 返回 130。

采集条数、用途与连败上限由专用命令参数指定。观测、Teacher 和门禁参数在相应配方中修改。

默认从 `/data/<robot>/<task>/logs/backend/` 下按最近修改时间选择符合 Teacher 命名的运行目录，再读取其 `log/`。未找到候选时使用 `teacher-backend/log`。直接配置模式在 `/data/_direct/logs/backend/` 下查找。存在多个后端运行目录或使用自定义运行名时，用 `--backend-log` 传入该后端启动时打印的日志路径。

该入口调用 `record.py`，逐局预留 campaign 的种子与预算，再由 `collect_attempt.py` 启动运行端并导出。正式采集达标时将有序合格局写入 `var/<robot>/<task>/datasets/raw/formal-select.json`。该文件的 `selected` 数组供 Learning 转换时划分训练集和验证集，最终采集摘要的 `formal_select` 给出清单路径。

`--config` / `--runtime-config` 可用于直接配置模式；预算、全局合格目标和已有台账仍生效，不会因批量录制重置。当前配置最多累计 50 条合格正式数据，因此已有数据时应相应减少 `--count`。

## 宿主机并行采集 `record_parallel.sh`

并行采集使用仓库根目录的宿主机入口，自动管理独立的 simulation worker：

```bash
# 在宿主机项目根目录执行，不在 runtime 容器内执行
./record_parallel.sh --robot marvin_sim --task stack \
  --purpose formal --target-total 50 --workers auto

# 也可只指定本次新增的合格条数
./record_parallel.sh --robot marvin_sim --task stack \
  --count 10 --workers 4 --max-consecutive-failures 3
```

单路 `pilot` 试采合格后停止手动 backend 和 viewer，在宿主机并行补足 50 条合格 formal 采集局，再在 Learning 容器执行转换和训练。Runtime 达标后自动发布正式清单。 [完整流程](../../../README.md)

`--target-total N` 根据已有台账计算差额，只补足到累计 N 条合格数据；已达到目标时直接返回成功且不启动新的仿真 worker。中断后可原样重跑。它与表示本次新增数量的 `--count` 互斥，formal 目标不得超过 campaign 配置的合格上限。调度器取得运行锁后还会重新检查差额，防止 backend 启动期间台账进度发生变化。

`record_parallel.sh` / `dual_arm_runtime.offline.record_parallel` 只在宿主机编排 Docker，宿主机仅需 Python 3 标准库和 Docker Compose。后台的 `record_workers.py` 从方案的 simulation Teacher 配方生成隔离配置，`record_pool.py` 在 runtime 容器内统一预留预算、分配种子和调度录制。每个 worker 的 MuJoCo 场景、ROS domain、命名空间、配置与日志均独立；原有手动 backend 无需启动。runtime 镜像不增加 MuJoCo 依赖，也不挂载 Docker socket。

自动并发数按可用 CPU/内存/显存估计，最多 4 路；手动 `--workers` 支持 1～16，实际数量还受本次目标条数限制。`--realtime-speed` 默认 1.0，保持原有仿真速度；提高并发不改变物理步长、20 Hz 录制频率和准入规则。各路设置 BLAS/OMP 线程数为 1，避免线程过量竞争。

并行模式关闭 `robot.visualization_enabled`、清除图形显示环境，并固定以 EGL 后台渲染相机；viewer 不可连接这些 worker。采集导出始终带 `--no-previews`，不生成预览图与 GIF，训练 RGB PNG 和校验哈希照常写入。

合格数量仅统计本批次，在途数受剩余目标和全局合格目标约束。每个 worker 独立统计连败，达到上限后停止调度并取消在途任务，取消项记入台账。预算不足时先完成已预留的任务再退出。中断会取消本批次进程并关闭其 simulation 容器。`datasets/raw/<purpose>-<seed>-<run_id>/` 保存合格 episode 的 `raw/`（MCAP、`manifest.json`、`task_result.json`）与 `export/`（`labels.json`、`raw_predictions.json`、`quality.json`、`rgb/`）。运行日志与 `console.log` 位于 `logs/collection/attempts/<attempt>/`。失败尝试在清理录制和导出之前，将可用的画面和轨迹质量结果、失败原因及标签拒绝计数写入台账的 `diagnostics`。批次配置、backend 日志、`workers.json` 和 `summary.json` 放在 `var/<robot>/<task>/logs/collection/record-pools/pool-*/`。

## 其余独立入口脚本

| 脚本 | 功能说明 |
| --- | --- |
| `build_sdk.sh` | 构建 ROS 接口 SDK（`/build/ros-dev/install`）与 Policy Protobuf/gRPC SDK（`/build/generated/policy-sdk`） |
| `record.sh` | 单路示教录制；也支持 `record.sh status --robot <robot> --task <task>` 查看采集与筛选状态 |
| `client_policy.sh` | 执行单局非交互式 Policy 推理闭环。通过 `--policy-endpoint HOST:PORT` 指定本次 gRPC 连接地址 |
| `client_interactive.sh` | 交互式执行单局 Policy 推理。输入 `y` 复位并开始，结束后保持 IDLE 直至 `Ctrl-C`。支持 `--policy-endpoint HOST:PORT` |
| `test.sh` | 运行固定禁用自动插件收集的 `pytest` 测试套件 |

常用示例：

```bash
# 查看采集进度与数据集状态
scripts/record.sh status --robot marvin_sim --task stack

# 直接以方案执行一局策略
scripts/client_policy.sh --robot marvin_sim --task stack \
  --reset-seed 20000 --run-id policy-run

# 交互式执行
scripts/client_interactive.sh --robot marvin_sim --task stack \
  --seed 20000 --run-id policy-interactive
```

> 容器内脚本在宿主机直接运行会报错退出（返回码 2）；请先通过 `./env.sh enter runtime` 进入容器（`./record_parallel.sh`、`./inference_parallel.sh` 在宿主机执行）。

目录导航：[上级目录](../README.md)
