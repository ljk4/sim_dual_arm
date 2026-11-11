# Runtime 容器命令入口（Scripts）

本目录只保留 Shell 入口。Python 业务逻辑位于 [`dual_arm_runtime/offline/`](../ros/dual_arm_runtime/dual_arm_runtime/offline/README.md)，由 `python3 -m` 调用，不依赖脚本目录的 `sys.path`。

示教录制使用独立入口 `record.sh`，在已经启动 teacher backend 的 `runtime` 容器内执行：

```bash
# 首次按根 README 完成手动后端和可选 viewer 后，单路试采一次
runtime/ros2/scripts/record.sh --purpose pilot --count 1 --max-consecutive-failures 1

# 本次新增 10 条合格 episode，或连续 3 次失败时结束
runtime/ros2/scripts/record.sh --count 10 --max-consecutive-failures 3

# 使用 pilot 预算，修改模板或后端日志目录
runtime/ros2/scripts/record.sh --purpose pilot --count 2 \
  --runtime-config /workspace/runtime/ros2/configs/teacher_collect.yaml \
  --backend-log /data/runs/backend/teacher-backend/log
```

`--count` 默认为 1，只计本次新增的合格 episode；任务成功、运行结果、录制完整性与导出质量必须全部合格。所有不合格尝试都增加连续失败计数，合格一次后清零。`--max-consecutive-failures` 默认为 3，达到后返回码为 1；预算/种子不足或配置无效返回非零，`Ctrl-C` 返回 130。该入口调用 `record.py`，逐局预留原有 campaign 的种子与预算，再由 `collect_attempt.py` 直接启动运行端并导出，录制流程不再经过 `run.sh`。

`--config` 可指定 campaign 配置；预算、全局合格目标和已有台账仍生效，不会因批量录制重置。当前配置最多累计 50 条合格正式数据，因此已有数据时应相应减少 `--count`。

并行采集使用宿主机入口，自动管理独立的 simulation worker：

```bash
# 在宿主机项目根目录执行，不在 runtime 容器内执行
runtime/ros2/scripts/record_parallel.sh --purpose formal --target-total 50 --workers auto

# 也可只指定本次新增的合格条数
runtime/ros2/scripts/record_parallel.sh --count 10 --workers 4 \
  --max-consecutive-failures 3
```

推荐顺序为：单路 `pilot` 试采合格 → 停止手动 backend/viewer → 宿主机并行补足 formal 50 条 → runtime 执行 `campaign select` → learning 执行 `convert` 和 `train`，完整步骤见[根 README](../../../README.md)。pilot 不进入正式训练清单。

`--target-total N` 根据已有台账计算差额，只补足到累计 N 条合格数据；已达到目标时直接返回成功且不启动新的仿真 worker。中断后可原样重跑。它与表示本次新增数量的 `--count` 互斥，formal 目标不得超过 campaign 配置的合格上限。调度器取得运行锁后还会重新检查差额，防止 backend 启动期间台账进度发生变化。

`record_parallel.sh` / `dual_arm_runtime.offline.record_parallel` 只在宿主机编排 Docker，宿主机仅需 Python 3 标准库和 Docker Compose。后台的 `record_workers.py` 生成隔离配置，`record_pool.py` 在 runtime 容器内统一预留预算、分配种子和调度录制。每个 worker 的 MuJoCo 场景、ROS domain、命名空间、配置与日志均独立；原有手动 backend 无需启动。runtime 镜像不增加 MuJoCo 依赖，也不挂载 Docker socket。

自动并发数按可用 CPU/内存/显存估计，最多 4 路；手动 `--workers` 支持 1～16，实际数量还受本次目标条数限制。`--realtime-speed` 默认 1.0，保持原有仿真速度；提高并发不改变物理步长、20 Hz 录制频率和准入规则。各路设置 BLAS/OMP 线程数为 1，避免线程过量竞争。

并行模式关闭 `robot.visualization_enabled`、清除图形显示环境，并固定以 EGL 后台渲染相机；viewer 不可连接这些 worker。导出自动带 `--no-previews`，跳过预览图与 GIF，训练 RGB PNG 和校验哈希照常写入。单路 `record.sh` 也支持可选的 `--no-previews`。

合格数量仅统计本批次；在途数不会超过剩余目标，也受全局合格目标约束。连续失败按完成顺序统计，达到上限后停止调度并取消在途任务，取消项记入台账且不算合格。预算不足时先完成已预留的任务再退出；中断会取消本批次进程并关闭其 simulation 容器，不停止已有的其他容器。各次尝试的 `console.log` 放在 attempt 目录，批次配置、backend 日志、`workers.json` 和 `summary.json` 放在 `var/record-pools/pool-*/`，汇总包含耗时与合格条数/小时。

其余命令继续通过 `run.sh` 分发：

| 子命令 | 功能说明 |
| --- | --- |
| `build-ros-sdk` | 编译 `contracts/ros/robot_api_msgs`、`contracts/ros/sim_api_msgs`至 `/build/ros-dev` |
| `build-policy-sdk` | 从 `policy.proto` 生成 Python Protobuf/gRPC SDK 至 `/build/generated/policy-sdk` |
| `prepare` | 将 `policy_binding.json`、运行端模板与 `seed`/`run-id` 合成为 `runtime.yaml` 与 `run_manifest.json`（`prepare_run.py`） |
| `policy` | 执行单局非交互式 Policy 推理闭环（须显式指定全新 `--output /data/raw/<run-id>`） |
| `interactive` | 交互式执行单局 Policy 推理：自动生成唯一 `run-id`，输入 `y` 复位环境并开始，结束后保持 IDLE 直至按 `Ctrl-C` 退出 |
| `export` | 从录制目录导出逐帧 RGB、动作参考标签与图像质量报告（`export_episode.py`、`export_media.py`、`export_quality.py`） |
| `campaign` | 查看预算台账、筛选训练/验证集或执行自定义单次尝试（`campaign.py`、`campaign_ledger.py`、`campaign_execution.py`）；示教录制使用 `record.sh` |
| `evaluate` | 生成只读评估指标与可视化对比报告（`evaluate.py`、`evaluation_metrics.py`、`evaluation_media.py`） |
| `test` | 运行固定禁用自动插件收集的 `pytest` 测试套件 |

> `record.sh` 和 `run.sh` 在宿主机直接运行会报错退出（返回码 2）；请先通过 `docker compose -f runtime/ros2/docker/docker-compose.yml exec runtime bash` 进入容器。

目录导航：[上级目录](../README.md)
