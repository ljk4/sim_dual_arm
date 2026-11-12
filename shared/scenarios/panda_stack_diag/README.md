# panda_stack_diag 方案（诊断复现）

阶段 12 有界集成回归使用的临时方案，只用于复现诊断，不是第二套正式流程。它复用 [`panda_stack`](../panda_stack/README.md) 的机器人档案、Teacher/Policy 配方与方案专用接口，只把 campaign 换成诊断版本。

## 与正式方案的区别

- [`binding.yaml`](binding.yaml)：档案、配方与扩展接口全部引用 `../panda_stack/`，仅 `recipes.campaign` 指向本目录。
- [`configs/stack_campaign_diag.yaml`](configs/stack_campaign_diag.yaml)：两个隔离 worker 的短期 Teacher 尝试；诊断输出根、台账、种子 900000/900001 与两次尝试上限。
- [`configs/stack_campaign_interrupt.yaml`](configs/stack_campaign_interrupt.yaml)：并行入口中断收尾检查；独立台账、种子 900100、一次尝试。
- [`configs/precommit_dataset.container.yaml`](configs/precommit_dataset.container.yaml) 与 [`configs/precommit_training.container.yaml`](configs/precommit_training.container.yaml)：提交前小闭环的 2 条真实 episode 转换与 128 步诊断训练；独立数据集、预算、checkpoint 与日志根。
- 业务质量门禁、Teacher 能力、必需路径与超时逐项复制自正式 [`stack_campaign.yaml`](../panda_stack/configs/runtime/experiments/stack_campaign.yaml)，未放宽。真实 `panda_stack/binding.yaml`、模型、Profile 与物理参数未改。

## 复现命令

```bash
# runtime 容器：只读预检
python3 -m dual_arm_runtime.offline.record_preflight \
  --scenario panda_stack_diag --purpose pilot --parallel

# 宿主机项目根目录：两个隔离 worker、各一次 Teacher 尝试
runtime/ros2/scripts/record_parallel.sh --scenario panda_stack_diag \
  --purpose pilot --count 2 --workers 2 --domain-base 210 \
  --max-consecutive-failures 2

# 宿主机：并行入口中断收尾（启动后发送 SIGTERM）
runtime/ros2/scripts/record_parallel.sh \
  --config shared/scenarios/panda_stack_diag/configs/stack_campaign_interrupt.yaml \
  --runtime-config shared/scenarios/panda_stack/configs/runtime/teacher_collect.yaml \
  --backend-template shared/scenarios/panda_stack/configs/simulation/backend/backend_stack_teacher.yaml \
  --purpose pilot --count 1 --workers 1 --domain-base 213 \
  --max-consecutive-failures 1
```

诊断配置把输出写入 `var/diagnostics/phase-12-20261112/`（容器内 `/data/diagnostics/phase-12-20261112/`），包括 Teacher 池、并行中断现场、新 episode 导出/审计、Policy prepared/raw/export/日志和历史 episode 只读导出/审计。提交前小闭环的诊断数据集、训练、bundle、prepared 与推理结果写入 `var/diagnostics/precommit-smoke-20261112/`。这些配置与结果作为运行证据保留；正式台账、正式预算与 `var/` 其他产物不受影响。

本目录不进入默认方案流程，也不得用于正式采集或训练清单。诊断结论与边界见[阶段 12 交接](../../../docs/architecture-tasks/shared-scenarios/deliveries/phase-12.md)。

目录导航：[方案目录](../) · [panda_stack 方案](../panda_stack/README.md) · [架构任务索引](../../../docs/architecture-tasks/README.md)
