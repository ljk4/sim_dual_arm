# Runtime 配置说明

本目录只保留运行端私有环境；方案运行配方位于方案目录。

- **`container.env`**：声明容器内项目根路径（`PROJECT_ROOT=/workspace`）、ROS 2 域 ID（`ROS_DOMAIN_ID=42`）、Fast DDS 配置路径与 Python 源码路径，由 Compose `env_file` 自动加载。
- **[`shared/scenarios/panda_stack/configs/runtime/`](../../../shared/scenarios/panda_stack/configs/runtime/)**：Panda 堆叠的 Teacher 采集、Policy 推理与 campaign 配方。
- **[`shared/scenarios/marvin_handover/configs/runtime/`](../../../shared/scenarios/marvin_handover/configs/runtime/)**：Marvin 交接的 Teacher 采集、Policy 评估与 campaign 配方。
- **`binding.yaml`**：各方案目录下的方案选择；`recipes.runtime_teacher` / `recipes.runtime_policy` / `recipes.campaign` 指向上述配方。

## 引用与绑定规则

1. **控制配置单一数据源**：运行配方通过相对路径引用方案目录 `configs/simulation/scene/` 下的场景文件、`profiles/control/` 下的共享控制/跟随参数以及 `profiles/` 下的机器人语义档案。`target_count` 唯一定义在对应方案的 follower 档案中。
2. **Policy 身份来自 Bundle**：`policy_rollout.yaml` 中的 `FROM_VERIFIED_ACT_BUNDLE` 只是占位符；`--scenario` 启动时会在内存中读取 `--bundle`（或 binding `models.bundle`）的 `manifest.json`，校验摘要、Profile 语义哈希、观测/动作顺序与 chunk 大小后替换这些占位符。直接 `--config` 加载含未替换占位符的模板会立即报错拒绝。
3. **方案选择**：所有运行入口用 `--scenario <id>` 从 binding 选择配方；直接配置模式使用 `--config <recipe>`，二者互斥。Teacher 角色不消费 Bundle；Policy 角色必须能解析出真实已验证 Bundle。重复 `--set dot.path=value` 可覆盖配方中已存在的键。

```bash
# 直接以方案执行一局策略（不再需要 prepare / policy_binding.json）
runtime/ros2/scripts/run.sh policy --scenario panda_stack \
  --bundle /data/panda_stack/models/bundles/stack-act \
  --reset-seed 20000 --run-id policy-run

# 交互式执行
runtime/ros2/scripts/run.sh interactive --scenario panda_stack \
  --bundle /data/panda_stack/models/bundles/stack-act --seed 20000 --run-id policy-interactive
```

目录导航：[上级目录](../README.md)
