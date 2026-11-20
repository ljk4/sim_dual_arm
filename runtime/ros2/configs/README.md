# Runtime 配置说明

本目录只保留运行端私有环境；方案运行配方位于方案目录。

- **`container.env`**：声明容器内项目根路径（`PROJECT_ROOT=/app`）、ROS 2 域 ID（`ROS_DOMAIN_ID=42`）、Fast DDS 配置路径与 Python 源码路径，由 Compose `env_file` 自动加载。
- **[`shared/scenarios/marvin_sim/stack/configs/runtime/`](../../../shared/scenarios/marvin_sim/stack/configs/runtime/)**：Marvin 堆叠的 Teacher 采集、Policy 推理与 campaign 配方。
- **[`shared/scenarios/marvin_sim/handover/configs/runtime/`](../../../shared/scenarios/marvin_sim/handover/configs/runtime/)**：Marvin 交接的 Teacher 采集、Policy 评估与 campaign 配方。
- **`binding.yaml`**：各方案目录（`<robot>/<task>/binding.yaml`）下的方案选择；`recipes.runtime_teacher` / `recipes.runtime_policy` / `recipes.campaign` 指向上述配方。

## 引用与绑定规则

1. **控制配置单一数据源**：运行配方通过相对路径引用方案目录 `configs/simulation/scene/` 下的场景文件、`profiles/control/` 下的共享控制/跟随参数以及 `profiles/` 下的机器人语义档案。`target_count` 唯一定义在对应方案的 follower 档案中。
2. **Policy 身份来自 Policy Service**：运行端不读取模型 Bundle、不要求 `--bundle`。推理启动时通过 `GetPolicyInfo` 握手从 Policy Service 获取并校验模型/处理器摘要、Profile 语义哈希、观测/动作顺序与 chunk 大小；运行配方只声明 RPC 端点与预算。
3. **方案选择**：所有运行入口用 `--robot <robot> --task <task>` 从 binding 选择配方；直接配置模式使用 `--config <recipe>`，二者互斥。重复 `--set dot.path=value` 可覆盖配方中已存在的键。

```bash
# 直接以方案执行一局策略
scripts/client_policy.sh --robot marvin_sim --task stack \
  --reset-seed 20000 --run-id policy-run

# 交互式执行
scripts/client_interactive.sh --robot marvin_sim --task stack \
  --seed 20000 --run-id policy-interactive
```

目录导航：[上级目录](../README.md)
