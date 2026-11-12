# Runtime 配置说明

本目录只保留运行端私有环境；Panda 实例运行配方已迁至方案目录。

- **`container.env`**：声明容器内项目根路径（`PROJECT_ROOT=/workspace`）、ROS 2 域 ID（`ROS_DOMAIN_ID=42`）、Fast DDS 配置路径与 Python 源码路径，由 Compose `env_file` 自动加载。
- **[`shared/scenarios/panda_stack/configs/runtime/`](../../../shared/scenarios/panda_stack/configs/runtime/)**：Teacher 采集、Policy 推理与 campaign 配方。
- **[`shared/scenarios/panda_stack/binding.yaml`](../../../shared/scenarios/panda_stack/binding.yaml)**：方案选择；`recipes.runtime_teacher` / `recipes.runtime_policy` 指向上述配方。

## 引用与绑定规则

1. **控制配置单一数据源**：运行配方通过相对路径引用 [`shared/scenarios/panda_stack/configs/simulation/scene/`](../../../shared/scenarios/panda_stack/configs/simulation/scene/README.md) 下的场景文件、[`shared/scenarios/panda_stack/profiles/control/`](../../../shared/scenarios/panda_stack/profiles/control/README.md) 下的共享控制/跟随参数以及 [`shared/scenarios/panda_stack/profiles/`](../../../shared/scenarios/panda_stack/profiles/README.md) 下的机器人语义档案。`target_count` 唯一定义于 `shared/scenarios/panda_stack/profiles/control/panda_follower_v1.yaml`。
2. **动态 Policy 绑定**：`policy_rollout.yaml` 中的 `FROM_VERIFIED_ACT_BUNDLE` 占位符必须通过 `runtime/ros2/scripts/run.sh prepare` 替换为真实模型包摘要后方可运行；直接加载含未替换占位符的模板会立即报错拒绝。
3. **方案选择**：`prepare --scenario panda_stack --role teacher|policy` 从 binding 选择配方；`prepare --template <recipe>` 保留直接配置模式。二者互斥。Policy 角色仍需真实 `policy_binding.json`，并可用 `--bundle` 提供已验证模型目录；Teacher 角色不消费 Policy binding。

```bash
runtime/ros2/scripts/run.sh prepare --scenario panda_stack --role policy \
  --policy-binding /data/prepared/<policy-run>/policy/policy_binding.json \
  --bundle /data/learning/bundles/<verified-bundle> \
  --seed 20001 --run-id policy-run --output /data/prepared/policy-run
```

目录导航：[上级目录](../README.md)
