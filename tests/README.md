# 测试套件（Tests）

本目录包含全仓单元测试、集成测试、测试替身（Fakes）与配置夹具（Fixtures）：

- **[`unit/`](unit/README.md)**：纯 Python 单元测试与合成数据夹具，覆盖契约模型、轨迹数学、状态机、配置校验与学习流水线。
- **[`integration/`](integration/README.md)**：涉及 ROS 2 通信、MuJoCo 物理执行、Ruckig 跟随与 LeRobot CPU 生命周期的集成测试。
- **[`fakes/`](fakes/README.md)**：仅供测试使用的确定性 Fake Policy Provider（`fakes/policy/`），标记 `test_only=true`，严禁混入正式评估。
- **[`fixtures/`](fixtures/simulation/README.md)**：仿真与策略回归对照专用的历史配置夹具（`fixtures/simulation/`）。

## 执行方式

所有测试通过各端容器内的 `scripts/test.sh`（底层调用 [`shared/common/tools/scripts/test.sh`](../shared/common/tools/scripts/test.sh)，固定设置 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`）执行：

```bash
# 在 runtime 容器内运行单元测试与 ROS 集成测试：
scripts/test.sh -q tests/unit

# 在 learning 容器内运行学习端单元测试：
scripts/test.sh -q tests/unit

# 在 simulation 容器内运行仿真端测试：
scripts/test.sh -q tests/unit
```

测试产生的临时文件与缓存输出至 `build/` 或 `var/`，不写入源码目录。

目录导航：[上级目录](../README.md)
