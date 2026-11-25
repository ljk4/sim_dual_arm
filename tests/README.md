# 测试套件（Tests）

本目录包含全仓单元测试、集成测试、测试替身（Fakes）与配置夹具（Fixtures）：

- **`unit/`**：纯 Python 单元测试与合成数据夹具，覆盖契约模型、轨迹数学、状态机、配置校验与学习流水线。
- **`integration/`**：涉及 ROS 2 通信、提交轨迹的物理执行与 LeRobot CPU 生命周期的集成测试。
- **`fakes/`**：仅供测试使用的确定性 Fake Policy Provider（`fakes/policy/`），标记 `test_only=true`，严禁混入正式评估。
- **`fixtures/`**：用于仿真、控制和策略协议回归的配置夹具。

## 执行方式

从仓库根目录将当前检出的源码和测试送入容器内的临时目录，再用该端的 `scripts/test.sh` 执行。按用例依赖选择 `mujoco`、`runtime`、`lerobot` 或 `tianji`。下面以 MuJoCo 仿真场景测试为例。

```bash
test_env=mujoco
./env.sh up "$test_env"

git ls-files -z --cached --others --exclude-standard |
  while IFS= read -r -d '' path; do
    if [[ -f "$path" ]]; then
      printf '%s\0' "$path"
    fi
  done |
  tar --null -T - -cf - |
  ./env.sh exec "$test_env" bash -ec '
    test_root=$(mktemp -d /tmp/sim-dual-arm-tests.XXXXXXXX)
    trap "rm -rf -- \"\$test_root\"" EXIT
    tar -xf - -C "$test_root"
    cd "$test_root"
    export PROJECT_ROOT="$test_root"
    export PROJECT_SOURCE_ROOT="$test_root/$PROJECT_SOURCE_PREFIX/src"
    export PROJECT_SOURCE_PREFIX=.
    export SHARED_ROOT="$test_root/shared"
    export DATA_ROOT="$test_root/output"
    mkdir -p "$DATA_ROOT"
    export ROS_DOMAIN_ID=78
    /app/scripts/test.sh "$@"
  ' _ -q tests/integration/test_sim_scene.py
```

需要 ROS 或 Policy gRPC 接口的用例使用 `output/build/` 中挂载的 SDK。首次运行前在 Runtime 容器中执行 `scripts/build_sdk.sh`。将示例最后一行的路径替换成待验证的用例。运行场景或 ROS 集成测试时，选择空闲的 ROS 域 ID。

天机设备层离线测试使用 `hardware/tianji/tests/`，在 `tianji` 容器执行其 `scripts/test.sh`；具体入口见 [天机后端](../hardware/tianji/README.md)。

目录导航：[上级目录](../README.md)
