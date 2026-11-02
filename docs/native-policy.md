# 连续 Policy RPC、受限执行与 MCAP v4

本文只描述 DSH-032 的 S3 接口闭环：三个 RGB 与 16 维测量经真实 gRPC fake
provider 返回 60×16 预测，再由真实 ROS `SubmitFollower` 交给已验收的 MuJoCo
bounded follower 执行，并记录 MCAP v4。它是有限诊断，`task=not_evaluated`；
不是正式示教数据、ACT 训练、分数或任务终止规则。

## 先确认宿主路径

这些入口不会自动猜测根目录。先显式设置并检查空间/挂载：

```bash
export ROOT=/home/ljk/work/sim_dual_arm  # replace with the checkout you are using
export BUILD_ROOT=/home/ljk/work/sim_dual_arm/.build/ros-dev
export DATA_ROOT="$ROOT/.build/runs/native-policy"
export ASSETS_ROOT=/home/ljk/work/sim_dual_arm/third_party/src/mujoco_menagerie
export POLICY_SDK_ROOT="$ROOT/.build/generated/policy-sdk"

mkdir -p "$DATA_ROOT"
findmnt -T "$ROOT"
findmnt -T "$BUILD_ROOT"
findmnt -T "$DATA_ROOT"
df -h "$ROOT" "$BUILD_ROOT" "$DATA_ROOT"
test -r "$ASSETS_ROOT" && echo assets-readable
```

源码、配置和 SDK 都是只读挂载；只有 `DATA_ROOT` 可写。不要复制资产进工作树。

## 生成并核对新 Policy SDK

`time_from_observation_ns` 与 `PolicyInfo.chunk_size/test_only` 是 wire 契约的
一部分。旧 SDK 不会被回退读取：

```bash
cd "$ROOT"
python3 tools/contracts/generate_policy_sdk.py --output "$POLICY_SDK_ROOT"
PYTHONPATH="$POLICY_SDK_ROOT" python3 - <<'PY'
from dual_arm_policy.v1 import policy_pb2
assert policy_pb2.DESCRIPTOR.package == "bimanual.policy.v1"
fields = policy_pb2.JointTarget.DESCRIPTOR.fields_by_name
assert fields["time_from_observation_ns"].number == 1
assert "time_from_start_ns" not in fields
info = policy_pb2.PolicyInfo.DESCRIPTOR.fields_by_name
assert (info["chunk_size"].number, info["test_only"].number) == (14, 15)
print(policy_pb2.__file__)
print(policy_pb2.DESCRIPTOR.name, policy_pb2.DESCRIPTOR.package)
PY
```

真实容器里 SDK 安装在 protocol/policy 镜像中；运行时可把生成目录只读挂到
`/opt/policy-sdk`，并把该路径放在 `PYTHONPATH` 最前。报告必须记录实际导入
路径和 descriptor，不能只凭文件存在宣称新协议生效。

## 本机 ROS-free 自测

```bash
cd "$ROOT"
# 纯配置/契约/服务层单测
PROJECT_PYTHON=python3
"$PROJECT_PYTHON" -m pytest -q tests/unit/test_policy_protocol.py \
  tests/unit/test_policy_config.py tests/unit/test_policy_service.py \
  tests/unit/test_native_policy_stream.py
```

服务层没有 Torch/ROS 依赖；fake provider 位于 `tests/doubles/policy`，通过
typed `PolicyProvider` 注入生产 `PolicyService`。有 gRPC 环境时可直接启动：

```bash
PYTHONPATH="$POLICY_SDK_ROOT:$ROOT/components/learning/src:$ROOT/tests/doubles/policy" \
  python3 -m run_fake --config "$ROOT/configs/learning/fake_policy.yaml"
```

它明确 `test_only=true`，输出无真值的小幅正弦运动；默认 runtime 配置没有
`policy` 段，因此拒绝 policy，只有 `configs/runtime/native_policy.yaml` 显式
`allow_test_policy: true`。

## 真实 Compose 入口（监督者构建镜像后）

```bash
cd "$ROOT"
# 需要已构建 backend_follower / native runtime / learning policy 镜像与 SDK
export NATIVE_UID=$(id -u)
export NATIVE_GID=$(id -g)
python3 tools/run/native_policy.py \
  --config configs/runtime/native_policy.yaml \
  --policy-config configs/learning/fake_policy.yaml \
  --policy-sdk "$POLICY_SDK_ROOT" \
  --build-root "$BUILD_ROOT" \
  --assets-root "$ASSETS_ROOT" \
  --asset-summary /home/ljk/work/sim_dual_arm/.build/assets/panda.json \
  --data-root "$DATA_ROOT" \
  --run-id policy-smoke-001 \
  --timeout 120
```

入口复用 `native_backend` 的 project guard 和 `dev_compose` 的 run id/path
校验；它只拥有自己的 Compose project，显式 stop 后 `down --remove-orphans`。
`native_backend.yaml + native_runtime.yaml + native_policy.yaml` 共用私有
network、`ROS_DOMAIN_ID`、只读源码/SDK/资产和可写数据根；fake policy 容器无
ROS、无 GPU。

## 有限流语义

- 第一次 hand-off 由 backend 的 `panda_stream.initial_lead_s=0.25` 决定；每个
  capture 的 60 个原始点按 50 ms 均匀分布，所有 `time_from_observation_ns`
  都相对该 capture 的 `reference_robot_time_ns`。
- 每块从新完整 capture 预测一次，选择 hand-off 之后的 5 个原始点；后继块用
  上一块真实 anchor 的 p/v 接续，不重基准、不改写左臂、不插入常量点。
- `max_chunks=8`，只有第 8 块 `settle_at_end=true`；结束后等待真实
  `NativeExecution COMPLETED`。超时、迟到、非法响应、旧 epoch、断连、超偏差
  都进入有界停止，实际反馈冻结后才收尾。
- `SubmitFollower` 未知 ACK 保留 command ID 和 chunk seq，先查询同一 ID，再
  同 ID 幂等重发；合法重试必须返回原 accepted sim/wall/digest。

## MCAP v4 与审计

policy 模式写 `schema_version=4`，包含 BackendInfo、状态/三原图、
ObservationIndex、完整原始 NativeCommand、FollowerReference、原生事件/反馈。
v1/v2/v3 和默认 v2 不变。独立审计先复用 v3 的 context/capture/event/terminal
检查，再按应用时刻用 `FollowerReference.applied_sample_at(profile)` 验证
actual 参考与数值修正，并用 `validate(record, profile, limits)` 重建 031 约束；
只把实际应用的区间当未来标签。完成运行要求 8 块、每块 60 个原始点全保留、
唯一 final 块和实际应用反馈。

```bash
# 独立审计已提交记录（容器内 SDK/ROS 环境）
python3 -m dual_arm_runtime.audit "$DATA_ROOT/<run-id>/raw/<run-id>/<episode>"
```

## 日志与边界

统一结构化日志记录 policy session/request、模型/处理器/obs/action 摘要、
capture/observation 身份、RPC 耗时/deadline/失败；不写图像或大数组。被拒
protobuf 只可另存小型 `.pb` 与 hash，JSON 日志不得因 NaN 破坏。配置快照明确
`action_source`、backend `test_only` 与 policy `test_only`。recording 完整性、
runtime 结果和 task 结果独立；合法中止可完整录制，未知执行不能伪造成功，所有
诊断仍 `task=not_evaluated`。
