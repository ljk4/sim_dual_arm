# panda_stack 方案

当前双 Panda 堆叠实例的选择关系、公共档案与数值配方：MuJoCo 仿真后端、ROS 2 运行端、LeRobot ACT 学习与 Policy 服务。本目录不持有四端业务实现，也不定义公共接口；公共协议约束见 [`shared/common`](../../common/README.md)。

## 目录

- `binding.yaml`：唯一方案选择文件（v1）。只记录实现选择、档案和 recipe 路径，不复制限值、相机尺寸或训练数值。
- `profiles/`：机器人、观测、动作公共档案（唯一权威来源）。
- `configs/simulation/`：MuJoCo 后端、场景与检查配方。
- `configs/runtime/`：Teacher/Policy 运行配方与采集 campaign。
- `configs/learning/`：LeRobot 数据集转换、训练、采样过滤与 Policy 服务配方。
- `extensions/ros/panda_stack_api_msgs/`：本方案专用固定堆叠 Teacher ROS 接口；只由 Teacher 两侧适配延迟加载。

## 接口与配方常量

公共 Policy/Teacher 协议不固定机器人配方。本方案的以下常量只在本目录档案与配方中定义：

- 16 维关节顺序：`left_joint1..left_joint7, left_gripper_width, right_joint1..right_joint7, right_gripper_width`；手臂单位为 rad，夹爪总开口为 m。
- 三路 RGB 相机：`global, left_wrist, right_wrist`，按观测档案顺序。
- Teacher 动作块：60 点 × 16 维，50 ms 间隔（20 Hz）。
- Teacher 能力标识：`panda_stack_api_msgs.teacher_chunk.v1`；服务名保持 `<namespace>/get_task_chunk`。

`models.bundle` 有意为空：collection 阶段可独立校验，inference 准备必须显式提供真实且已由学习端校验的 bundle 目录（`--bundle`），不得用空目录或伪造摘要代替。scenario policy prepare 还要求该 bundle 与 `policy_binding.json` 的 `model.bundle` 指向同一真实目录（符号链接、相对路径或 bind-mount 别名均可）；不同目录无法在不重新校验模型内容的情况下证明一致，会被拒绝。

`source_roots` 为相对 binding 的共享根 `../..`；`artifact_roots` 为部署数据根 `/data`，因此该 binding 应在项目容器内读取（宿主机没有 `/data`）。

## 校验

以下命令在 runtime/learning 容器内执行（共享 Python 包已在 `PYTHONPATH`）：

```bash
python3 shared/common/tools/scenario.py show \
  --binding shared/scenarios/panda_stack/binding.yaml
python3 shared/common/tools/scenario.py validate \
  --binding shared/scenarios/panda_stack/binding.yaml --stage collection
python3 shared/common/tools/scenario.py validate \
  --binding shared/scenarios/panda_stack/binding.yaml --stage inference \
  --bundle /data/learning/bundles/<verified-bundle>
```

## 操作入口

以下命令均在对应容器内执行；完整的容器启动、数据筛选与操作步骤见[根 README](../../../README.md)。所有准备入口通过 `--scenario panda_stack` 读取本目录 `binding.yaml`，`--scenario` 与直接配置的 `--template`/`--config` 互斥。

### SDK

```bash
# runtime 容器：核心 robot_api_msgs、sim_api_msgs 与 panda_stack_api_msgs
runtime/ros2/scripts/run.sh build-ros-sdk --scenario panda_stack --shared-root /shared
```

### 采集

```bash
# simulation 容器：生成并启动手动 Teacher 后端
scripts/run.sh prepare --scenario panda_stack --role teacher \
  --seed 0 --run-id prep-teacher --output /data/prepared/prep-teacher
scripts/run.sh backend --config /data/prepared/prep-teacher/backend.yaml --run-id prep-teacher

# runtime 容器：单路试采
runtime/ros2/scripts/record.sh --scenario panda_stack \
  --purpose pilot --count 1 --max-consecutive-failures 1

# 宿主机项目根目录：并行补足正式数据（自动管理独立 simulation worker）
runtime/ros2/scripts/record_parallel.sh --scenario panda_stack \
  --purpose formal --target-total 50 --workers auto
```

采集配方由 binding 的 `recipes.runtime_teacher` 与 `recipes.campaign` 选择；手动 Teacher 后端在前台运行，单路试采在另一个 runtime 终端执行。并行入口无需手动后端，使用独立 ROS domain、命名空间和日志目录，预览图与 GIF 关闭，训练 RGB、标签和质量门禁保留。

### 数据转换与训练

```bash
# learning 容器
learning/lerobot/scripts/run.sh convert --scenario panda_stack --shared-root /shared
learning/lerobot/scripts/run.sh train --scenario panda_stack --shared-root /shared --mode full
learning/lerobot/scripts/run.sh bundle --scenario panda_stack --shared-root /shared \
  --mode full --run /data/learning/runs/<run-dir> \
  --output /data/learning/bundles/stack-act
```

binding 与直接 `--config` 入口共用 `act_dataset.yaml`、`act_training.yaml`。源文件和产物路径按项目根解析，构建缓存位于 `build/`，数据、训练预算与模型位于 `var/`；容器挂载使这些项目相对路径可写，无需维护第二份容器配置。

### 模型准备

```bash
# learning 容器：校验真实 bundle，生成 policy.yaml 与 policy_binding.json
learning/lerobot/scripts/run.sh prepare-policy \
  --scenario panda_stack --shared-root /shared \
  --bundle /data/learning/bundles/<verified-bundle> \
  --run-id <run-id> --output /data/prepared/<run-id>/policy
```

`models.bundle` 在 binding 中为空，采集与学习可不绑定模型；推理准备必须显式提供 `--bundle`。bundle 内容、processor 与身份由 learning 校验，真实 `policy_binding.json` 也只由 learning 生成。

### 推理

```bash
# simulation 容器：生成并启动 Policy 推理后端
scripts/run.sh prepare --scenario panda_stack --role policy \
  --bundle /data/learning/bundles/<verified-bundle> \
  --seed 20001 --run-id infer-1 --output /data/prepared/infer-1/simulation
scripts/run.sh backend --config /data/prepared/infer-1/simulation/backend.yaml --run-id infer-1

# learning 容器：启动 gRPC Policy 服务
learning/lerobot/scripts/run.sh policy \
  --config /data/prepared/<run-id>/policy/policy.yaml \
  --run-id <run-id> --log-directory /data/logs/<new-log-dir>

# runtime 容器：生成运行配置并交互式执行一局
runtime/ros2/scripts/run.sh prepare --scenario panda_stack --role policy \
  --policy-binding /data/prepared/<run-id>/policy/policy_binding.json \
  --bundle /data/learning/bundles/<verified-bundle> \
  --seed 20001 --run-id infer-1 --output /data/prepared/infer-1/runtime
runtime/ros2/scripts/run.sh interactive \
  --config /data/prepared/infer-1/runtime/runtime.yaml --seed 20001
```

运行入口只消费 `prepare`/`prepare-policy` 产物，不绕过准备与身份校验。直接配置模式继续可用：`prepare --template <recipe>` 与运行入口的 `--config` 不依赖 binding。

## 已知边界

- 方案与直接配置入口共用 `backend_stack_act_chunked.yaml`，该配方的运行速度和容量以文件内设置为准。
- `learning/lerobot/configs/fake_policy.yaml` 是测试夹具，不属于本方案。
- 当前已实现并被入口接受的组合只有 MuJoCo 仿真后端、ROS 2 运行端与 LeRobot ACT；第二仿真后端、第二学习算法、第二机器人和真实驱动均未实现。
- 小样本诊断可显式使用数据配置的 `diagnostic: true` 与非正式模型准入；输出、台账和模型应放在新的诊断目录，正式配方与默认准入不变。
