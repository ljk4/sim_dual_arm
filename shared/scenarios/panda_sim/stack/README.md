# panda_sim/stack 方案

双 Panda 堆叠实例的选择关系、公共档案与数值配方：MuJoCo 仿真后端、ROS 2 运行端、LeRobot ACT 学习与 Policy 服务。本目录不持有三端业务实现，也不定义公共接口；公共协议约束见 [`shared/common`](../../common/README.md)。

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

binding 不声明模型：collection 与 learning 阶段不绑定模型，`scenario.py validate --stage collection` 与 `--stage inference` 均可独立通过；推理启动时必须通过 `--bundle` 提供真实且已由学习端校验的 bundle 目录。运行端与学习端直接读取 `<bundle>/manifest.json`校验 Bundle 摘要、Profile 语义哈希、观测/动作顺序与 chunk 大小，不再生成或消费 `policy_binding.json`。

`source_roots` 为相对 binding 的共享根 `../../..`；`artifact_roots` 为部署数据根 `/data`，因此该 binding 应在项目容器内读取（宿主机没有 `/data`）。

## 校验

以下命令在 runtime/learning 容器内执行（共享 Python 包已在 `PYTHONPATH`）：

```bash
python3 /shared/common/tools/scenario.py show \
  --binding /shared/scenarios/panda_sim/stack/binding.yaml
python3 /shared/common/tools/scenario.py validate \
  --binding /shared/scenarios/panda_sim/stack/binding.yaml --stage collection
python3 /shared/common/tools/scenario.py validate \
  --binding /shared/scenarios/panda_sim/stack/binding.yaml --stage inference \
  --bundle /data/panda_sim/stack/models/bundles/<verified-bundle>
```

## 操作入口

以下命令均在对应容器内执行；完整的容器启动、数据筛选与操作步骤见[根 README](../../../README.md)。所有 `--scenario panda_sim/stack` 入口直接读取本目录 `binding.yaml` 并在内存中展开配方，不生成中间配置目录；`--scenario` 与直接配置的 `--config` 互斥，重复 `--set dot.path=value` 可覆盖配方中已存在的键。

### SDK

```bash
# runtime 容器：一次构建全部公共与方案扩展接口包，切换方案无需重编
scripts/build_sdk.sh
```

### 采集

```bash
# simulation 容器：直接启动手动 Teacher 后端
scripts/backend.sh --scenario panda_sim/stack --role teacher \
  --seed 0 --run-id teacher-backend

# 宿主机：可选实时可视化（独立只读 viewer）
./env.sh viewer --scenario panda_sim/stack --role teacher

# runtime 容器：单路试采（默认读取 /data/panda_sim/stack/logs/backend/teacher-backend/log）
scripts/record.sh --scenario panda_sim/stack \
  --purpose pilot --count 1 --max-consecutive-failures 1

# 宿主机项目根目录：并行补足正式数据（自动管理独立 simulation worker）
./record_parallel.sh --scenario panda_sim/stack \
  --purpose formal --target-total 50 --workers auto
```

采集配方由 binding 的 `recipes.runtime_teacher` 与 `recipes.campaign` 选择；手动 Teacher 后端在前台运行，单路试采在另一个 runtime 终端执行。并行入口无需手动后端，使用独立 ROS domain、命名空间和日志目录，预览图与 GIF 关闭，训练 RGB、标签和质量门禁保留。

正式采集达到 `--target-total` 时自动原子写出 `var/panda_sim/stack/datasets/raw/formal-select.json`。查看当前采集与筛选状态：

```bash
scripts/record.sh status --scenario panda_sim/stack
```

### 数据转换与训练

```bash
# learning 容器
scripts/convert.sh --scenario panda_sim/stack
scripts/train.sh --scenario panda_sim/stack --mode full
scripts/bundle.sh --scenario panda_sim/stack \
  --mode full --run /data/panda_sim/stack/models/<run-dir> \
  --output /data/panda_sim/stack/models/bundles/<verified-bundle>
```

binding 与直接 `--config` 入口共用 `act_dataset.yaml`、`act_training.yaml`。训练默认使用 50 条正式数据（45 训练 / 5 验证）与 full 5000 步；骨干权重为共享官方输入 `third_party/weights/resnet18-f37072fd.pth`。源文件和产物路径按项目根解析，生成 SDK 位于 `build/`，数据、训练预算与模型位于 `var/`；容器挂载使这些项目相对路径可写，无需维护第二份容器配置。

### 推理

```bash
# learning 容器：直接启动 gRPC Policy 服务
scripts/policy.sh --scenario panda_sim/stack \
  --bundle /data/panda_sim/stack/models/bundles/<verified-bundle>

# simulation 容器：直接启动 Policy 推理后端
scripts/backend.sh --scenario panda_sim/stack --role policy \
  --bundle /data/panda_sim/stack/models/bundles/<verified-bundle> \
  --seed 20000 --run-id policy-backend

# runtime 容器：交互式执行一局；非交互式单局或批量测试（需保持上方 policy.sh 与 backend.sh 在线）
scripts/interactive.sh --scenario panda_sim/stack \
  --bundle /data/panda_sim/stack/models/bundles/<verified-bundle> \
  --seed 20000 --run-id policy-interactive
scripts/policy.sh --scenario panda_sim/stack \
  --bundle /data/panda_sim/stack/models/bundles/<verified-bundle> \
  --reset-seed 20000 --run-id policy-run
scripts/batch_policy.sh --scenario panda_sim/stack \
  --bundle /data/panda_sim/stack/models/bundles/<verified-bundle> \
  --count 10 --seed-start 20000
```

三端都直接消费同一个已验证 bundle，不再经过 `prepare` / `prepare-policy` 产物。更换种子只需改 `--seed`/`--reset-seed`；临时调参追加 `--set`，例如 `--set observation.max_age_ns=200000000`。直接配置模式仍可用：把 `--scenario panda_sim/stack --role ...` 换成 `--config <recipe>`。本轮推理累计 120 次，每批最多 10 次；失败、拒绝和取消均保留并占用次数。按不同模型与有效配置分组报告，验收要求至少一次真实任务成功。

## 已知边界

- 方案与直接配置入口共用 `backend_stack_act_chunked.yaml`，该配方的运行速度和容量以文件内设置为准。
- `learning/lerobot/configs/fake_policy.yaml` 是测试夹具，不属于本方案。
- 当前已实现并被入口接受的组合只有 MuJoCo 仿真后端、ROS 2 运行端与 LeRobot ACT；第二仿真后端、第二学习算法、真实驱动均未实现。
- 小样本诊断可显式使用数据配置的 `diagnostic: true` 与非正式模型准入；输出、台账和模型应放在新的诊断目录，正式配方与默认准入不变。

目录导航：[公共层与方案绑定](../../README.md) · [根 README](../../../README.md)
