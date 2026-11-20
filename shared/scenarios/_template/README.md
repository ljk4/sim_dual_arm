# 新方案模板（_template）

在 `shared/scenarios/<robot>/<task>/` 下新建 `binding.yaml`，以
[`binding.example.yaml`](binding.example.yaml) 为模板，填写实际档案、配方与接口路径。

## 方案、实现与档案

- 方案：`shared/scenarios/<robot>/<task>/`，持有档案、配方路径引用和方案专用接口。
- 实现：`simulation/`、`runtime/`、`learning/`、`hardware/`，提供实际能力。
- `binding.yaml` 只做选择：实现名、档案路径、配方路径。限位、相机尺寸、训练参数等
  数值仍留在档案和配方中，不复制进 binding。

实现支持某个取值后，binding 才能写这个取值。当前已实现并会被入口接受的取值：

| 字段 | 当前取值 | 接受位置 |
| --- | --- | --- |
| `backend.kind` / `backend.implementation` | `simulation` / `mujoco` | runtime 采集选择拒绝其他组合 |
| `runtime.implementation` | `ros2` | runtime 采集选择 |
| `learning.implementation` / `learning.adapter` | `lerobot` / `act` | learning 的机器人和任务选择入口 |

`hardware` 只是 binding 的结构取值；仓库目前没有已验证的真实驱动。
simulation 的 `backend` / `viewer` 只按 binding 取配方，不按实现名分支；能力是否可用由
实际运行的端点决定。

## 目录

```text
shared/scenarios/<robot>/<task>/
├── binding.yaml            # 必需，唯一选择文件（v1）
├── profiles/               # 本方案拥有的机器人/观测/动作档案
├── configs/                # 被选中的数值配方（由所属端维护）
└── extensions/             # 方案专用接口，例如固定 Teacher 服务
```

除 `binding.yaml` 外，只有方案实际拥有的文件才建立对应目录。示例 binding 按上述
布局写相对路径；复用其他方案的文件时通过 `source_roots` 声明可访问根。

## 路径基准

- binding 内的所有路径相对 binding 文件所在目录解析，与调用者 cwd 无关。
- `source_roots` 必须相对 binding 目录；`profiles.*`、`recipes.*`、
  `interfaces.extension_ros[*]` 必须位于 binding 目录或某个 `source_root` 之内。
  例如 Marvin Stack 通过 `source_roots: ["../../.."]` 声明 `shared/`，
  并用 `../handover/profiles/actions/marvin_joint_position_v1.json` 复用 Marvin 的动作档案。
- `artifact_roots` 可写绝对路径，容器数据根为 `/data`。学习端校验显式模型路径位于
  binding 目录、source root 或 artifact root 内。
- `profiles.*`、`recipes.*` 必须是文件，`interfaces.extension_ros[*]` 必须是目录。
- `--robot <robot> --task <task>` 查找
  `<shared-root>/scenarios/<robot>/<task>/binding.yaml`；`shared-root`
  取显式参数、`SHARED_ROOT` 或容器内 `/shared`。档案 schema 固定从
  `<shared-root>/common/interfaces/schemas` 读取。
- 配方内部的 profile、场景、控制路径相对配方文件自身解析。容器部署配方中的
  `/app`、`/shared`、`/data` 是各容器内的真实路径。

## 权威配置与模型引用

- 档案唯一权威来源是本方案 `profiles/` 或通过 `source_roots` 显式引用的既有档案；
  结构约束见 [`shared/common/interfaces/schemas/`](../../common/interfaces/schemas/README.md)。
  binding 加载时会校验 robot/observation/action 三者的绑定关系。
- 配方由所属端维护。binding 只记录选中文件的路径；同一方案需要用两组数值时，在
  所属端新增第二个配方文件，并在 binding 的不同角色字段中分别引用。可用字段只有
  [示例 binding](binding.example.yaml) 中列出的那些，`extends`、`defaults` 等继承
  字段会被拒绝。
- 被选中的配方必须引用 binding 声明的同一个 robot profile。simulation 的
  `backend`/`viewer` 启动时检查 `robot.profile`，runtime 采集选择检查 runtime/simulation
  两侧，learning 检查训练和服务配方与模型档案的一致性。
- 策略服务启动时从 `models/` 选择最新模型，或用 `--model` 指定模型目录。
  学习端校验 `model.safetensors` 内的模型和档案身份，运行端通过 gRPC 调用策略服务。

## 校验与运行

用只读入口校验方案结构与阶段配置，再按需要启动对应服务。

```bash
# 宿主机或挂载 /shared 的容器内
python3 /shared/common/tools/scenario.py show \
  --binding /shared/scenarios/<robot>/<task>/binding.yaml
python3 /shared/common/tools/scenario.py validate \
  --binding /shared/scenarios/<robot>/<task>/binding.yaml --stage collection
python3 /shared/common/tools/scenario.py validate \
  --binding /shared/scenarios/<robot>/<task>/binding.yaml --stage inference
```

三端入口在内存中展开 binding 选定的配方。`--robot/--task` 与 `--config`
分别选择方案和直接配方。`--set dot.path=value` 可重复覆盖配方中已存在的键。

```bash
# simulation 容器（工作目录 /app）
scripts/backend.sh --robot <robot> --task <task> --role teacher \
  --seed 0 --run-id teacher-backend
scripts/viewer.sh --robot <robot> --task <task> --role teacher
scripts/backend.sh --robot <robot> --task <task> --role policy \
  --seed 0 --run-id policy-backend

# runtime 容器（工作目录 /app）；一次构建覆盖全部方案接口包与 Policy SDK
scripts/build_sdk.sh
scripts/record.sh --robot <robot> --task <task> \
  --purpose pilot --count 1 --max-consecutive-failures 1
scripts/client_interactive.sh --robot <robot> --task <task> --seed 0 --run-id policy-interactive
scripts/client_policy.sh --robot <robot> --task <task> \
  --reset-seed 0 --run-id policy-run

# learning 容器（工作目录 /app）
scripts/convert.sh --robot <robot> --task <task>
scripts/train.sh --robot <robot> --task <task> --mode smoke
scripts/server_policy.sh --robot <robot> --task <task> \
  --model /data/<robot>/<task>/models/<act_timestamp>
```

数据、模型、日志和诊断分别写入 `var/<robot>/<task>/{datasets,models,logs,debug}/`。

## 更换内容时改哪里

“配置切换”指现有实现已经支持，只改引用或数值；“需要新增实现”指对应端要先提供
能力，binding 才能选中它。

| 更换内容 | 配置切换（现有实现） | 需要新增实现 |
| --- | --- | --- |
| checkpoint | 用 `--model` 指定新的模型目录 | 观测、动作或分块契约变化时同步适配器、处理器与档案 |
| 学习算法 | 已有第二个适配器实现时，切换 `learning.adapter` 及 `recipes.dataset`/`training`/`policy_service` 的配方引用 | 在 learning 实现适配器、配置与 bundle 校验，再在 binding 选择；当前只有 `act`，换成其他算法属于新增实现 |
| 机器人 | 改 `profiles.robot`/`observation`/`action` 及 control、stream、follower、scene、task 配方引用；摘要按新档案计算，配方与模型绑定须匹配新档案 | 新几何/关节/相机需要在 simulation 补模型、场景与执行映射，硬件驱动归对应端 |
| 仿真后端 | 改 `backend.implementation` 与 simulation 配方；当前只有 `mujoco` | 在 simulation 实现后端和部署要求（`docker/deployment.yaml`），再在 binding 选择 |
| 任务 | 改 `backend.task_type`、task 配方，以及 runtime 的 `teacher_api`/扩展引用 | 任务逻辑在所属端实现；方案专用接口放 `extensions/` |
| 仿真切真机 | 结构上可把 `backend.kind` 改为 `hardware` 并省略 simulation 配方 | hardware 端需要真实驱动、设备配置与能力适配；当前没有已验证驱动，runtime 采集仍拒绝非 simulation 后端 |
| 新观测/动作能力 | 在现有协议内用已支持的字段组合出新档案 | 在 `shared/common/interfaces` 演进公共协议，并在生产/消费两端实现；binding 单独改路径不够 |

方案专用能力只在确实需要时新增 `extensions/`，公共协议按实际消费者演进；模板不预建
未来算法、机器人、驱动或后端的占位实现。

目录导航：[方案目录](../) · [marvin_sim/stack 方案](../marvin_sim/stack/README.md) · [文档索引](../../../docs/README.md)
