# 新方案模板（_template）

本目录说明如何在现有接口内新增一个方案。它不是可运行方案：目录中没有
`binding.yaml`，所以 `--scenario _template` 会报没有 binding 文件，不会回退
到 [`panda_stack`](../panda_stack/README.md)。[`binding.example.yaml`](binding.example.yaml)
的全部取值都是明显占位符，未填写时校验直接失败；入口只读取
`binding.yaml`，不会读取本示例。

## 方案、实现与档案

- 方案：`shared/scenarios/<scenario-id>/`，持有档案、配方路径引用和方案专用接口。
- 实现：`simulation/`、`runtime/`、`learning/`、`hardware/`，提供实际能力。
- `binding.yaml` 只做选择：实现名、档案路径、配方路径。限位、相机尺寸、训练参数等
  数值仍留在档案和配方中，不复制进 binding。

实现支持某个取值后，binding 才能写这个取值。当前已实现并会被入口接受的取值：

| 字段 | 当前取值 | 接受位置 |
| --- | --- | --- |
| `backend.kind` / `backend.implementation` | `simulation` / `mujoco` | runtime 采集选择拒绝其他组合 |
| `runtime.implementation` | `ros2` | runtime 采集选择 |
| `learning.implementation` / `learning.adapter` | `lerobot` / `act` | learning 所有 `--scenario` 入口 |

`hardware` 只是 binding 的结构取值；仓库目前没有已验证的真实驱动。
simulation 的 `backend` / `viewer` 只按 binding 取配方，不按实现名分支；能力是否可用由
实际运行的端点决定。

## 目录

```text
shared/scenarios/<scenario-id>/
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
  例如复用相邻方案：`source_roots: [".."]`，档案写
  `../panda_stack/profiles/robots/panda_stack_learning_v1/profile.json`。
- `artifact_roots` 可写绝对路径；部署数据根通常是 `/data`。bundle 覆盖路径先在
  调用者 cwd 解析，再检查是否落在 binding 目录、source root 或 artifact root 内。
- `profiles.*`、`recipes.*` 必须是文件，`interfaces.extension_ros[*]` 必须是目录。
- `--scenario <id>` 查找 `<shared-root>/scenarios/<id>/binding.yaml`；`shared-root`
  取显式参数、`SHARED_ROOT` 或容器内 `/shared`。档案 schema 固定从
  `<shared-root>/common/interfaces/schemas` 读取。
- 配方内部的 profile、场景、控制路径相对配方文件自身解析。容器部署配方中的
  `/workspace`、`/data` 是该容器内的真实路径。

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
  两侧，learning 检查训练/服务配方与 bundle 的一致性；不一致时启动失败。
- `models.bundle: null` 表示采集与学习不绑定模型；推理必须显式给
  `--bundle <verified-bundle-dir>`（或填写 `models.bundle`）。运行端与学习端直接读取
  bundle 内的 manifest 与 Profile 校验身份，不生成也不消费 `policy_binding.json`；
  binding 只检查目录存在和类型，不生成替代摘要。
- 更换 checkpoint：输入输出契约不变时，只把 `models.bundle` 或 `--bundle` 指向新的
  已验证 bundle，方案、档案和配方都不必改。契约改变则属于新实现，需要新的
  learning 适配和档案，不是替换一个路径就能完成。

## 校验与运行

只读校验不需要启动仿真；`inference` 阶段需要真实 bundle：

```bash
# runtime/learning 容器内（仓库根 /workspace，/data 已挂载）
python3 shared/common/tools/scenario.py show \
  --binding shared/scenarios/<scenario-id>/binding.yaml
python3 shared/common/tools/scenario.py validate \
  --binding shared/scenarios/<scenario-id>/binding.yaml --stage collection
python3 shared/common/tools/scenario.py validate \
  --binding shared/scenarios/<scenario-id>/binding.yaml --stage inference \
  --bundle /data/<scenario-id>/models/bundles/<verified-bundle>
```

所有入口都消费同一份 binding，并在内存中展开配方；`--scenario` 与直接配置的
`--config` 互斥，`--role`、`--shared-root`、`--bundle` 只在方案模式下使用，
`--set dot.path=value` 可重复覆盖配方中已存在的键：

```bash
# simulation 容器（工作目录 /app）
scripts/run.sh backend --scenario <scenario-id> --role teacher \
  --seed 0 --run-id teacher-backend
scripts/run.sh viewer --scenario <scenario-id> --role teacher
scripts/run.sh backend --scenario <scenario-id> --role policy \
  --bundle /data/<scenario-id>/models/bundles/<verified-bundle> \
  --seed 0 --run-id policy-backend

# runtime 容器（工作目录 /workspace）
runtime/ros2/scripts/run.sh build-ros-sdk --scenario <scenario-id> --shared-root /shared
runtime/ros2/scripts/record.sh --scenario <scenario-id> \
  --purpose pilot --count 1 --max-consecutive-failures 1
runtime/ros2/scripts/run.sh interactive --scenario <scenario-id> \
  --bundle /data/<scenario-id>/models/bundles/<verified-bundle> --seed 0 --run-id policy-interactive
runtime/ros2/scripts/run.sh policy --scenario <scenario-id> \
  --bundle /data/<scenario-id>/models/bundles/<verified-bundle> \
  --reset-seed 0 --run-id policy-run

# learning 容器（工作目录 /workspace）
learning/lerobot/scripts/run.sh policy --scenario <scenario-id> \
  --bundle /data/<scenario-id>/models/bundles/<verified-bundle>
learning/lerobot/scripts/run.sh train --scenario <scenario-id> --mode smoke
```

运行入口直接按 binding 解析配方并展开路径，不写 `/data/prepared/` 等中间目录；
采集产物按 `var/<scenario-id>/logs/campaign/attempts/<purpose>-<seed>-<run_id>/` 拍平保存；所有运行期产物只进入 `var/<scenario-id>/{datasets,models,logs,diagnostics}/`。
直接配置模式继续可用，且与方案选择互斥。

## 更换内容时改哪里

“配置切换”指现有实现已经支持，只改引用或数值；“需要新增实现”指对应端要先提供
能力，binding 才能选中它。

| 更换内容 | 配置切换（现有实现） | 需要新增实现 |
| --- | --- | --- |
| checkpoint | 改 `models.bundle` 或 `--bundle` 指向新的已验证 bundle；契约不变时档案和配方不变 | 观测/动作/分块契约变化：在 learning 增加适配与处理器，补对应档案 |
| 学习算法 | 已有第二个适配器实现时，切换 `learning.adapter` 及 `recipes.dataset`/`training`/`policy_service` 的配方引用 | 在 learning 实现适配器、配置与 bundle 校验，再在 binding 选择；当前只有 `act`，换成其他算法属于新增实现 |
| 机器人 | 改 `profiles.robot`/`observation`/`action` 及 control、stream、follower、scene、task 配方引用；摘要按新档案计算，配方与模型绑定须匹配新档案 | 新几何/关节/相机需要在 simulation 补模型、场景与执行映射，硬件驱动归对应端 |
| 仿真后端 | 改 `backend.implementation` 与 simulation 配方；当前只有 `mujoco` | 在 simulation 实现后端和部署要求（`docker/deployment.yaml`），再在 binding 选择 |
| 任务 | 改 `backend.task_type`、task 配方，以及 runtime 的 `teacher_api`/扩展引用 | 任务逻辑在所属端实现；方案专用接口放 `extensions/` |
| 仿真切真机 | 结构上可把 `backend.kind` 改为 `hardware` 并省略 simulation 配方 | hardware 端需要真实驱动、设备配置与能力适配；当前没有已验证驱动，runtime 采集仍拒绝非 simulation 后端 |
| 新观测/动作能力 | 在现有协议内用已支持的字段组合出新档案 | 在 `shared/common/interfaces` 演进公共协议，并在生产/消费两端实现；binding 单独改路径不够 |

方案专用能力只在确实需要时新增 `extensions/`，公共协议按实际消费者演进；模板不预建
未来算法、机器人、驱动或后端的占位实现。

目录导航：[方案目录](../) · [panda_stack 方案](../panda_stack/README.md) · [文档索引](../../../docs/README.md)
