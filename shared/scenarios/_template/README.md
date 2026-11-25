# 新方案模板（_template）

在 `shared/scenarios/<robot>/<backend>/<task>/` 下新建 `binding.yaml`，以
[`binding.example.yaml`](binding.example.yaml) 为模板，填写实际档案、配方与接口路径。

## 方案、本体资产与实现

- 机器人本体：`shared/robots/<robot>/`，持有跨后端/跨任务复用的 URDF、meshes、公共档案与扩展接口。
- 方案：`shared/scenarios/<robot>/<backend>/<task>/`，持有任务级档案（如 `profiles/robot/profile.json`）、配方引用与任务专用接口。
- 实现：`simulation/`、`runtime/`、`learning/`、`hardware/`，提供实际能力。
- `binding.yaml` 只做选择：实现名、档案路径、配方路径。限位、相机尺寸、训练参数等数值仍留在档案和配方中，不复制进 binding。

当前已实现并被入口接受的取值：

| 字段 | 当前取值 | 接受位置 |
| --- | --- | --- |
| `backend.kind` / `backend.implementation` | `simulation` / `mujoco` | runtime 采集/推理选择与仿真后端 |
| `runtime.implementation` | `ros2` | runtime 采集/推理选择 |
| `learning.implementation` / `learning.adapter` | `lerobot` / `act` | learning 转换、训练与推理服务 |

`hardware` 与 `hardware/tianji` 的控制/相机底座已就位，待完成 ROS 2 Robot API 与运行时真机钩子后接入方案选择（详见[架构扩展与演进指南](../../../docs/features/架构扩展与演进指南.md)）。

## 目录

```text
shared/scenarios/<robot>/<backend>/<task>/
├── binding.yaml            # 方案选择文件
├── profiles/               # 本方案拥有的任务级机器人/观测/动作档案
├── configs/                # 被选中的数值配方（由所属端维护）
└── extensions/             # 任务专用接口
```

除 `binding.yaml` 外，只有本方案独立拥有的文件才建立对应目录；机器人公共资产与公共档案统一保存在 `shared/robots/<robot>/`，并通过 `source_roots` 显式声明。

## 路径基准

- binding 内的所有路径相对 binding 文件所在目录解析，与调用者 cwd 无关。
- `source_roots` 必须相对 binding 目录；`profiles.*`、`recipes.*`、`interfaces.extension_ros[*]` 必须位于 binding 目录或某个 `source_root` 之内。例如 `marvin/mujoco/stack` 与 `marvin/mujoco/handover` 通过 `source_roots: ["../../../../robots/marvin"]` 声明对 `shared/robots/marvin/` 的访问根，引用其公共观测/动作/控制档案和 `marvin_teacher_api_msgs`。
- `artifact_roots` 可写绝对路径，容器数据根为 `/data`。学习端校验显式模型路径位于 binding 目录、source root 或 artifact root 内。
- `profiles.*`、`recipes.*` 必须是文件，`interfaces.extension_ros[*]` 必须是目录。
- `--robot <robot> --backend <backend> --task <task>` 查找 `<shared-root>/scenarios/<robot>/<backend>/<task>/binding.yaml`；`shared-root` 取显式参数、`SHARED_ROOT` 或容器内 `/shared`。档案 schema 固定从 `<shared-root>/common/interfaces/schemas` 读取。
- 配方内部的 profile、场景、控制路径相对配方文件自身解析；容器内的 `/app`、`/shared`、`/data`、`/build` 为挂载路径。

## 权威配置与模型引用

- 档案唯一权威来源是 `shared/robots/<robot>/profiles/` 或本方案 `profiles/`；结构约束位于 `shared/common/interfaces/schemas/`。binding 加载时会校验 robot/observation/action 三者的绑定关系。
- 配方由所属端维护。binding 只记录选中文件的路径，不接受 `extends`、`defaults` 等继承字段。
- 被选中的配方必须引用 binding 声明的同一个 robot profile。各端启动或选择时会交叉校验 `robot.profile` 一致性。
- 策略服务启动时从 `models/` 选择最新模型，或用 `--model` 指定模型目录；学习端校验 `model.safetensors` 内的模型和档案身份。

## 校验与运行

三端入口在内存中展开 binding 选定的配方。`--robot/--backend/--task` 与 `--config` 分别选择方案和直接配方：

```bash
# mujoco 容器（工作目录 /app）
scripts/backend.sh --robot <robot> --backend <backend> --task <task> --role teacher \
  --seed 0 --run-id teacher-backend
./env.sh viewer --robot <robot> --backend <backend> --task <task>
scripts/backend.sh --robot <robot> --backend <backend> --task <task> --role policy \
  --seed 0 --run-id policy-backend

# runtime 容器（工作目录 /app）；一次构建覆盖全部接口包与 Policy SDK（写入 output/build/）
scripts/build_sdk.sh
scripts/record.sh --robot <robot> --backend <backend> --task <task> \
  --purpose pilot --count 1 --max-consecutive-failures 1
scripts/client_interactive.sh --robot <robot> --backend <backend> --task <task> --seed 0
scripts/client_policy.sh --robot <robot> --backend <backend> --task <task> \
  --reset-seed 0 --run-id policy-run

# lerobot 容器（工作目录 /app）
scripts/convert.sh --robot <robot> --backend <backend> --task <task>
scripts/train.sh --robot <robot> --backend <backend> --task <task> --mode smoke
scripts/server_policy.sh --robot <robot> --backend <backend> --task <task> \
  --model /data/<robot>/<backend>/<task>/models/<act_timestamp>
```

数据、模型、日志和诊断分别写入 `output/<robot>/<backend>/<task>/{datasets,models,logs,debug}/`。

## 更换内容时改哪里

| 更换内容 | 配置切换（现有实现） | 需要新增实现 |
| --- | --- | --- |
| checkpoint | 用 `--model` 指定新的模型目录 | 观测、动作或分块契约变化时同步适配器、处理器与档案 |
| 学习算法 | 已有第二个适配器实现时，切换 `learning.adapter` 及相应配方引用 | 在 `learning/` 实现适配器与 bundle 校验（如 RL），再在 binding 选择 |
| 机器人 | 改 `profiles.*` 及 control/stream/follower/scene/task 配方引用 | 在 `shared/robots/<robot>/` 添加资产与档案，并在对应仿真/硬件后端补全驱动与插件 |
| 执行后端 | 改 `backend.implementation` 与后端配方 | 新增 `simulation/<backend>/` 或 `hardware/<backend>/` 并实现 `robot_api_msgs` |
| 任务 | 改 `backend.task_type`、task 配方与扩展引用 | 任务逻辑在所属端实现；任务专用接口放 `extensions/` |
| 新观测/动作能力 | 在现有协议内用已支持的字段组合出新档案 | 在 `shared/common/interfaces` 演进公共协议，并在生产/消费两端实现 |

目录导航：[方案目录](../) · [marvin/mujoco/stack 方案](../marvin/mujoco/stack/README.md) · [文档索引](../../../docs/README.md)
