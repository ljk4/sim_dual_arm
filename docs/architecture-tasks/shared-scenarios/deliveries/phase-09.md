# 阶段 09 交接：部署要求归属与公共检查

- 日期：2026-11-12（Asia/Shanghai）。状态：完成（静态配置与测试）；未进入阶段 10。
- 依据：总要求、阶段 09、progress、阶段 08 交接。未改 Compose、Dockerfile、`container.env`、控制参数、数据、模型或台账。

## 声明位置与字段

每个实现与 Dockerfile/Compose 同级维护一份 `docker/deployment.yaml`：

- `simulation/mujoco/docker/deployment.yaml`
- `runtime/ros2/docker/deployment.yaml`
- `learning/lerobot/docker/deployment.yaml`

字段只覆盖无法由 Compose 结构本身判断用途的要求：

| 字段 | 含义 |
| --- | --- |
| `service` | 声明约束的 Compose 服务名 |
| `source.mount` | 只读源码挂载目标；同时作为容器工作目录 |
| `source.scope` | 源码绑定范围：`implementation`（本端目录）或 `repository`（仓库根） |
| `mounts` | 除 `/shared` 外的必需挂载及 `read-only`/`writable` 模式 |
| `networks` | 必需网络集合 |
| `gpu` | 是否需要 `runtime: nvidia` 与 `gpus: all` |
| `commands` | `scripts/run.sh` 必须提供的命名命令 |

当前值：simulation 使用 `/app`、仅绑定 `simulation/mujoco`、`ros` 网络、GPU、挂载 `/data` 可写与 `/opt/ros-dev` 只读；runtime 使用 `/workspace`、仓库根只读、`ros` 与 `learning` 网络、不需要 GPU；learning 使用 `/workspace`、仓库根只读、仅 `learning` 网络、GPU，并额外只读挂载 `/third_party`。声明不含挂载源路径、构建配置、环境变量或依赖锁。

## 检查规则归属

`shared/common/tools/scripts/check_dockerfiles.py` 继续负责共同规则：唯一入口与空闲启动、Dockerfile 不复制业务源码、不内嵌程序、依赖镜像与 `:current` 标签、`/shared` 只读、bind 源为直接路径、禁止宿主虚拟环境与 Docker socket、`container.env` 的 `PROJECT_CONTAINER`/`PROJECT_ROOT`。

声明驱动的检查只作用于对应服务：源码挂载与工作目录、必需挂载及模式、未声明挂载、网络集合、GPU、`run.sh` 命名命令。公共代码不再包含 `simulation`/`runtime`/`learning` 到 GPU、挂载或命令的映射；`source.scope` 决定源码绑定到本端目录还是仓库根。声明缺失、字段非法或实际 Compose 违背声明时，错误信息同时给出 Compose 文件、声明文件与字段。

## 验证结果

- 三个实际部署：`python3 shared/common/tools/scripts/check_dockerfiles.py` → `Dockerfile/Compose boundaries passed: 3 Dockerfiles, 3 idle environment files`，exit 0。
- 声明测试：`tests/unit/test_container_build.py` 28 passed。沿用原有共同规则用例并改为传入各端声明；新增一个 CPU 仿真用例：同一 MuJoCo Compose 删除 `runtime`/`gpus` 后把声明改为 `gpu: false`，检查通过。
- 声明缺失要求时明确失败：删除 MuJoCo `gpus` 得到 `service simulation must use runtime: nvidia and gpus: all as declared in simulation/mujoco/docker/deployment.yaml: gpu`；删除 `/opt/ros-dev` 挂载得到 `must mount /opt/ros-dev read-only as declared in ...: mounts./opt/ros-dev`。输出记录在 `.build/phase-09-20261112/declaration-check.txt`。
- ruff：`check_dockerfiles.py` 与 `test_container_build.py` 通过。宿主机无 pytest/ruff，使用 `uv run --no-project --python 3.12 --with pytest --with pyyaml --with pydantic` 与 `--with ruff` 运行。

## 已知问题

- 只做静态配置检查；未启动容器、未重建镜像，也未运行 Compose 实际启动。
- `hardware` 没有当前实现，检查器仍只要求 `simulation`/`runtime`/`learning` 三端各一份声明；新增实现需同时提供 `docker/deployment.yaml`。
- CPU 仿真验证覆盖 Compose 与声明的检查路径；未提供完整 CPU 仿真镜像或运行配方。
- 未 commit/push/stash/reset/clean。
