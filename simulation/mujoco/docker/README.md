# MuJoCo 容器构建与运行指南

本目录的 `docker-compose.yml` 管理独立的 `simulation` 容器（默认镜像 `sim-dual-arm/simulation:current`）。容器启动后保持 `sleep infinity` 空闲状态，所有仿真与检查命令均在进入容器后通过 `/app/scripts/run.sh` 手动执行。

---

## 1. 挂载边界与磁盘准备

`simulation` 容器实行最小化权限挂载：

- 本端目录 `simulation/mujoco/` 只读挂载至 `/app:ro`（含 `/app/assets/franka_emika_panda/` 模型资产）。
- 仓库 `shared/` 只读挂载至 `/shared:ro`；`interfaces/` 提供契约，`scenarios/panda_stack/profiles/` 提供机器人档案，`python/` 提供 `ros_support`、`run_logging`、`trajectory` 与 `contracts` 源码，`tools/` 提供 setup、DDS 与检查脚本。
- ROS 2 SDK 只读挂载至 `/opt/ros-dev:ro`（宿主机 `.build/ros-dev/`）。
- 运行产物目录可写挂载至 `/data`（宿主机 `var/`）。

首次运行前，请在仓库根目录创建 `.build` 与 `var` 目录，并确认宿主机用户 UID/GID 与 Compose 中的 `user: "1000:1000"` 一致。

---

## 2. 构建与离线检查

前置条件：已安装 Docker Compose、NVIDIA 显卡驱动与 NVIDIA Container Toolkit。镜像基于 `ros:jazzy-ros-base-noble`，通过单次 `pip install --require-hashes -r /opt/dependencies/requirements.lock` 安装固定版本的 MuJoCo 及随仓提供的 [Ruckig wheel](wheels/README.md)。

```bash
cd simulation/mujoco/docker
docker compose build
docker compose up -d
docker compose exec simulation bash
```

在容器内运行环境、三相机视野与运动学净空检查（无需预先构建 ROS SDK，检查报告写入 `/data/reports/sim-check/<run-id>/`）：

```bash
scripts/run.sh --help
scripts/run.sh environment --run-id env-001
scripts/run.sh scene --run-id scene-001
scripts/run.sh kinematics --run-id kin-001
# 仓库单测需临时只读挂载完整仓库并使用相应 PROJECT_ROOT/PYTHONPATH；标准挂载不含 tests/。
```

---

## 3. 启动 ROS 2 仿真后端

启动 `backend` 前，须先在 `runtime` 容器中执行 `runtime/ros2/scripts/run.sh build-ros-sdk --scenario panda_stack --shared-root /shared` 生成 `/opt/ros-dev`。无方案选择的核心构建写入 `.build/ros-core`，不会覆盖该完整 SDK。

在 `simulation` 容器内准备后端配置并前台启动服务：

```bash
scripts/run.sh prepare \
  --scenario panda_stack --role policy \
  --bundle /data/learning/bundles/<verified-bundle> \
  --seed 20001 \
  --run-id sim-001 \
  --output /data/prepared/sim-001

# 直接配置模式仍可用：--template <recipe>

scripts/run.sh backend \
  --config /data/prepared/sim-001/backend.yaml \
  --run-id sim-001
```

按 `Ctrl-C` 可安全停止后端并完成日志落盘。

---

## 4. 实时 MuJoCo 可视化（Viewer）

`backend` 进程始终采用无窗口 NVIDIA EGL 离屏渲染。如需在图形桌面查看实时物理状态，可在宿主机通过 `docker compose run` 临时注入 X11 显示套接字启动只读 `viewer`（若遇权限错误，先在宿主机执行 `xhost +SI:localuser:$(id -un)`）：

```bash
cd simulation/mujoco/docker
docker compose run --rm \
  -e DISPLAY="$DISPLAY" \
  -e XAUTHORITY=/tmp/.Xauthority \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${XAUTHORITY:-/dev/null}:/tmp/.Xauthority:ro" \
  simulation scripts/run.sh viewer \
  --config /data/prepared/sim-001/backend.yaml
```

`viewer` 仅订阅后端的完整物理状态快照进行渲染，不推进物理步，也不介入控制回路。

---

## 5. 容器停止与清理

仅停止并移除 `simulation` 容器（保留共享网络与 `/data` 产物）：

```bash
cd simulation/mujoco/docker
docker compose stop simulation
docker compose rm -f simulation
```

> 切勿对单端执行 `docker compose down --remove-orphans`，以免误停共享同一项目名 `sim-dual-arm` 的 `runtime` 或 `learning` 容器。

目录导航：[上级目录](../README.md)
