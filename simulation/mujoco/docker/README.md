# MuJoCo Docker 构建及运行维护

本目录的 Compose 只管理 `simulation` 空闲容器。进入后手动运行
`/app/scripts/run.sh`；构建和启动不会自动加载场景或开始数采。

## 路径与磁盘

Panda 模型位于 `../assets/franka_emika_panda/`，随仿真项目只读挂到
`/app/assets/franka_emika_panda/`。仓库的 `var/` 挂到 `/data`，
`.build/ros-dev/` 只读挂到 `/opt/ros-dev`。首次使用先核对实际磁盘：

```bash
cd /path/to/sim_dual_arm
mkdir -p var .build
findmnt -T var
df -hT var
findmnt -T .build
df -hT .build
```

如需数据盘，修改本目录 `docker-compose.yml` 中 `/data` 对应的宿主来源，
并确保它指向预期磁盘。仓库位置不会改变 Docker 镜像存储。
Compose 中 `../` 指本端 `simulation/mujoco`，`../../../` 指仓库根；不需要
`PROJECT_ROOT`、`DATA_ROOT` 等宿主环境变量。当前 `user: "1000:1000"` 对应本机用户，
换机器时先核对 `id -u` 和 `id -g`。

## 构建、进入和检查

需要 Docker Compose、NVIDIA 驱动和 NVIDIA Container Toolkit。镜像基于
`ros:jazzy-ros-base-noble`，安装固定的 [Ruckig wheel](wheels/README.md) 和
`requirements.lock`。容器内路径及 ROS/EGL 设置在 `../configs/container.env`。

```bash
cd /path/to/sim_dual_arm/simulation/mujoco/docker
docker compose config --quiet
docker compose build
docker compose up -d
docker compose exec simulation bash
```

进入容器后：

```bash
scripts/run.sh --help
scripts/run.sh environment --run-id env-001
scripts/run.sh scene --run-id scene-001
scripts/run.sh kinematics --run-id kin-001
```

单次命令也可在宿主本目录运行
`docker compose exec simulation scripts/run.sh scene --run-id scene-002`。
检查报告写入 `/data/reports/sim-check/<run-id>/`；使用新的 run-id，不覆盖旧报告。

## 与 runtime 连接

只有 `backend` 需要 ROS SDK。先在宿主构建并启动 runtime 容器：

```bash
cd /path/to/sim_dual_arm/runtime/ros2/docker
docker compose build
docker compose up -d
docker compose exec runtime bash
# runtime 容器内：
runtime/ros2/scripts/run.sh build-ros-sdk
```

仿真与运行端使用同一 Docker 项目名和 `ros` 网络；两端的 `ROS_DOMAIN_ID=42`
写在各自 `configs/container.env`。回到 simulation 容器后准备配置并启动：

```bash
scripts/run.sh prepare \
  --template /app/configs/backend/backend_stack_inference.yaml \
  --seed 42 --run-id prep-001 --output /data/prepared/prep-001
scripts/run.sh backend \
  --config /data/prepared/prep-001/backend.yaml --run-id backend-001
```

`backend` 前台运行，用 Ctrl+C 正常停止。采集、控制和 Policy 调用见
[项目使用指南](../../../README.md#运行端)。

## 停止与排查

在本目录仅停止并移除仿真容器，bind 数据保留：

```bash
docker compose stop simulation
docker compose rm -f simulation
```

不要对单端使用 `down --remove-orphans`，以免影响同项目名的其他服务。
GPU/EGL 失败先检查宿主 `nvidia-smi` 与 `docker compose config` 中的 GPU 请求；
SDK 失败先在 runtime 重新构建 SDK。空闲容器日志只反映 `sleep`，业务错误查看
该命令的终端输出及 `/data` 下对应 run 报告。

目录导航：[上级目录](../README.md)。
