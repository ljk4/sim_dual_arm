# Simulation 容器构建与使用

- **基础镜像**：`ros:jazzy-ros-base-noble` + 锁定版本的 MuJoCo 与 Ruckig（需要 NVIDIA GPU 与 Container Toolkit）。
- **空闲启动**：容器以 `sleep infinity` 运行，命令均通过容器内 `scripts/*.sh` 执行。

## 常用操作

推荐使用仓库根目录 `./env.sh` 管理容器：

```bash
# 宿主机：构建并启动容器
./env.sh up simulation
./env.sh enter simulation

# 容器内：启动 ROS 2 仿真后端（需先在 runtime 容器运行过 scripts/build_sdk.sh）
scripts/backend.sh --robot panda_sim --task stack --role teacher --seed 0 --run-id teacher-backend
```

### 实时可视化（Viewer）

仿真后端默认无窗口离屏渲染。如需查看实时画面，在宿主机执行：

```bash
./env.sh viewer --robot panda_sim --task stack --role teacher
```

> 若遇到 X11 权限报错，先在宿主机执行 `xhost +SI:localuser:$(id -un)`。

### 停止与清理

```bash
./env.sh stop simulation
./env.sh rm simulation
```

> 切勿对单端执行 `docker compose down --remove-orphans`，以免误停共享网络的其他容器。
