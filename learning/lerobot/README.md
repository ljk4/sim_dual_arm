# LeRobot ACT 学习与策略服务

提供无 ROS 依赖的 LeRobot 数据集转换、ACT 训练与 gRPC Policy 推理服务（默认监听 `127.0.0.1:50051`）。

## 目录结构

| 目录 / 文件 | 说明 |
| :--- | :--- |
| `src/dual_arm_learning/` | 核心 Python 包，按 `dataset/`（数据转换）、`training/`（ACT 训练）、`serving/`（gRPC 策略服务）与 `adapters/`（模型适配器）组织 |
| `configs/` | 容器环境配置 `container.env` |
| `docker/` | `Dockerfile`、[`docker-compose.yml`](docker/docker-compose.yml)（`network_mode: host`）与 [`learning-source.json`](docker/learning-source.json)（固定 LeRobot 版本、Python 版本与基础镜像声明） |
| `scripts/` | 容器内单职责入口脚本：`convert.sh`、`train.sh`、`server_policy.sh`、`test.sh` |

---

## 本机容器快速使用与常用脚本

首次在本机使用学习端前，先按 [`third_party/README.md`](../../third_party/README.md) 准备 `third_party/` 离线源码与权重，在 `runtime` 容器内执行 `scripts/build_sdk.sh` 生成 Policy SDK，再构建并进入 `learning` 容器：

```bash
./env.sh build learning
./env.sh learning
```

在 `learning` 容器内执行，使用 `--robot <robot> --task <task>` 或 `--config <recipe>` 选择配方。训练模式、步数上限、模型路径和设备等常用设置有专用参数，其余参数在对应配方中修改：

- `scripts/convert.sh`：将合格采集数据转换为训练/验证 LeRobot 数据集（输出到 `var/<robot>/<task>/datasets/lerobot/`）。
- `scripts/train.sh`：训练 ACT 模型，直接写出 `var/<robot>/<task>/models/act_<时间戳>/model.safetensors` 与 `training_args.yaml`；训练日志、指标和终态摘要输出到 `var/<robot>/<task>/logs/training/act_<时间戳>/`。
- `scripts/server_policy.sh`：启动 gRPC Policy 推理服务（默认监听 `127.0.0.1:50051`），默认读取 `var/<robot>/<task>/models/` 下最新的 `act_*` 模型，也可用 `--model` 显式指定模型目录或 `model.safetensors`。
- `scripts/test.sh`：运行容器内 `pytest` 测试。

```bash
scripts/convert.sh --robot marvin_sim --task stack
scripts/train.sh --robot marvin_sim --task stack --mode full
scripts/server_policy.sh --robot marvin_sim --task stack
```

> `convert.sh` 和 `train.sh` 启动时会打印配置文件与有效参数，终端使用原地刷新的单行进度条，详细日志写入启动时提示的日志文件。完整参数说明见 [脚本入口与使用指南](../../docs/features/脚本入口与使用指南.md)。

---

## 远端 GPU 机器部署与 SSH 隧道联调

除了三端容器全部跑在本机之外，也可以把**学习端放在远端 GPU 机器**（训练 + 策略推理），**本机只跑运行端与仿真端**，两者通过 SSH 隧道连接：

```text
本机: simulation ◀──ROS 2──▶ runtime ──gRPC──▶ 127.0.0.1:50051
                                                     │
                                                 SSH 隧道
                                                     ▼
远端:                                          127.0.0.1:50051 (learning)
```

### 第一步：搭建远端学习端环境（二选一）

两种方式均把工具、依赖和运行产物限制在当前仓库目录内，不修改系统环境或用户主目录（`~`）。

#### 方式 A：云服务器（无 Docker，使用仓库 `uv` 初始化脚本）

适用于没有 Docker、只提供 NVIDIA 驱动与 CUDA 的云服务器。仓库内 [`setup_colab_learning.sh`](../../docs/colab/scripts/setup_colab_learning.sh) 根据 [`learning-source.json`](docker/learning-source.json) 一步完成全部初始化：准备并校验 `third_party/`（LeRobot 源码与 ResNet18 权重）、在 `var/_system/uv/` 与 `.venv/` 中安装 `uv` 与 Python 3.12、按驱动 CUDA 版本自动选择 PyTorch（`cu128` / `cu126` / `cpu`）及全部学习端依赖、生成 Policy SDK、写出 `.env.learning` 并执行导入与 CUDA 自检。

以下全部命令在**远端仓库根目录**执行：

```bash
bash docs/colab/scripts/setup_colab_learning.sh
```

常用可选参数：
- `--cache-dir <dir>`：指定持久化缓存目录（复用已下载的 LeRobot 源码包与 ResNet18 权重）。
- `--torch-index-url <url>`：显式覆盖 PyTorch wheel 源。

激活 `.env.learning` 后即可直接运行学习端脚本（以后每次新开终端只需先执行 `source .env.learning`）：

```bash
source .env.learning
bash learning/lerobot/scripts/convert.sh --robot marvin_sim --task stack
bash learning/lerobot/scripts/train.sh --robot marvin_sim --task stack --mode full
bash learning/lerobot/scripts/server_policy.sh --robot marvin_sim --task stack
```

#### 方式 B：GPU 工作站（有 Docker，使用项目容器）

适用于已安装 NVIDIA 驱动、Docker Compose 插件和 NVIDIA Container Toolkit 的工作站。容器默认使用 `network_mode: host`，`server_policy.sh` 直接绑定远端 `127.0.0.1:50051`。所有命令在**远端仓库根目录**执行：

1. **拉取仓库并准备第三方源码与权重**（详见 [`third_party/README.md`](../../third_party/README.md)）。
2. **创建宿主机输出目录、构建镜像、生成 Policy SDK 并启动容器**：
   先由当前用户创建 `build` 与 `var` 目录避免挂载归 root 所有，`learning` 容器以只读方式将宿主机 `build/generated/policy-sdk/` 挂载至 `/opt/policy-sdk`：

   ```bash
   mkdir -p build var
   ./env.sh build learning
   docker compose -f learning/lerobot/docker/docker-compose.yml run --rm --no-deps \
     -v "$PWD/build:/build" learning \
     python3 /shared/common/tools/scripts/generate_policy_sdk.py --output /build/generated/policy-sdk
   ./env.sh up learning
   ```

3. **在容器内运行转换、训练与策略服务**：

   ```bash
   ./env.sh exec learning scripts/convert.sh --robot marvin_sim --task stack
   ./env.sh exec learning scripts/train.sh --robot marvin_sim --task stack --mode full
   ./env.sh exec learning scripts/server_policy.sh --robot marvin_sim --task stack
   ```

### 第二步：本机通过 SSH 隧道联调（通用）

远端环境就绪后，按以下顺序完成数据/模型同步与跨机推理（示例中的 `user@remote:/path/to/sim_dual_arm/` 请替换为实际 SSH 地址与远端仓库路径）：

1. **同步数据或模型（在本机仓库根目录执行）**：

   ```bash
   # 将本机采集的原始数据传给远端用于转换和训练
   rsync -aR var/marvin_sim/stack/datasets/raw/ user@remote:/path/to/sim_dual_arm/

   # 或把本机已有模型传给远端用于推理
   rsync -aR var/marvin_sim/stack/models/act_YYYYMMDD_HHMMSS/ user@remote:/path/to/sim_dual_arm/

   # 训练完成后，将远端模型拉回本机
   rsync -aR user@remote:/path/to/sim_dual_arm/./var/marvin_sim/stack/models/act_YYYYMMDD_HHMMSS/ .
   ```

2. **远端启动策略服务并保持运行**：按方式 A 或方式 B 启动 `server_policy.sh`（默认监听 `127.0.0.1:50051`）。
3. **本机建立 SSH 本地转发（在本机宿主机新终端保持运行）**：
   本机容器共享宿主机回环网络，将远端 `127.0.0.1:50051` 直接转发到本机 `127.0.0.1:50051`：

   ```bash
   ssh -NT -o ExitOnForwardFailure=yes -L 127.0.0.1:50051:127.0.0.1:50051 user@remote
   ```

   > 若本机 `50051` 端口已被本地策略服务占用，可转发至其他本地端口（如 `-L 127.0.0.1:15051:127.0.0.1:50051`），并在下一步推理命令中追加 `--policy-endpoint 127.0.0.1:15051`。

4. **本机启动仿真与推理**：
   在本机按[推理仿真后端启动步骤](../../README.md#34-直接启动推理仿真后端)启动仿真后端，直接运行推理（默认连接 `127.0.0.1:50051`）：

   ```bash
   # 单局或交互推理（在 runtime 容器内执行）
   ./env.sh exec runtime scripts/client_policy.sh --robot marvin_sim --task stack \
     --reset-seed 20000
   ./env.sh exec runtime scripts/client_interactive.sh --robot marvin_sim --task stack \
     --seed 20000

   # 批量并行推理（在本机宿主机仓库根目录执行）
   ./inference_parallel.sh --robot marvin_sim --task stack \
     --count 10 --workers auto
   ```

---

## Google Colab 云端训练

如使用 Google Colab GPU 与 Google Drive 进行数据集与模型中转，可配合仓库提供的两个 Notebook：

- [`docs/colab/sync_drive_rclone.ipynb`](../../docs/colab/sync_drive_rclone.ipynb)：在本机 Python 环境中通过 `rclone` 打包上传 `datasets/lerobot/`、查看云端清单以及拉回训练好的模型与日志。
- [`docs/colab/train_act_colab.ipynb`](../../docs/colab/train_act_colab.ipynb)：在 Colab GPU 运行时中调用 [`setup_colab_learning.sh`](../../docs/colab/scripts/setup_colab_learning.sh) 自动配置环境，执行资源探测（最优 `batch_size` / `num_workers` / `prefetch_factor`）、按 batch 比例自适应缩放训练步数、周期绘制训练/验证损失曲线并将模型回传至 Google Drive。

---

目录导航：[上级目录](../../README.md) · [脚本入口与使用指南](../../docs/features/脚本入口与使用指南.md) · [第三方源码说明](../../third_party/README.md) · [Colab 训练流程](../../docs/colab/train_act_colab.ipynb)
