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

首次在本机使用学习端前，先按 [`third_party/README.md`](../../third_party/README.md) 准备 `third_party/` 离线源码与权重，在 `runtime` 容器内执行 `scripts/build_sdk.sh` 生成 Policy SDK（输出至 `output/build/generated/policy-sdk/`），再构建并进入 `lerobot` 容器：

```bash
./env.sh build lerobot
./env.sh lerobot
```

在 `lerobot` 容器内执行，使用 `--robot <robot> --backend <backend> --task <task>` 或 `--config <recipe>` 选择配方。训练模式、步数上限、模型路径和设备等常用设置有专用参数，其余参数在对应配方中修改：

- `scripts/convert.sh`：将合格采集数据转换为训练/验证 LeRobot 数据集（输出到 `output/<robot>/<backend>/<task>/datasets/lerobot/`）。
- `scripts/train.sh`：训练 ACT 模型，直接写出 `output/<robot>/<backend>/<task>/models/act_<时间戳>/model.safetensors` 与 `training_args.yaml`；训练日志、指标和终态摘要输出到 `output/<robot>/<backend>/<task>/logs/training/act_<时间戳>/`。
- `scripts/server_policy.sh`：启动 gRPC Policy 推理服务（默认监听 `127.0.0.1:50051`），默认读取 `output/<robot>/<backend>/<task>/models/` 下最新的 `act_*` 模型，也可用 `--model` 显式指定模型目录或 `model.safetensors`。
- `scripts/test.sh`：运行容器内 `pytest` 测试。

```bash
scripts/convert.sh --robot marvin --backend mujoco --task stack
scripts/train.sh --robot marvin --backend mujoco --task stack --mode full
scripts/server_policy.sh --robot marvin --backend mujoco --task stack
```

> `convert.sh` 和 `train.sh` 启动时会打印配置文件与有效参数，终端使用原地刷新的单行进度条，详细日志写入启动时提示的日志文件。完整参数说明见 [脚本入口与使用指南](../../docs/features/脚本入口与使用指南.md)。

---

## 远端 GPU 机器部署与 SSH 隧道联调

除了三端容器全部跑在本机之外，也可以把**学习端放在远端 GPU 机器**（训练 + 策略推理），**本机只跑运行端与后端**，两者通过 SSH 隧道连接：

```text
本机: mujoco ◀──ROS 2──▶ runtime ──gRPC──▶ 127.0.0.1:50051
                                                  │
                                              SSH 隧道
                                                  ▼
远端:                                       127.0.0.1:50051 (lerobot)
```

### 第一步：搭建远端学习端环境（二选一）

#### 方式 A：云服务器（无 Docker，使用仓库 `uv` 初始化脚本）

适用于没有 Docker、只提供 NVIDIA 驱动与 CUDA 的云服务器。仓库内 [`setup_colab_learning.sh`](../../docs/colab/scripts/setup_colab_learning.sh) 根据 [`learning-source.json`](docker/learning-source.json) 一步完成全部初始化：准备并校验 `third_party/`、在 `output/_system/uv/` 与 `.venv/` 中安装 `uv` 与 Python 3.12、选择 PyTorch 与学习端依赖、生成 Policy SDK（`output/build/generated/policy-sdk/`）、写出 `.env.learning` 并执行自检。

以下全部命令在**远端仓库根目录**执行：

```bash
bash docs/colab/scripts/setup_colab_learning.sh
```

常用可选参数：
- `--cache-dir <dir>`：指定持久化缓存目录（复用已下载的 LeRobot 源码包与 ResNet18 权重）。
- `--torch-index-url <url>`：显式覆盖 PyTorch wheel 源。

激活 `.env.learning` 后即可直接运行学习端脚本（新开终端执行 `source .env.learning`）：

```bash
source .env.learning
bash learning/lerobot/scripts/convert.sh --robot marvin --backend mujoco --task stack
bash learning/lerobot/scripts/train.sh --robot marvin --backend mujoco --task stack --mode full
bash learning/lerobot/scripts/server_policy.sh --robot marvin --backend mujoco --task stack
```

#### 方式 B：GPU 工作站（有 Docker，使用项目容器）

适用于已安装 NVIDIA 驱动、Docker Compose 插件和 NVIDIA Container Toolkit 的工作站。所有命令在**远端仓库根目录**执行：

1. **准备第三方源码与权重**（详见 [`third_party/README.md`](../../third_party/README.md)）。
2. **创建宿主机输出目录、构建镜像、生成 Policy SDK 并启动容器**：
   `lerobot` 容器以只读方式将宿主机 `output/build/generated/policy-sdk/` 挂载至 `/opt/policy-sdk`：

   ```bash
   mkdir -p output/build/generated/policy-sdk
   ./env.sh build lerobot
   docker compose -f learning/lerobot/docker/docker-compose.yml run --rm --no-deps \
     -v "$PWD/output/build:/build" lerobot \
     python3 /shared/common/tools/scripts/generate_policy_sdk.py --output /build/generated/policy-sdk
   ./env.sh up lerobot
   ```

3. **在容器内运行转换、训练与策略服务**：

   ```bash
   ./env.sh exec lerobot scripts/convert.sh --robot marvin --backend mujoco --task stack
   ./env.sh exec lerobot scripts/train.sh --robot marvin --backend mujoco --task stack --mode full
   ./env.sh exec lerobot scripts/server_policy.sh --robot marvin --backend mujoco --task stack
   ```

### 第二步：本机通过 SSH 隧道联调（通用）

1. **同步数据或模型（在本机仓库根目录执行）**：

   ```bash
   # 将本机采集的原始数据传给远端用于转换和训练
   rsync -aR output/marvin/mujoco/stack/datasets/raw/ user@remote:/path/to/sim_dual_arm/

   # 或把本机已有模型传给远端用于推理
   rsync -aR output/marvin/mujoco/stack/models/act_YYYYMMDD_HHMMSS/ user@remote:/path/to/sim_dual_arm/

   # 训练完成后，将远端模型拉回本机
   rsync -aR user@remote:/path/to/sim_dual_arm/./output/marvin/mujoco/stack/models/act_YYYYMMDD_HHMMSS/ .
   ```

2. **远端启动策略服务并保持运行**：按方式 A 或方式 B 启动 `server_policy.sh`（默认监听 `127.0.0.1:50051`）。
3. **本机建立 SSH 本地转发（在本机宿主机新终端保持运行）**：

   ```bash
   ssh -NT -o ExitOnForwardFailure=yes -L 127.0.0.1:50051:127.0.0.1:50051 user@remote
   ```

   > 若本机 `50051` 端口已被占用，可转发至其他本地端口（如 `-L 127.0.0.1:15051:127.0.0.1:50051`），并在推理命令中追加 `--policy-endpoint 127.0.0.1:15051`。

4. **本机启动后端与推理**：

   ```bash
   # 单局或交互推理（在 runtime 容器内执行）
   ./env.sh exec runtime scripts/client_policy.sh --robot marvin --backend mujoco --task stack \
     --reset-seed 20000
   ./env.sh exec runtime scripts/client_interactive.sh --robot marvin --backend mujoco --task stack \
     --seed 20000

   # 批量并行推理（在本机宿主机仓库根目录执行）
   ./inference_parallel.sh --robot marvin --backend mujoco --task stack \
     --count 10 --workers auto
   ```

---

## Google Colab 云端数据转换与训练

如使用 Google Colab 高 RAM 运行时与 Google Drive 进行数据集转换、缓存与模型中转，可配合仓库提供的两个 Notebook：

- [`docs/colab/sync_drive_rclone.ipynb`](../../docs/colab/sync_drive_rclone.ipynb)：在本机 Python 环境中通过 `rclone` 上传仅含合格 `export/`（不含 MCAP）的 `raw_exports.tar` 或已转换的 `lerobot.tar`，查看云端清单以及拉回转换数据集、训练模型与日志。
- [`docs/colab/train_act_colab.ipynb`](../../docs/colab/train_act_colab.ipynb)：在 Colab 运行时中调用 [`setup_colab_learning.sh`](../../docs/colab/scripts/setup_colab_learning.sh) 自动配置环境，支持利用 `/dev/shm` 内存盘执行大内存数据转换并自动回传 `lerobot.tar`，随后在 GPU 运行时完成资源探测、按 batch 比例自适应缩放训练步数、周期绘制损失曲线并将模型回传至 Google Drive。

---

目录导航：[上级目录](../../README.md) · [脚本入口与使用指南](../../docs/features/脚本入口与使用指南.md) · [第三方源码说明](../../third_party/README.md) · [Colab 训练流程](../../docs/colab/train_act_colab.ipynb)
