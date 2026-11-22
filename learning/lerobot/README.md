# LeRobot ACT 学习与策略服务

提供无 ROS 依赖的 LeRobot 数据集转换、ACT 训练与 gRPC Policy 推理服务。

除了全部四端跑在本机之外，也可以把**学习端放在远端 GPU 机器**（训练 + 策略推理），**本机只跑运行端与仿真端**，两者通过 SSH 隧道连接：

```text
本机: simulation ◀──ROS 2──▶ runtime ──gRPC──▶ host.docker.internal:15051
                                                         │
                                                     SSH 隧道
                                                         ▼
远端:                                          127.0.0.1:50051 (learning)
```

---

## 第一步：搭建远端学习端环境（二选一）

两种方式均把工具、依赖和运行产物限制在当前仓库目录内，不修改系统环境或用户主目录（`~`）。

### 方式 A：云服务器（无 Docker，使用 `uv` 在本目录配环境）

适用于没有 Docker、只提供 NVIDIA 驱动与 CUDA 的云服务器。以下全部命令在**远端仓库根目录**执行：

1. **拉取仓库并准备第三方源码与权重**（详见 [`third_party/README.md`](../../third_party/README.md)）。
2. **在本目录内安装 `uv`、Python 3.12 与依赖**：所有文件收敛在已被 `.gitignore` 忽略的 `var/_system/uv/` 和 `.venv/` 中，不会写入 `~/.local` 或 `~/.bashrc`。

   ```bash
   export UV_INSTALL_DIR="$PWD/var/_system/uv/bin"
   export UV_PYTHON_INSTALL_DIR="$PWD/var/_system/uv/python"
   export UV_CACHE_DIR="$PWD/var/_system/uv/cache"
   export PATH="$UV_INSTALL_DIR:$PATH"

   mkdir -p "$UV_INSTALL_DIR" "$UV_PYTHON_INSTALL_DIR" "$UV_CACHE_DIR" build var
   curl -LsSf https://astral.sh/uv/install.sh | env UV_NO_MODIFY_PATH=1 UV_INSTALL_DIR="$UV_INSTALL_DIR" sh

   uv python install 3.12
   uv venv --python 3.12 .venv
   source .venv/bin/activate

   # 按显卡 CUDA 版本选择 PyTorch 轮子（示例为 CUDA 12.4）
   uv pip install torch torchvision --index-url https://download.pytorch.org/whl/cu124
   uv pip install -e third_party/src/lerobot-act-v0.6.1
   uv pip install pydantic pyyaml jsonschema pillow pyarrow safetensors grpcio grpcio-tools av
   ```

3. **生成 Policy SDK 与目录内环境文件 `.env.learning`**（已被 `.gitignore` 忽略）：

   ```bash
   python3 shared/common/tools/scripts/generate_policy_sdk.py

   cat > .env.learning <<EOF
   source "$PWD/.venv/bin/activate"
   export PROJECT_CONTAINER=1
   export PROJECT_ROOT="$PWD"
   export SHARED_ROOT="$PWD/shared"
   export DATA_ROOT="$PWD/var"
   export BUILD_ROOT="$PWD/build"
   export THIRD_PARTY_ROOT="$PWD/third_party"
   export POLICY_SDK_ROOT="$PWD/build/generated/policy-sdk"
   export HF_HOME="$PWD/var/_system/hf-cache"
   export HF_HUB_OFFLINE=1 HF_DATASETS_OFFLINE=1 TRANSFORMERS_OFFLINE=1
   export PATH="$PWD/var/_system/uv/bin:\$PATH"
   export PYTHONPATH="$PWD/learning/lerobot/src:$PWD/shared/common/python/run_logging/src:$PWD/shared/common/python/contracts/src:$PWD/third_party/src/lerobot-act-v0.6.1/src:$PWD/build/generated/policy-sdk"
   EOF
   ```

4. **激活环境并运行**（以后每次新开终端只需先执行 `source .env.learning`）：

   ```bash
   source .env.learning
   bash learning/lerobot/scripts/convert.sh --robot marvin_sim --task stack
   bash learning/lerobot/scripts/train.sh --robot marvin_sim --task stack --mode full
   bash learning/lerobot/scripts/server_policy.sh --robot marvin_sim --task stack
   ```

### 方式 B：GPU 工作站（有 Docker，使用项目容器）

适用于已安装 NVIDIA 驱动、Docker Compose 插件和 NVIDIA Container Toolkit 的工作站。所有命令在**远端仓库根目录**执行：

1. **拉取仓库并准备第三方源码与权重**（详见 [`third_party/README.md`](../../third_party/README.md)）。
2. **暴露工作站本机回环端口**：在工作站的 [`learning/lerobot/docker/docker-compose.yml`](docker/docker-compose.yml) 中为 `services.learning` 添加以下 `ports` 配置（仅监听工作站 `127.0.0.1`，不对外开放）：

   ```yaml
   services:
     learning:
       ports:
         - "127.0.0.1:50051:50051"
   ```

3. **构建、准备 Policy SDK 并启动容器**：
   `learning` 容器以只读方式将宿主机 `build/generated/policy-sdk/` 挂载至 `/opt/policy-sdk`。可在宿主机先生成 Policy SDK，再启动容器：

   ```bash
   ./env.sh build learning
   docker compose -f learning/lerobot/docker/docker-compose.yml run --rm --no-deps \
     -v "$PWD/build:/build" learning \
     python3 /shared/common/tools/scripts/generate_policy_sdk.py --output /build/generated/policy-sdk
   ./env.sh up learning
   ```

4. **在容器内运行转换、训练与策略服务**：

   ```bash
   ./env.sh exec learning scripts/convert.sh --robot marvin_sim --task stack
   ./env.sh exec learning scripts/train.sh --robot marvin_sim --task stack --mode full
   ./env.sh exec learning scripts/server_policy.sh --robot marvin_sim --task stack
   ```

---

## 第二步：本机通过 SSH 隧道联调（通用）

远端环境就绪后，按以下顺序完成数据同步与跨机推理（示例中的 `user@remote:/path/to/sim_dual_arm/` 请替换为实际 SSH 地址与远端仓库路径）：

1. **同步数据或模型（在本机仓库根目录执行）**：

   ```bash
   # 将本机采集的原始数据传给远端用于转换和训练
   rsync -aR var/marvin_sim/stack/datasets/raw/ user@remote:/path/to/sim_dual_arm/

   # 或直接把本机已有模型传给远端用于推理
   rsync -aR var/marvin_sim/stack/models/act_YYYYMMDD_HHMMSS/ user@remote:/path/to/sim_dual_arm/
   ```

2. **远端启动策略服务并保持运行**：按方式 A 或方式 B 启动 `server_policy.sh`（默认监听 `50051`）。
3. **本机建立 SSH 本地转发（在本机宿主机新终端保持运行）**：
   本机的 `runtime` 在容器内，通过 `host.docker.internal` 访问宿主机网络，因此把隧道绑定到 `runtime` 容器解析出的 Docker 网关 IP（示例使用本地 `15051` 端口）：

   ```bash
   ./env.sh up runtime
   GATEWAY_IP=$(./env.sh exec runtime getent ahostsv4 host.docker.internal | awk 'NR == 1 { print $1 }')
   ssh -NT -o ExitOnForwardFailure=yes -L "${GATEWAY_IP}:15051:127.0.0.1:50051" user@remote
   ```

4. **本机启动仿真与推理**：
   在本机按[推理仿真后端启动步骤](../../README.md#34-直接启动推理仿真后端)启动仿真后端，并在推理命令中追加 `--policy-endpoint host.docker.internal:15051`：

   ```bash
   # 单局或交互推理（在 runtime 容器内执行）
   ./env.sh exec runtime scripts/client_policy.sh --robot marvin_sim --task stack \
     --reset-seed 20000 --policy-endpoint host.docker.internal:15051
   ./env.sh exec runtime scripts/client_interactive.sh --robot marvin_sim --task stack \
     --seed 20000 --policy-endpoint host.docker.internal:15051

   # 批量并行推理（在本机宿主机仓库根目录执行）
   ./inference_parallel.sh --robot marvin_sim --task stack \
     --count 10 --workers auto --policy-endpoint host.docker.internal:15051
   ```

---

## 目录结构

- [`src/dual_arm_learning/`](src/dual_arm_learning/README.md)：`dual_arm_learning` 核心 Python 包。
- [`configs/`](configs/README.md)：容器环境配置 `container.env`。
- [`docker/`](docker/README.md)：`Dockerfile`、`docker-compose.yml` 与源码声明。
- [`scripts/`](scripts/README.md)：容器内单职责入口脚本（`convert.sh`、`train.sh`、`server_policy.sh`、`test.sh`）。

## 常用脚本

在 `learning` 容器内执行，使用 `--robot <robot> --task <task>` 或 `--config <recipe>` 选择配方。训练模式、步数上限、模型路径和设备等常用设置有专用参数，其余参数在对应配方中修改。

- `scripts/convert.sh`：将合格采集数据转换为训练/验证 LeRobot 数据集（输出到 `var/<robot>/<task>/datasets/lerobot/`）。
- `scripts/train.sh`：训练 ACT 模型，直接写出 `var/<robot>/<task>/models/act_<时间戳>/model.safetensors` 与 `training_args.yaml`；训练日志、指标和终态摘要输出到 `var/<robot>/<task>/logs/training/act_<时间戳>/`。
- `scripts/server_policy.sh`：启动 gRPC Policy 推理服务（默认端口 `50051`），默认读取 `var/<robot>/<task>/models/` 下最新的 `act_*` 模型。
- `scripts/test.sh`：运行容器内 `pytest` 测试。

```bash
scripts/convert.sh --robot marvin_sim --task stack
scripts/train.sh --robot marvin_sim --task stack --mode full
scripts/server_policy.sh --robot marvin_sim --task stack
```

> `convert.sh` 和 `train.sh` 启动时会打印配置文件与有效参数，终端使用原地刷新的单行进度条，详细日志写入启动时提示的日志文件。
