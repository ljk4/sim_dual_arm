# LeRobot ACT 学习与策略服务

提供无 ROS 依赖的 LeRobot 数据集转换、ACT 训练与 gRPC Policy 推理服务。

## 在云服务器上用 uv 搭建学习端环境

学习端只需要 Python 3.12、NVIDIA 驱动/CUDA 与仓库源码，不需要 ROS 或 Docker。以下步骤在一台 Ubuntu 云服务器（建议 ≥16 GB 显存）上执行：

```bash
# 1. 安装 uv（独立安装到 ~/.local/bin，不依赖系统 Python）
curl -LsSf https://astral.sh/uv/install.sh | sh
export PATH="$HOME/.local/bin:$PATH"
uv --version

# 2. 取得仓库与固定版本 LeRobot 源码
git clone https://github.com/ljk4/sim_dual_arm.git
cd sim_dual_arm
# third_party/src 被 .gitignore 忽略；按 third_party/README.md 恢复
# lerobot-act-v0.6.1 到 third_party/src/lerobot-act-v0.6.1，
# 并把 resnet18-f37072fd.pth 放到 third_party/weights/。

# 3. 创建独立环境并安装依赖（先按显卡/CUDA 版本选择 PyTorch 轮子）
uv python install 3.12
uv venv --python 3.12 .venv
source .venv/bin/activate
uv pip install torch torchvision --index-url https://download.pytorch.org/whl/cu124
uv pip install -e third_party/src/lerobot-act-v0.6.1
uv pip install pydantic pyyaml jsonschema pillow pyarrow safetensors grpcio grpcio-tools av
```

在 Runtime 容器运行 `scripts/build_sdk.sh --policy-only` 生成 Policy gRPC SDK。把 `build/generated/policy-sdk/` 整个目录复制到云服务器仓库的同名路径，保留 `dual_arm_policy` 包和 `source.sha256`。训练使用本地权重 `third_party/weights/resnet18-f37072fd.pth`。

云服务器上没有项目容器，脚本内的 `setup.sh` 需要一个显式环境。把下面内容写入 `~/.bashrc`（路径按实际仓库位置调整）：

```bash
export PROJECT_CONTAINER=1
export PROJECT_ROOT="$HOME/sim_dual_arm"
export SHARED_ROOT="$PROJECT_ROOT/shared"
export DATA_ROOT="$PROJECT_ROOT/var"
export HF_HOME="$PROJECT_ROOT/var/_system/hf-cache"
export HF_HUB_OFFLINE=1 HF_DATASETS_OFFLINE=1 TRANSFORMERS_OFFLINE=1
export PYTHONPATH="$PROJECT_ROOT/learning/lerobot/src:$PROJECT_ROOT/shared/common/python/run_logging/src:$PROJECT_ROOT/shared/common/python/contracts/src:$PROJECT_ROOT/third_party/src/lerobot-act-v0.6.1/src:$PROJECT_ROOT/build/generated/policy-sdk"
```

之后在仓库根目录直接运行：

```bash
bash learning/lerobot/scripts/convert.sh --robot marvin_sim --task stack
bash learning/lerobot/scripts/train.sh --robot marvin_sim --task stack --mode full
bash learning/lerobot/scripts/server_policy.sh --robot marvin_sim --task stack
```

`server_policy.sh` 默认监听 `0.0.0.0:50051`。云服务器安全组只需放行 SSH；学习端与运行端之间通过 SSH 隧道通信（见下节），不要直接对公网开放 50051。

## 推荐拓扑：云服务器跑学习端，本机跑运行端与仿真端

```text
本机: runtime (ROS 2) ──gRPC──▶ 127.0.0.1:50051 ──SSH 隧道──▶ 云服务器:50051 (learning)
本机: simulation (MuJoCo) ◀──ROS 2── runtime
```

1. **同步数据**：在本机采集完成后，把训练输入同步到云服务器（模型与数据集都按 `var/<robot>/<task>/` 结构存放）：

   ```bash
   rsync -av --delete var/marvin_sim/stack/datasets/ \
     user@cloud:/home/user/sim_dual_arm/var/marvin_sim/stack/datasets/
   ```

2. **云端转换与训练**：在云服务器执行上面的 `convert.sh` 与 `train.sh`；训练产物写入 `var/<robot>/<task>/models/act_<时间戳>/`。

3. **云端启动策略服务**（保持运行）：

   ```bash
   bash learning/lerobot/scripts/server_policy.sh --robot marvin_sim --task stack
   ```

4. **本机建立 SSH 本地转发**（保持运行，`50051` 为云端服务端口）：

   ```bash
   ssh -N -L 127.0.0.1:50051:127.0.0.1:50051 user@cloud
   ```

5. **本机运行端指向隧道**：运行端容器通过 `host.docker.internal` 访问宿主机端口（Compose 已声明 `extra_hosts`）。启动推理脚本时追加：

   ```bash
   scripts/client_policy.sh --robot marvin_sim --task stack \
     --reset-seed 20000 --policy-endpoint host.docker.internal:50051
   scripts/client_interactive.sh --robot marvin_sim --task stack \
     --seed 20000 --policy-endpoint host.docker.internal:50051
   ```

   若策略服务改为在云服务器的 Docker 容器中运行，用 `docker compose` 的 `ports: ["50051:50051"]`（只绑定 `127.0.0.1`）暴露服务，再按第 4 步建立隧道即可。

6. **批量并行推理**。在本机项目根目录执行 `./inference_parallel.sh --robot marvin_sim --task stack --count 10 --workers auto --policy-endpoint host.docker.internal:50051`。各仿真 worker 独立运行，复用同一个云端策略服务。

> `ssh` 建议使用密钥登录与 `~/.ssh/config` 别名；连接中断会同时断开策略服务，长时批量评测请使用 `tmux`/`systemd` 保持隧道。

## 目录结构

- [`src/dual_arm_learning/`](src/dual_arm_learning/README.md)：`dual_arm_learning` 核心 Python 包。
- [`configs/`](configs/README.md)：容器环境配置 `container.env` 与测试替身配置。
- [`docker/`](docker/README.md)：`Dockerfile`、`docker-compose.yml` 与源码声明。
- [`scripts/`](scripts/README.md)：容器内单职责入口脚本（`convert.sh`、`train.sh`、`policy.sh`、`test.sh`）。

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
