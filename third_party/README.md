# 配置第三方源码与权重

初次使用学习端前，在**仓库根目录**执行以下两步。下载产物位于 `third_party/src/` 与 `third_party/weights/`，均已加入 `.gitignore`，可长期保留复用。

## 1. 准备 LeRobot 源码

目标目录为 `third_party/src/lerobot-act-v0.6.1/`，版本锁定于 [`learning/lerobot/docker/learning-source.json`](../learning/lerobot/docker/learning-source.json) 中的提交 `7e241bd630a3719a56157a497ce5d08f244784f1`。在仓库根目录执行：

```bash
mkdir -p third_party/src/lerobot-act-v0.6.1
curl -fL https://github.com/huggingface/lerobot/archive/7e241bd630a3719a56157a497ce5d08f244784f1.tar.gz \
  | tar -xz --strip-components=1 -C third_party/src/lerobot-act-v0.6.1
```

> 如需重新下载，请先清空或移走旧的 `third_party/src/lerobot-act-v0.6.1/` 目录。

## 2. 下载并校验 ResNet18 权重

ACT 视觉骨干使用 TorchVision 官方 `ResNet18_Weights.IMAGENET1K_V1` 权重（`third_party/weights/resnet18-f37072fd.pth`）。下载并执行 SHA-256 校验（输出 `OK` 即通过）：

```bash
mkdir -p third_party/weights
curl -fL -o third_party/weights/resnet18-f37072fd.pth \
  https://download.pytorch.org/models/resnet18-f37072fd.pth
echo "f37072fd47e89c5e827621c5baffa7500819f7896bbacec160b1a16c560e07ec  third_party/weights/resnet18-f37072fd.pth" | sha256sum -c -
```

## 3. 在学习容器中验证（可选）

使用 Docker 运行学习端时，`third_party/` 会只读挂载为 `/third_party/`。构建并进入容器后可确认源码与权重已就绪：

```bash
./env.sh build learning && ./env.sh enter learning
```

容器内执行（第一条应打印 `/third_party/src/lerobot-act-v0.6.1/src/lerobot/__init__.py`）：

```bash
source "$PROJECT_SETUP"
python3 -c 'import lerobot; print(lerobot.__file__)'
test -r /third_party/weights/resnet18-f37072fd.pth
```

无 Docker、直接在仓库目录内用 `uv` 运行学习端时，请按 [`learning/lerobot/README.md`](../learning/lerobot/README.md) 配置目录内环境。

## 其他外部资产位置

- **Marvin 仿真模型**：维护于 [`simulation/mujoco/assets/marvin/`](../simulation/mujoco/assets/marvin/README.md)，随仿真目录只读挂载至容器 `/app/assets/`。

目录导航：[上级目录](../README.md)
