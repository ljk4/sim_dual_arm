#!/usr/bin/env bash
# Bootstrap the ROS-free ACT learning environment on Google Colab or any Linux GPU host.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
DEFAULT_REPO_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd -P)"

RESNET18_URL="https://download.pytorch.org/models/resnet18-f37072fd.pth"
RESNET18_SHA256="f37072fd47e89c5e827621c5baffa7500819f7896bbacec160b1a16c560e07ec"

REPO_ROOT="$DEFAULT_REPO_ROOT"
CACHE_DIR=""
TORCH_INDEX_URL=""

usage() {
  cat <<'EOF'
Usage: setup_colab_learning.sh [OPTIONS]

Bootstrap third-party assets, uv Python environment, PyTorch, LeRobot,
Policy SDK, and .env.learning inside the repository root without Docker.

Options:
  --repo-root <dir>        Repository root directory (default: auto-detected from script path)
  --cache-dir <dir>        Persistent cache directory for LeRobot archive and ResNet18 weights
                           (e.g. /content/drive/MyDrive/sim_dual_arm/_cache)
  --torch-index-url <url>  Explicit PyTorch wheel index URL
                           (default: auto-detect cu128 / cu126 / cpu from nvidia-smi)
  -h, --help               Show this help message and exit
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --repo-root)
      [[ $# -ge 2 ]] || { echo "error: --repo-root requires a value" >&2; exit 2; }
      REPO_ROOT="$2"
      shift 2
      ;;
    --cache-dir)
      [[ $# -ge 2 ]] || { echo "error: --cache-dir requires a value" >&2; exit 2; }
      CACHE_DIR="$2"
      shift 2
      ;;
    --torch-index-url)
      [[ $# -ge 2 ]] || { echo "error: --torch-index-url requires a value" >&2; exit 2; }
      TORCH_INDEX_URL="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "error: unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

for cmd in python3 curl tar sha256sum; do
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo "error: required command not found: $cmd" >&2
    exit 1
  fi
done

if [[ ! -d "$REPO_ROOT" ]]; then
  echo "error: repository root does not exist: $REPO_ROOT" >&2
  exit 1
fi
REPO_ROOT="$(cd "$REPO_ROOT" && pwd -P)"

SOURCE_MANIFEST="$REPO_ROOT/learning/lerobot/docker/learning-source.json"
if [[ ! -f "$SOURCE_MANIFEST" ]]; then
  echo "error: missing learning source manifest: $SOURCE_MANIFEST" >&2
  exit 1
fi

if [[ -n "$CACHE_DIR" ]]; then
  mkdir -p "$CACHE_DIR"
  CACHE_DIR="$(cd "$CACHE_DIR" && pwd -P)"
fi

# 1. Read SSOT metadata from learning-source.json
read -r LEROBOT_SOURCE LEROBOT_REV PYTHON_VERSION < <(
  python3 - "$SOURCE_MANIFEST" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as fh:
    doc = json.load(fh)
for key in ("source", "revision", "python"):
    if not isinstance(doc.get(key), str) or not doc[key]:
        raise SystemExit(f"missing required key {key!r} in {sys.argv[1]}")
print(
    doc["source"],
    doc["revision"],
    doc["python"],
)
PY
)

LEROBOT_DIR="$REPO_ROOT/$LEROBOT_SOURCE"
RESNET_PATH="$REPO_ROOT/third_party/weights/resnet18-f37072fd.pth"

# Detect GPU and driver CUDA version
GPU_NAME="none"
DRIVER_CUDA="none"
if command -v nvidia-smi >/dev/null 2>&1 && SMI_OUT="$(nvidia-smi 2>/dev/null)"; then
  DETECTED_CUDA="$(printf '%s\n' "$SMI_OUT" | sed -nE 's/.*CUDA Version:[[:space:]]*([0-9]+\.[0-9]+).*/\1/p' | head -n1 || true)"
  DETECTED_GPU="$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -n1 | xargs || true)"
  if [[ -n "$DETECTED_GPU" ]]; then
    GPU_NAME="$DETECTED_GPU"
  fi
  if [[ -n "$DETECTED_CUDA" ]]; then
    DRIVER_CUDA="$DETECTED_CUDA"
  fi
fi

if [[ -z "$TORCH_INDEX_URL" ]]; then
  if [[ "$DRIVER_CUDA" != "none" ]]; then
    TORCH_INDEX_URL="$(
      python3 - "$DRIVER_CUDA" <<'PY'
import sys
parts = tuple(int(p) for p in sys.argv[1].split(".")[:2])
print("https://download.pytorch.org/whl/cu128" if parts >= (12, 8) else "https://download.pytorch.org/whl/cu126")
PY
    )"
  else
    TORCH_INDEX_URL="https://download.pytorch.org/whl/cpu"
  fi
fi

echo "=== Colab / Linux 学习端环境配置 ==="
echo "REPO_ROOT:        $REPO_ROOT"
echo "SOURCE_MANIFEST:  $SOURCE_MANIFEST"
echo "LEROBOT_DIR:      $LEROBOT_DIR"
echo "LEROBOT_REV:      $LEROBOT_REV"
echo "PYTHON_VERSION:   $PYTHON_VERSION"
echo "RESNET_PATH:      $RESNET_PATH"
echo "CACHE_DIR:        ${CACHE_DIR:-<disabled>}"
echo "GPU_NAME:         $GPU_NAME"
echo "DRIVER_CUDA:      $DRIVER_CUDA"
echo "TORCH_INDEX_URL:  $TORCH_INDEX_URL"
echo "====================================="

TMP_DIR="$(mktemp -d)"
trap 'rm -rf "$TMP_DIR"' EXIT

verify_sha256() {
  local file_path="$1"
  local expected_sha="$2"
  if [[ -z "$expected_sha" ]]; then
    return 0
  fi
  printf '%s  %s\n' "$expected_sha" "$file_path" | sha256sum -c - >/dev/null 2>&1
}

atomic_cache_copy() {
  local src="$1"
  local dst="$2"
  local tmp_dst="${dst}.tmp.$$"
  cp -f "$src" "$tmp_dst"
  mv -f "$tmp_dst" "$dst"
}

# 2. Prepare third_party/ assets (idempotent, non-destructive)
mkdir -p \
  "$REPO_ROOT/output/build" \
  "$REPO_ROOT/output/_system" \
  "$REPO_ROOT/third_party/src" \
  "$REPO_ROOT/third_party/weights"

if [[ -f "$LEROBOT_DIR/pyproject.toml" ]]; then
  echo "[1/5] LeRobot 源码已就绪，跳过下载: $LEROBOT_DIR"
else
  echo "[1/5] 准备固定版本 LeRobot 源码 ($LEROBOT_REV)..."
  TARBALL_NAME="lerobot-${LEROBOT_REV}.tar.gz"
  TARBALL_URL="https://github.com/huggingface/lerobot/archive/${LEROBOT_REV}.tar.gz"
  LOCAL_TARBALL="$TMP_DIR/$TARBALL_NAME"
  USE_TARBALL=""

  if [[ -n "$CACHE_DIR" && -f "$CACHE_DIR/$TARBALL_NAME" ]]; then
    if tar -tzf "$CACHE_DIR/$TARBALL_NAME" >/dev/null 2>&1; then
      echo "      命中持久化缓存: $CACHE_DIR/$TARBALL_NAME"
      USE_TARBALL="$CACHE_DIR/$TARBALL_NAME"
    else
      echo "      缓存源码包校验未通过，重新下载: $CACHE_DIR/$TARBALL_NAME"
    fi
  fi

  if [[ -z "$USE_TARBALL" ]]; then
    curl -fL --retry 3 --retry-delay 2 -o "$LOCAL_TARBALL" "$TARBALL_URL"
    if ! tar -tzf "$LOCAL_TARBALL" >/dev/null 2>&1; then
      echo "error: downloaded LeRobot archive is unreadable or corrupted: $LOCAL_TARBALL" >&2
      exit 1
    fi
    if [[ -n "$CACHE_DIR" ]]; then
      atomic_cache_copy "$LOCAL_TARBALL" "$CACHE_DIR/$TARBALL_NAME"
    fi
    USE_TARBALL="$LOCAL_TARBALL"
  fi

  EXTRACT_STAGE="$TMP_DIR/lerobot-stage"
  mkdir -p "$EXTRACT_STAGE"
  tar -xz --strip-components=1 -f "$USE_TARBALL" -C "$EXTRACT_STAGE"
  if [[ ! -f "$EXTRACT_STAGE/pyproject.toml" ]]; then
    echo "error: extracted LeRobot archive missing pyproject.toml: $EXTRACT_STAGE" >&2
    exit 1
  fi
  rm -rf "$LEROBOT_DIR"
  mkdir -p "$(dirname "$LEROBOT_DIR")"
  mv "$EXTRACT_STAGE" "$LEROBOT_DIR"
fi

if [[ -f "$RESNET_PATH" ]] && verify_sha256 "$RESNET_PATH" "$RESNET18_SHA256"; then
  echo "[2/5] ResNet18 离线权重已就绪且校验通过: $RESNET_PATH"
else
  echo "[2/5] 准备并校验 ResNet18 离线权重..."
  CACHED_RESNET="${CACHE_DIR:+$CACHE_DIR/resnet18-f37072fd.pth}"
  if [[ -n "$CACHED_RESNET" && -f "$CACHED_RESNET" ]] && verify_sha256 "$CACHED_RESNET" "$RESNET18_SHA256"; then
    echo "      命中持久化缓存: $CACHED_RESNET"
    atomic_cache_copy "$CACHED_RESNET" "$RESNET_PATH"
  else
    TMP_RESNET="$TMP_DIR/resnet18-f37072fd.pth"
    curl -fL --retry 3 --retry-delay 2 -o "$TMP_RESNET" "$RESNET18_URL"
    if ! verify_sha256 "$TMP_RESNET" "$RESNET18_SHA256"; then
      echo "error: downloaded ResNet18 weights failed sha256 verification" >&2
      exit 1
    fi
    mv -f "$TMP_RESNET" "$RESNET_PATH"
    if [[ -n "$CACHE_DIR" ]]; then
      atomic_cache_copy "$RESNET_PATH" "$CACHE_DIR/resnet18-f37072fd.pth"
    fi
  fi
fi

# 3. Install uv, Python, .venv, and learning dependencies
echo "[3/5] 配置 uv、Python $PYTHON_VERSION 虚拟环境与依赖..."
export UV_INSTALL_DIR="$REPO_ROOT/output/_system/uv/bin"
export UV_PYTHON_INSTALL_DIR="$REPO_ROOT/output/_system/uv/python"
export UV_CACHE_DIR="$REPO_ROOT/output/_system/uv/cache"
export PATH="$UV_INSTALL_DIR:$PATH"
mkdir -p "$UV_INSTALL_DIR" "$UV_PYTHON_INSTALL_DIR" "$UV_CACHE_DIR"

if [[ ! -x "$UV_INSTALL_DIR/uv" ]] && ! command -v uv >/dev/null 2>&1; then
  curl -LsSf https://astral.sh/uv/install.sh | env UV_NO_MODIFY_PATH=1 UV_INSTALL_DIR="$UV_INSTALL_DIR" sh
fi

uv python install "$PYTHON_VERSION"

VENV_DIR="$REPO_ROOT/.venv"
VENV_PYTHON="$VENV_DIR/bin/python"
if [[ ! -x "$VENV_PYTHON" ]]; then
  uv venv --python "$PYTHON_VERSION" "$VENV_DIR"
fi

# Install PyTorch first from the selected CUDA/CPU index (no --extra-index-url).
uv pip install --python "$VENV_PYTHON" \
  --index-url "$TORCH_INDEX_URL" \
  "torch>=2.7,<2.12.0" "torchvision>=0.22.0,<0.27.0"

# Install pinned LeRobot (base package only; project uses video_backend=pyav without torchcodec)
# and the runtime/conversion/serving dependencies required by dual_arm_learning.
uv pip install --python "$VENV_PYTHON" \
  -e "$LEROBOT_DIR" \
  "datasets>=4.8.0,<5.0.0" \
  "pandas>=2.0.0,<3.0.0" \
  "pyarrow>=21.0.0,<30.0.0" \
  "av>=15.0.0,<16.0.0" \
  "jsonlines>=4.0.0,<5.0.0" \
  pydantic pyyaml jsonschema pillow matplotlib safetensors grpcio grpcio-tools

# 4. Generate Policy SDK and write .env.learning
echo "[4/5] 生成 Policy gRPC SDK 并写入 .env.learning..."
SDK_OUTPUT="$REPO_ROOT/output/build/generated/policy-sdk"
"$VENV_PYTHON" "$REPO_ROOT/shared/common/tools/scripts/generate_policy_sdk.py" --output "$SDK_OUTPUT"

HF_CACHE_DIR="$REPO_ROOT/output/_system/hf-cache"
mkdir -p "$HF_CACHE_DIR"

ENV_FILE="$REPO_ROOT/.env.learning"
cat > "$ENV_FILE" <<EOF
source "$VENV_DIR/bin/activate"
export PROJECT_CONTAINER=1
export PROJECT_ROOT="$REPO_ROOT"
export SHARED_ROOT="$REPO_ROOT/shared"
export DATA_ROOT="$REPO_ROOT/output"
export BUILD_ROOT="$REPO_ROOT/output/build"
export THIRD_PARTY_ROOT="$REPO_ROOT/third_party"
export POLICY_SDK_ROOT="$SDK_OUTPUT"
export RESNET18_WEIGHTS="$RESNET_PATH"
export HF_HOME="$HF_CACHE_DIR"
export HF_HUB_OFFLINE=1
export HF_DATASETS_OFFLINE=1
export TRANSFORMERS_OFFLINE=1
export PATH="$UV_INSTALL_DIR:\$PATH"
export PYTHONPATH="$REPO_ROOT/learning/lerobot/src:$REPO_ROOT/shared/common/python/run_logging/src:$REPO_ROOT/shared/common/python/contracts/src:$LEROBOT_DIR/src:$SDK_OUTPUT"
EOF

# 5. Self-check
echo "[5/5] 执行环境与导入自检..."
(
  set +u
  source "$ENV_FILE"
  set -u
  python3 "$REPO_ROOT/shared/common/tools/scripts/policy_sdk.py" check \
    --project-root "$REPO_ROOT" \
    --sdk-root "$SDK_OUTPUT"
  python3 - "$LEROBOT_DIR" "$RESNET_PATH" "$DRIVER_CUDA" <<'PY'
from pathlib import Path
import sys

import av
import datasets
import dual_arm_learning
import dual_arm_policy
import grpc
import jsonschema
import lerobot
import pydantic
import pyarrow
import safetensors
import torch
import torchvision
import yaml

lerobot_dir = Path(sys.argv[1]).resolve()
resnet_path = Path(sys.argv[2]).resolve()
driver_cuda = sys.argv[3]

lerobot_file = Path(lerobot.__file__).resolve()
if not lerobot_file.is_relative_to(lerobot_dir):
    raise SystemExit(f"lerobot imported from unexpected path: {lerobot_file} (expected under {lerobot_dir})")

if not resnet_path.is_file():
    raise SystemExit(f"ResNet18 weights missing: {resnet_path}")
with resnet_path.open("rb") as fh:
    fh.read(16)

cuda_ok = torch.cuda.is_available()
if driver_cuda != "none" and not cuda_ok:
    raise SystemExit(f"GPU driver detected (CUDA {driver_cuda}) but torch.cuda.is_available() is False")

gpu_name = torch.cuda.get_device_name(0) if cuda_ok else "cpu"
bf16_ok = bool(cuda_ok and torch.cuda.get_device_capability(0)[0] >= 8)
print(
    f"Self-check OK | torch={torch.__version__} | cuda={torch.version.cuda} | "
    f"gpu={gpu_name} | bf16={bf16_ok} | lerobot={lerobot_file}"
)
PY
)

echo "学习端环境准备完成。后续运行前请先执行: source \"$ENV_FILE\""
