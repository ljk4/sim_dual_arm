#!/usr/bin/env bash
# Host-only Docker orchestration; business inference stays inside runtime.
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${PROJECT_CONTAINER:-0}" == "1" ]]; then
  printf '并行推理入口需要在宿主机执行，用于管理独立 simulation 容器。\n' >&2
  exit 2
fi
export PYTHONPATH="$ROOT_DIR/shared/common/tools${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -m host.inference_parallel "$@"
