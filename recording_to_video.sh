#!/usr/bin/env bash
# Host-only entry: convert recorded rosbag2/MCAP episodes into visualization MP4 videos.
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${PROJECT_CONTAINER:-0}" == "1" ]]; then
  printf '错误: 视频转换入口需要在宿主机执行（需调用宿主机 ffmpeg 与 runtime 容器解包）。\n' >&2
  exit 2
fi
export PYTHONPATH="$ROOT_DIR/shared/common/tools${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -m host.recording_to_video "$@"
