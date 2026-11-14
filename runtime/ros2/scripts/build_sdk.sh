#!/usr/bin/env bash
# Build both the ROS 2 interface workspace and the Policy Protobuf/gRPC SDK
# inside the runtime container.
set -euo pipefail

APP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SHARED_ROOT="${SHARED_ROOT:-$APP_ROOT/../../shared}"
source "${PROJECT_SETUP:-$SHARED_ROOT/common/tools/docker/setup.sh}"

usage() {
  cat <<'EOF'
usage: scripts/build_sdk.sh [options]

Build both the ROS 2 interface workspace (/build/ros-dev) and the Policy
Protobuf/gRPC SDK (/build/generated/policy-sdk) inside the runtime container.

options:
  --ros-only      build only the ROS 2 interface workspace
  --policy-only   build only the Policy Protobuf/gRPC SDK
  -h, --help      show this help message
EOF
}

build_ros=true
build_policy=true

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ros-only)
      build_policy=false
      shift
      ;;
    --policy-only)
      build_ros=false
      shift
      ;;
    -h|--help|help)
      usage
      exit 0
      ;;
    *)
      printf 'scripts/build_sdk.sh: unknown argument %q\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ "$build_ros" == "true" ]]; then
  bash "$SHARED_ROOT/common/tools/scripts/build_ros_sdk.sh" --shared-root "$SHARED_ROOT"
fi

if [[ "$build_policy" == "true" ]]; then
  bash "$SHARED_ROOT/common/tools/scripts/build_policy_sdk.sh"
fi
