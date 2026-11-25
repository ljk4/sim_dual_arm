#!/usr/bin/env bash
# Sole common container helper.  Source it from business, SDK/test scripts or an
# interactive shell; it rejects host execution, then sets PATH/PYTHONPATH/ROS
# only.  It never executes a command, validates an SDK, imports project modules
# or starts a task.  Sourcing it again in `docker compose exec` gives the same
# environment as the manually invoked business script.
#
# ROS setup files are not nounset-safe; temporarily relax nounset while
# sourcing and restore the caller's setting.

if [[ "${PROJECT_CONTAINER:-}" == "0" || ( "${PROJECT_CONTAINER:-}" != "1" && ! -e /.dockerenv ) ]]; then
  cat >&2 <<'EOF'
错误 (must run inside a project container): 该脚本只能在对应项目容器内执行。
请先在宿主机项目根目录启动或进入容器，或通过 ./env.sh exec 转发：
  ./env.sh <env>                  # 启动并进入容器 (runtime | mujoco | lerobot | tianji)
  ./env.sh exec <env> scripts/... # 在宿主机直接向容器转发执行一条命令
EOF
  exit 2
fi
export PROJECT_CONTAINER=1

_setup_project_root="${PROJECT_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"
_setup_build_root="${BUILD_ROOT:-/build}"
_setup_use_ros="${PROJECT_USE_ROS:-0}"
_setup_ros_root="/opt/ros/${ROS_DISTRO:-jazzy}"
_setup_ros_sdk="${ROS_SDK_ROOT:-/opt/ros-dev}"

_setup_nounset=0
if [[ "$-" == *u* ]]; then
  _setup_nounset=1
fi
set +u
if [[ "$_setup_use_ros" == "1" && -f "$_setup_ros_root/setup.bash" ]]; then
  source "$_setup_ros_root/setup.bash"
fi
if [[ "$_setup_use_ros" == "1" && -f "$_setup_ros_sdk/install/setup.bash" ]]; then
  source "$_setup_ros_sdk/install/setup.bash"
fi
if [[ "$_setup_nounset" == "1" ]]; then
  set -u
fi

export PROJECT_ROOT="$_setup_project_root"
export BUILD_ROOT="$_setup_build_root"
export PYTHONDONTWRITEBYTECODE=1
export PYTHONUNBUFFERED=1

# Implementation paths come from its Compose file, not from a global role
# switch.  Missing directories are skipped: a fresh checkout may not have an
# SDK yet, and idle/shell must still start before the first SDK build.
IFS=: read -r -a python_paths <<< "${PROJECT_SOURCE_PATHS:-}${PYTHONPATH:+:$PYTHONPATH}"

joined=""
declare -A _setup_seen_paths=()
for path in "${python_paths[@]}"; do
  if [[ -n "$path" && -d "$path" && -z "${_setup_seen_paths[$path]:-}" ]]; then
    _setup_seen_paths["$path"]=1
    if [[ -n "$joined" ]]; then
      joined+=":"
    fi
    joined+="$path"
  fi
done
if [[ -n "$joined" ]]; then
  export PYTHONPATH="$joined"
fi

unset _setup_project_root _setup_build_root _setup_use_ros _setup_ros_root _setup_ros_sdk
unset _setup_nounset _setup_seen_paths joined python_paths path
