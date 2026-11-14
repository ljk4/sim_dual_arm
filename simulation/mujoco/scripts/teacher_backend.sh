#!/usr/bin/env bash
# Start the MuJoCo teacher backend for a scenario inside the simulation container,
# printing the resolved config file paths and full effective parameters.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${PROJECT_SETUP:-$ROOT/../../shared/common/tools/docker/setup.sh}"

DATA_ROOT="${DATA_ROOT:-/data}"
ROS_SDK="${ROS_SDK_ROOT:-/opt/ros-dev}"

usage() {
  cat <<'EOF'
usage: scripts/teacher_backend.sh [options]

Start the MuJoCo teacher backend (--role teacher) inside the simulation
container and print the resolved configuration file paths and parameters.

options:
  --scenario <id>       scenario id (required)
  --seed <int>          initial simulation seed (default: 0)
  --run-id <id>         backend run id (default: teacher-backend)
  --shared-root <path>  override shared scenarios root
  --run-root <path>     override backend run root
  --set PATH=VALUE      override one existing configuration path (repeatable)
  -h, --help            show this help message
EOF
}

scenario=""
seed="0"
run_id="teacher-backend"
shared_root=""
run_root=""
overrides=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --scenario)
      scenario="${2:?missing value for --scenario}"
      shift 2
      ;;
    --scenario=*)
      scenario="${1#*=}"
      shift
      ;;
    --seed)
      seed="${2:?missing value for --seed}"
      shift 2
      ;;
    --seed=*)
      seed="${1#*=}"
      shift
      ;;
    --run-id)
      run_id="${2:?missing value for --run-id}"
      shift 2
      ;;
    --run-id=*)
      run_id="${1#*=}"
      shift
      ;;
    --shared-root)
      shared_root="${2:?missing value for --shared-root}"
      shift 2
      ;;
    --shared-root=*)
      shared_root="${1#*=}"
      shift
      ;;
    --run-root)
      run_root="${2:?missing value for --run-root}"
      shift 2
      ;;
    --run-root=*)
      run_root="${1#*=}"
      shift
      ;;
    --set)
      overrides+=("${2:?missing value for --set}")
      shift 2
      ;;
    --set=*)
      overrides+=("${1#*=}")
      shift
      ;;
    -h|--help|help)
      usage
      exit 0
      ;;
    *)
      printf 'scripts/teacher_backend.sh: unknown argument %q\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -z "$scenario" ]]; then
  printf 'scripts/teacher_backend.sh: --scenario is required\n' >&2
  exit 2
fi

if [[ -z "$run_root" ]]; then
  run_root="$DATA_ROOT/$scenario/logs/backend"
fi

python3 "${SHARED_ROOT:-/shared}/common/tools/scripts/dev_workspace.py" check \
  --project-root / \
  --build-root "$ROS_SDK" 1>&2

python3 - "$scenario" "$seed" "$run_id" "$run_root" "$shared_root" "${overrides[@]}" <<'PY'
import sys
from pathlib import Path
import yaml
from robot_backend_sim.config.scenario_config import resolve_scenario_backend

scenario, seed_str, run_id, run_root, shared_root, *overrides = sys.argv[1:]
resolved = resolve_scenario_backend(
    scenario=scenario,
    role="teacher",
    shared_root=Path(shared_root) if shared_root else None,
    seed=int(seed_str),
    overrides=overrides,
)
run_dir = Path(run_root) / run_id
print("=== Teacher Backend 配置摘要 ===")
print(f"方案 (scenario):   {scenario}")
print(f"角色 (role):       teacher")
print(f"Binding 文件:      {resolved.selection['binding']['path']}")
print(f"后端配方文件:      {resolved.selection['recipe']['path']}")
print(f"生效快照路径:      {run_dir / 'backend.yaml'}")
print(f"运行日志目录:      {run_dir / 'log'}")
print("--- 详细参数 ---")
print(yaml.safe_dump(resolved.document, sort_keys=False, allow_unicode=True), end="")
print("================================", flush=True)
PY

cmd=(
  python3 -m robot_backend_sim.ros.node
  --scenario "$scenario"
  --role teacher
  --seed "$seed"
  --run-id "$run_id"
  --run-root "$run_root"
)
if [[ -n "$shared_root" ]]; then
  cmd+=(--shared-root "$shared_root")
fi
for item in "${overrides[@]}"; do
  cmd+=(--set "$item")
done

exec "${cmd[@]}"
