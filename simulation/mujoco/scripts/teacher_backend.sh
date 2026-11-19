#!/usr/bin/env bash
# Start the MuJoCo teacher backend for a robot/task inside the simulation container,
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
  --robot <id>          robot id (required)
  --task <id>           task id (required)
  --seed <int>          initial simulation seed (default: 0)
  --run-id <id>         backend run id prefix (default: teacher-backend_<YYYYMMDD_HHMMSS>)
  --shared-root <path>  override shared scenarios root
  --run-root <path>     override backend run root
  --set PATH=VALUE      override one existing configuration path (repeatable)
  -h, --help            show this help message
EOF
}

robot=""
task=""
seed="0"
run_id=""
shared_root=""
run_root=""
overrides=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --robot)
      robot="${2:?missing value for --robot}"
      shift 2
      ;;
    --robot=*)
      robot="${1#*=}"
      shift
      ;;
    --task)
      task="${2:?missing value for --task}"
      shift 2
      ;;
    --task=*)
      task="${1#*=}"
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

if [[ -z "$robot" || -z "$task" ]]; then
  printf 'scripts/teacher_backend.sh: --robot and --task are required\n' >&2
  exit 2
fi

if [[ -z "$run_root" ]]; then
  run_root="$DATA_ROOT/$robot/$task/logs/backend"
fi

ts="$(date +%Y%m%d_%H%M%S)"
base="${run_id:-teacher-backend}"
if [[ ! "$base" =~ (_[0-9]{8}_[0-9]{6}|-[0-9]{8}T[0-9]{6}Z)$ ]]; then
  run_id="${base}_${ts}"
else
  run_id="$base"
fi
if [[ -e "$run_root/$run_id" || -e "$run_root/${run_id}-startup-parameters.yaml" ]]; then
  suffix=2
  while [[ -e "$run_root/${run_id}_${suffix}" || -e "$run_root/${run_id}_${suffix}-startup-parameters.yaml" ]]; do
    suffix=$((suffix + 1))
  done
  run_id="${run_id}_${suffix}"
fi

python3 "${SHARED_ROOT:-/shared}/common/tools/scripts/dev_workspace.py" check \
  --project-root / \
  --build-root "$ROS_SDK" 1>&2

python3 - "$robot" "$task" "$seed" "$run_id" "$run_root" "$shared_root" "${overrides[@]}" <<'PY'
import sys
from pathlib import Path
import yaml
from robot_backend_sim.common.config.scenario_config import resolve_scenario_backend

robot, task, seed_str, run_id, run_root, shared_root, *overrides = sys.argv[1:]
resolved = resolve_scenario_backend(
    robot=robot,
    task=task,
    role="teacher",
    shared_root=Path(shared_root) if shared_root else None,
    seed=int(seed_str),
    overrides=overrides,
)
run_dir = Path(run_root) / run_id
Path(run_root).mkdir(parents=True, exist_ok=True)
parameters_path = Path(run_root) / f"{run_id}-startup-parameters.yaml"
parameters_path.write_text(
    yaml.safe_dump(resolved.document, sort_keys=False, allow_unicode=True),
    encoding="utf-8",
)
print(f"方案: {robot}/{task} | 角色: teacher | seed: {seed_str}")
print(f"Binding: {resolved.selection['binding']['path']}")
print(f"后端配方: {resolved.selection['recipe']['path']}")
print(f"运行日志: {run_dir / 'log'}")
print(f"详细参数: {parameters_path}")
print("================================", flush=True)
PY

cmd=(
  python3 -m robot_backend_sim.common.ros.node
  --robot "$robot"
  --task "$task"
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
