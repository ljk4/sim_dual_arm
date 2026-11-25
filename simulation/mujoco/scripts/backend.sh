#!/usr/bin/env bash
# Unified MuJoCo Robot API backend launcher (teacher, policy, or direct --config).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${PROJECT_SETUP:-$ROOT/../../shared/common/tools/docker/setup.sh}"

DATA_ROOT="${DATA_ROOT:-/data}"
ROS_SDK="${ROS_SDK_ROOT:-/opt/ros-dev}"

usage() {
  cat <<'EOF'
usage: scripts/backend.sh [options]

Start the MuJoCo Robot API backend inside the mujoco container.

options:
  --robot <id>          robot id under shared/scenarios/<robot>/<backend>/<task>/
  --backend <id>        backend id under shared/scenarios/<robot>/<backend>/<task>/
  --task <id>           task id under shared/scenarios/<robot>/<backend>/<task>/
  --role <role>         teacher | policy (scenario mode default: teacher)
  --seed <int>          initial simulation seed (default: random)
  --run-id <id>         backend run id prefix
  --run-root <path>     override backend run root
  --shared-root <path>  override shared scenarios root
  --realtime-speed <n>  simulation seconds per wall-clock second
  --namespace <ns>      override robot.namespace
  --no-visualization    disable MuJoCo state visualization topic
  --config <path>       direct backend YAML recipe (mutually exclusive with --robot/--backend/--task)
  -h, --help            show this help message
EOF
}

robot=""
backend=""
task=""
role=""
seed=""
run_id=""
run_root=""
shared_root=""
realtime_speed=""
namespace=""
no_visualization=false
direct_config=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --robot) robot="${2:?missing value for --robot}"; shift 2 ;;
    --robot=*) robot="${1#*=}"; shift ;;
    --backend) backend="${2:?missing value for --backend}"; shift 2 ;;
    --backend=*) backend="${1#*=}"; shift ;;
    --task) task="${2:?missing value for --task}"; shift 2 ;;
    --task=*) task="${1#*=}"; shift ;;
    --role) role="${2:?missing value for --role}"; shift 2 ;;
    --role=*) role="${1#*=}"; shift ;;
    --seed) seed="${2:?missing value for --seed}"; shift 2 ;;
    --seed=*) seed="${1#*=}"; shift ;;
    --run-id) run_id="${2:?missing value for --run-id}"; shift 2 ;;
    --run-id=*) run_id="${1#*=}"; shift ;;
    --run-root) run_root="${2:?missing value for --run-root}"; shift 2 ;;
    --run-root=*) run_root="${1#*=}"; shift ;;
    --shared-root) shared_root="${2:?missing value for --shared-root}"; shift 2 ;;
    --shared-root=*) shared_root="${1#*=}"; shift ;;
    --realtime-speed) realtime_speed="${2:?missing value for --realtime-speed}"; shift 2 ;;
    --realtime-speed=*) realtime_speed="${1#*=}"; shift ;;
    --namespace) namespace="${2:?missing value for --namespace}"; shift 2 ;;
    --namespace=*) namespace="${1#*=}"; shift ;;
    --no-visualization) no_visualization=true; shift ;;
    --config) direct_config="${2:?missing value for --config}"; shift 2 ;;
    --config=*) direct_config="${1#*=}"; shift ;;
    -h|--help|help)
      usage
      exit 0
      ;;
    *)
      printf 'scripts/backend.sh: unknown argument %q\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

python3 "${SHARED_ROOT:-/shared}/common/tools/scripts/dev_workspace.py" check \
  --project-root / \
  --build-root "$ROS_SDK" >/dev/null

if [[ -n "$direct_config" ]]; then
  if [[ -n "$robot" || -n "$backend" || -n "$task" ]]; then
    printf 'scripts/backend.sh: --config cannot be combined with --robot/--backend/--task\n' >&2
    exit 2
  fi
  target_root="${run_root:-$DATA_ROOT/_direct/logs/backend}"
  cmd=(python3 -m robot_backend_sim.common.ros.node --config "$direct_config" --run-root "$target_root")
  [[ -n "$seed" ]] && cmd+=(--seed "$seed")
  [[ -n "$run_id" ]] && cmd+=(--run-id "$run_id")
  [[ -n "$realtime_speed" ]] && cmd+=(--realtime-speed "$realtime_speed")
  [[ -n "$namespace" ]] && cmd+=(--namespace "$namespace")
  [[ "$no_visualization" == "true" ]] && cmd+=(--no-visualization)
  exec "${cmd[@]}"
fi

if [[ -z "$robot" || -z "$backend" || -z "$task" ]]; then
  printf 'scripts/backend.sh: --robot, --backend, and --task are required (unless using --config)\n' >&2
  exit 2
fi

role="${role:-teacher}"
run_root="${run_root:-$DATA_ROOT/$robot/$backend/$task/logs/backend}"

ts="$(date +%Y%m%d_%H%M%S)"
base="${run_id:-${role}-backend}"
if [[ ! "$base" =~ (_[0-9]{8}_[0-9]{6}|-[0-9]{8}T[0-9]{6}Z)$ ]]; then
  run_id="${base}_${ts}"
else
  run_id="$base"
fi
if [[ -e "$run_root/$run_id" ]]; then
  suffix=2
  while [[ -e "$run_root/${run_id}_${suffix}" ]]; do
    suffix=$((suffix + 1))
  done
  run_id="${run_id}_${suffix}"
fi

cmd=(
  python3 -m robot_backend_sim.common.ros.node
  --robot "$robot"
  --backend "$backend"
  --task "$task"
  --role "$role"
  --run-id "$run_id"
  --run-root "$run_root"
)
[[ -n "$seed" ]] && cmd+=(--seed "$seed")
[[ -n "$shared_root" ]] && cmd+=(--shared-root "$shared_root")
[[ -n "$realtime_speed" ]] && cmd+=(--realtime-speed "$realtime_speed")
[[ -n "$namespace" ]] && cmd+=(--namespace "$namespace")
[[ "$no_visualization" == "true" ]] && cmd+=(--no-visualization)

exec "${cmd[@]}"
