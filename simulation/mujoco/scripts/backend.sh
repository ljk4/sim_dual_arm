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

Start the MuJoCo Robot API backend inside the simulation container.

options:
  --robot <id>          robot id under shared/scenarios/<robot>/<task>/
  --task <id>           task id under shared/scenarios/<robot>/<task>/
  --role <role>         teacher | policy (scenario mode default: teacher)
  --seed <int>          initial simulation seed (override run.seed)
  --run-id <id>         backend run id prefix
  --run-root <path>     override backend run root
  --shared-root <path>  override shared scenarios root
  --realtime-speed <n>  simulation seconds per wall-clock second
  --namespace <ns>      override robot.namespace
  --no-visualization    disable MuJoCo state visualization topic
  --config <path>       direct backend YAML recipe (mutually exclusive with --robot/--task)
  -h, --help            show this help message
EOF
}

robot=""
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
  --build-root "$ROS_SDK" 1>&2

if [[ -n "$direct_config" ]]; then
  if [[ -n "$robot" || -n "$task" ]]; then
    printf 'scripts/backend.sh: --config cannot be combined with --robot/--task\n' >&2
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

if [[ -z "$robot" || -n "$task" && -z "$robot" || -z "$task" ]]; then
  printf 'scripts/backend.sh: --robot and --task are required (unless using --config)\n' >&2
  exit 2
fi

role="${role:-teacher}"
run_root="${run_root:-$DATA_ROOT/$robot/$task/logs/backend}"

ts="$(date +%Y%m%d_%H%M%S)"
base="${run_id:-${role}-backend}"
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

python3 - "$robot" "$task" "$role" "$seed" "$run_id" "$run_root" "$shared_root" "$realtime_speed" "$namespace" "$no_visualization" <<'PY'
import sys
from pathlib import Path
import yaml
from robot_backend_sim.common.config.scenario_config import resolve_scenario_backend

robot, task, role, seed_str, run_id, run_root, shared_root, speed, namespace, no_vis = sys.argv[1:]
resolved = resolve_scenario_backend(
    robot=robot,
    task=task,
    role=role,
    shared_root=Path(shared_root) if shared_root else None,
    seed=int(seed_str) if seed_str else None,
    realtime_speed=float(speed) if speed else None,
)
doc = resolved.document
if namespace:
    doc.setdefault("robot", {})["namespace"] = namespace
if no_vis == "true":
    doc.setdefault("robot", {})["visualization_enabled"] = False
run_dir = Path(run_root) / run_id
Path(run_root).mkdir(parents=True, exist_ok=True)
parameters_path = Path(run_root) / f"{run_id}-startup-parameters.yaml"
parameters_path.write_text(
    yaml.safe_dump(doc, sort_keys=False, allow_unicode=True),
    encoding="utf-8",
)
effective_seed = doc.get("run", {}).get("seed")
print(f"方案: {robot}/{task} | 角色: {role} | seed: {effective_seed}")
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
