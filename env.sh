#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_NAME="sim-dual-arm"
DEFAULT_DOCKER_MIRRORS=(
  "docker.m.daocloud.io"
  "docker.1ms.run"
  "docker.xuanyuan.me"
)

usage() {
  cat <<'EOF'
用法:
  ./env.sh <env>                  复用本地镜像、后台启动并进入容器 (等同于 enter <env>)
  ./env.sh enter <env>            启动已有镜像后进入容器 bash
  ./env.sh up <env>               后台启动，不构建或拉取镜像
  ./env.sh build <env>            显式构建指定环境镜像（先并行测连通性，再选用直连或可用镜像源）
  ./env.sh exec <env> [cmd...]    进入已运行的容器执行命令 (默认 bash)
  ./env.sh stop <env>             停止指定环境容器
  ./env.sh rm <env>               删除指定环境容器 (先 stop 再 rm -f)
  ./env.sh viewer <args...>       在宿主机启动 MuJoCo 实时可视化 (需传入 --robot <id> --task <id>)
  ./env.sh ps                     查看当前项目所有容器状态
  ./env.sh list                   列出支持的 Docker 环境

可用环境 (<env>):
  runtime       ROS 2 运行端   (runtime/ros2/docker/docker-compose.yml)
  simulation    MuJoCo 仿真端  (simulation/mujoco/docker/docker-compose.yml)
  learning      LeRobot 学习端 (learning/lerobot/docker/docker-compose.yml)
  hardware      天机硬件端     (hardware/tianji/docker/docker-compose.yml)

构建镜像源环境变量 (用于 ./env.sh build <env>):
  DOCKER_MIRROR=<host>            跳过探测，直接使用指定镜像源 (如 docker.m.daocloud.io)
  DOCKER_MIRRORS="<h1 h2 ...>"    自定义探测列表（空格分隔）
  DOCKER_PROBE_TIMEOUT_S=<sec>    单源连通性探测超时秒数 (默认: 10)
EOF
}

resolve_env() {
  case "${1:-}" in
    runtime)
      COMPOSE_FILE="$ROOT_DIR/runtime/ros2/docker/docker-compose.yml"
      SERVICE="runtime"
      BASE_IMAGE_DIRECT="ros:jazzy-ros-base-noble"
      BASE_IMAGE_REPO="library/ros:jazzy-ros-base-noble"
      ;;
    simulation)
      COMPOSE_FILE="$ROOT_DIR/simulation/mujoco/docker/docker-compose.yml"
      SERVICE="simulation"
      BASE_IMAGE_DIRECT="ros:jazzy-ros-base-noble"
      BASE_IMAGE_REPO="library/ros:jazzy-ros-base-noble"
      ;;
    learning)
      COMPOSE_FILE="$ROOT_DIR/learning/lerobot/docker/docker-compose.yml"
      SERVICE="learning"
      BASE_IMAGE_DIRECT="huggingface/lerobot-gpu:latest"
      BASE_IMAGE_REPO="huggingface/lerobot-gpu:latest"
      ;;
    hardware)
      COMPOSE_FILE="$ROOT_DIR/hardware/tianji/docker/docker-compose.yml"
      SERVICE="hardware"
      BASE_IMAGE_DIRECT="ros:jazzy-ros-base-noble"
      BASE_IMAGE_REPO="library/ros:jazzy-ros-base-noble"
      ;;
    "")
      usage
      exit 1
      ;;
    *)
      echo "错误: 未知环境 '$1'。可用环境: runtime, simulation, learning, hardware" >&2
      exit 1
      ;;
  esac
}

normalize_mirror() {
  local raw="${1:-}"
  raw="${raw#https://}"
  raw="${raw#http://}"
  raw="${raw%/}"
  printf '%s' "$raw"
}

probe_image_ref() {
  local ref="$1"
  local timeout_s="$2"
  if command -v timeout >/dev/null 2>&1; then
    timeout "$timeout_s" docker manifest inspect "$ref" >/dev/null 2>&1
  else
    docker manifest inspect "$ref" >/dev/null 2>&1
  fi
}

cleanup_probe() {
  local dir="$1"
  shift || true
  local pid=""
  for pid in "$@"; do
    pkill -P "$pid" 2>/dev/null || true
    kill "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  done
  rm -rf -- "$dir"
}

cmd_up() {
  local env_name="${1:-}"
  resolve_env "$env_name"
  mkdir -p "$ROOT_DIR/build/ros-dev" "$ROOT_DIR/build/generated/policy-sdk" "$ROOT_DIR/var"
  docker compose -f "$COMPOSE_FILE" up -d --no-build --pull never "$SERVICE"
}

cmd_build() {
  resolve_env "${1:-}"
  local mirror=""
  local probe_timeout="${DOCKER_PROBE_TIMEOUT_S:-10}"
  local -a mirrors=()
  local -a candidate_refs=("$BASE_IMAGE_DIRECT")
  local -a candidate_labels=("直连 Docker Hub")
  local -a pids=()
  local probe_dir=""
  local selected_idx=""
  local idx=0

  if [[ -n "${DOCKER_MIRROR:-}" ]]; then
    mirror="$(normalize_mirror "$DOCKER_MIRROR")"
    if [[ -z "$mirror" ]]; then
      echo "错误: DOCKER_MIRROR 无效: '$DOCKER_MIRROR'" >&2
      return 2
    fi
    echo "使用指定镜像源构建: ${mirror}/${BASE_IMAGE_REPO}"
    docker compose -f "$COMPOSE_FILE" build --build-arg "BASE_IMAGE=${mirror}/${BASE_IMAGE_REPO}" "$SERVICE"
    return $?
  fi

  if [[ -n "${DOCKER_MIRRORS:-}" ]]; then
    read -r -a mirrors <<<"$DOCKER_MIRRORS"
  else
    mirrors=("${DEFAULT_DOCKER_MIRRORS[@]}")
  fi
  for raw_mirror in "${mirrors[@]}"; do
    mirror="$(normalize_mirror "$raw_mirror")"
    [[ -z "$mirror" ]] && continue
    candidate_refs+=("${mirror}/${BASE_IMAGE_REPO}")
    candidate_labels+=("${mirror}")
  done

  echo "正在并行探测基础镜像连通性 (超时 ${probe_timeout}s): ${candidate_refs[*]}"
  probe_dir="$(mktemp -d)"
  for idx in "${!candidate_refs[@]}"; do
    (
      if probe_image_ref "${candidate_refs[$idx]}" "$probe_timeout"; then
        : > "$probe_dir/${idx}.ok"
      fi
    ) &
    pids+=("$!")
  done

  wait "${pids[0]}" || true
  if [[ -f "$probe_dir/0.ok" ]]; then
    selected_idx=0
  else
    echo "直连 Docker Hub (${BASE_IMAGE_DIRECT}) 不可达，检查备用镜像源探测结果..." >&2
    for ((idx = 1; idx < ${#candidate_refs[@]}; idx++)); do
      wait "${pids[$idx]}" || true
      if [[ -f "$probe_dir/${idx}.ok" ]]; then
        selected_idx="$idx"
        break
      fi
    done
  fi

  cleanup_probe "$probe_dir" "${pids[@]}"

  if [[ -z "$selected_idx" ]]; then
    echo "错误: 直连与全部镜像源均不可达 (${candidate_refs[*]})；可尝试 DOCKER_PROBE_TIMEOUT_S=20 或手动指定 DOCKER_MIRROR=<mirror> ./env.sh build ${SERVICE}" >&2
    return 1
  fi

  if [[ "$selected_idx" -eq 0 ]]; then
    echo "连通性探测通过: 直连 Docker Hub (${BASE_IMAGE_DIRECT})"
    docker compose -f "$COMPOSE_FILE" build "$SERVICE"
  else
    echo "连通性探测通过，选用镜像源: ${candidate_refs[$selected_idx]}"
    docker compose -f "$COMPOSE_FILE" build --build-arg "BASE_IMAGE=${candidate_refs[$selected_idx]}" "$SERVICE"
  fi
}

cmd_exec() {
  local env_name="${1:-}"
  resolve_env "$env_name"
  shift || true
  if [[ $# -eq 0 ]]; then
    docker compose -f "$COMPOSE_FILE" exec "$SERVICE" bash
  else
    docker compose -f "$COMPOSE_FILE" exec "$SERVICE" "$@"
  fi
}

cmd_enter() {
  local env_name="${1:-}"
  cmd_up "$env_name"
  cmd_exec "$env_name"
}

cmd_stop() {
  local env_name="${1:-}"
  resolve_env "$env_name"
  docker compose -f "$COMPOSE_FILE" stop "$SERVICE"
}

cmd_rm() {
  local env_name="${1:-}"
  resolve_env "$env_name"
  docker compose -f "$COMPOSE_FILE" stop "$SERVICE"
  docker compose -f "$COMPOSE_FILE" rm -f "$SERVICE"
}

cmd_viewer() {
  resolve_env "simulation"
  if [[ -z "${DISPLAY:-}" ]]; then
    echo "错误: DISPLAY 未设置，无法启动 MuJoCo X11 可视化" >&2
    exit 1
  fi
  if [[ $# -eq 0 ]]; then
    echo "错误: viewer 必须显式传入 --robot <id> --task <id>" >&2
    exit 2
  fi
  docker compose -f "$COMPOSE_FILE" run --rm \
    -e DISPLAY="$DISPLAY" \
    -e XAUTHORITY=/tmp/.Xauthority \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "${XAUTHORITY:-/dev/null}:/tmp/.Xauthority:ro" \
    "$SERVICE" scripts/viewer.sh "$@"
}

main() {
  if [[ $# -eq 0 ]]; then
    usage
    exit 1
  fi

  case "$1" in
    -h|--help|help)
      usage
      ;;
    list)
      echo "runtime simulation learning hardware"
      ;;
    ps)
      docker compose -p "$PROJECT_NAME" ps
      ;;
    up)
      shift
      cmd_up "$@"
      ;;
    build)
      shift
      cmd_build "$@"
      ;;
    exec)
      shift
      cmd_exec "$@"
      ;;
    enter)
      shift
      cmd_enter "$@"
      ;;
    stop)
      shift
      cmd_stop "$@"
      ;;
    rm)
      shift
      cmd_rm "$@"
      ;;
    viewer)
      shift
      cmd_viewer "$@"
      ;;
    runtime|simulation|learning|hardware)
      cmd_enter "$1"
      ;;
    *)
      echo "错误: 未知命令或环境 '$1'" >&2
      usage
      exit 1
      ;;
  esac
}

main "$@"
