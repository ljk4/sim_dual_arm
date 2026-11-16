#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_NAME="sim-dual-arm"

usage() {
  cat <<'EOF'
用法:
  ./env.sh <env>                  复用本地镜像、后台启动并进入容器 (等同于 enter <env>)
  ./env.sh enter <env>            启动已有镜像后进入容器 bash
  ./env.sh up <env>               后台启动，不构建或拉取镜像
  ./env.sh build <env>            显式构建指定环境镜像
  ./env.sh exec <env> [cmd...]    进入已运行的容器执行命令 (默认 bash)
  ./env.sh stop <env>             停止指定环境容器
  ./env.sh rm <env>               删除指定环境容器 (先 stop 再 rm -f)
  ./env.sh viewer <args...>       在宿主机启动 MuJoCo 实时可视化 (显式指定方案或配置)
  ./env.sh ps                     查看当前项目所有容器状态
  ./env.sh list                   列出支持的 Docker 环境

可用环境 (<env>):
  runtime       ROS 2 运行端   (runtime/ros2/docker/docker-compose.yml)
  simulation    MuJoCo 仿真端  (simulation/mujoco/docker/docker-compose.yml)
  learning      LeRobot 学习端 (learning/lerobot/docker/docker-compose.yml)
EOF
}

resolve_env() {
  case "${1:-}" in
    runtime)
      COMPOSE_FILE="$ROOT_DIR/runtime/ros2/docker/docker-compose.yml"
      SERVICE="runtime"
      ;;
    simulation)
      COMPOSE_FILE="$ROOT_DIR/simulation/mujoco/docker/docker-compose.yml"
      SERVICE="simulation"
      ;;
    learning)
      COMPOSE_FILE="$ROOT_DIR/learning/lerobot/docker/docker-compose.yml"
      SERVICE="learning"
      ;;
    "")
      usage
      exit 1
      ;;
    *)
      echo "错误: 未知环境 '$1'。可用环境: runtime, simulation, learning" >&2
      exit 1
      ;;
  esac
}

cmd_up() {
  local env_name="${1:-}"
  resolve_env "$env_name"
  mkdir -p "$ROOT_DIR/build" "$ROOT_DIR/var"
  docker compose -f "$COMPOSE_FILE" up -d --no-build --pull never "$SERVICE"
}

cmd_build() {
  resolve_env "${1:-}"
  docker compose -f "$COMPOSE_FILE" build "$SERVICE"
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
    echo "错误: viewer 必须显式传入 --scenario <id> 或 --config <path>" >&2
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
      echo "runtime simulation learning"
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
    runtime|simulation|learning)
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
