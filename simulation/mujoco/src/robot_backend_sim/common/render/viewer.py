"""Live MuJoCo viewer for the simulation backend.

The viewer is intentionally separate from the physics process.  It rebuilds the
same read-only scene model, subscribes to the backend's full-physics debug state,
applies each snapshot to its own ``MjData`` and only renders.  It never steps
physics and never reads the backend's live ``MjData``.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import mujoco
import mujoco.viewer
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray

from robot_backend_sim.common.config.backend_config import load_backend_document
from robot_backend_sim.common.config.scenario_config import (
    ScenarioError,
    resolve_scenario_viewer,
)
from robot_backend_sim.common.config.scene_loader import load_scene_config_for
from robot_backend_sim.common.physics.robot_adapters import build_scene_for

_POLL_HZ = 60.0
_STATE_SIGNATURE = mujoco.mjtState.mjSTATE_FULLPHYSICS
_VIEWER_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--robot",
        required=True,
        help="robot id under shared/scenarios/<robot>/<backend>/<task>/",
    )
    parser.add_argument(
        "--backend",
        required=True,
        help="backend id under shared/scenarios/<robot>/<backend>/<task>/",
    )
    parser.add_argument(
        "--task",
        required=True,
        help="task id under shared/scenarios/<robot>/<backend>/<task>/",
    )
    return parser


def _load_backend(robot: str, backend: str, task: str):
    resolved = resolve_scenario_viewer(robot=robot, backend=backend, task=task)
    return load_backend_document(resolved.document, resolved.source)


def _state_size(model: mujoco.MjModel) -> int:
    return int(mujoco.mj_stateSize(model, _STATE_SIGNATURE))


def _apply_state(model: mujoco.MjModel, data: mujoco.MjData, values: object) -> bool:
    try:
        state = [float(value) for value in values]  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return False
    expected = _state_size(model)
    if len(state) != expected:
        print(
            f"viewer: state length {len(state)} != model state size {expected}; "
            "backend and viewer configs/assets differ",
            file=sys.stderr,
            flush=True,
        )
        return False
    try:
        mujoco.mj_setState(model, data, state, _STATE_SIGNATURE)
        mujoco.mj_forward(model, data)
    except Exception as error:  # noqa: BLE001 - a bad snapshot must not kill the viewer
        print(f"viewer: cannot apply state: {type(error).__name__}: {error}", file=sys.stderr)
        return False
    return True


def main() -> int:
    args = _parser().parse_args()

    try:
        backend = _load_backend(args.robot, args.backend, args.task)
    except (ScenarioError, OSError, ValueError) as error:
        print(f"错误: 无法加载可视化配置: {error}", file=sys.stderr)
        return 2
    if not backend.visualization_enabled:
        print("错误: 该方案的后端配方未开启可视化 (visualization_enabled=false)", file=sys.stderr)
        return 2
    scene_config = load_scene_config_for(backend.scene_path)  # type: ignore[arg-type]
    scene = build_scene_for(scene_config)
    model = scene.model
    data = scene.data
    topic = f"{backend.namespace.rstrip('/')}/visualization/mujoco_state"

    rclpy.init(args=[])
    node = Node("mujoco_live_viewer")
    latest: list[float] | None = None
    first_state = True

    def on_state(message: Float64MultiArray) -> None:
        nonlocal latest, first_state
        latest = list(message.data)
        if first_state:
            print("viewer: 已收到首帧物理快照，画面实时同步中", flush=True)
            first_state = False

    subscription = node.create_subscription(Float64MultiArray, topic, on_state, _VIEWER_QOS)
    del subscription
    print(
        "=== MuJoCo 实时可视化 ===\n"
        f"方案: {args.robot}/{args.backend}/{args.task} | 场景: {backend.scene_path}\n"
        f"订阅话题: {topic} | 刷新率: {int(_POLL_HZ)} Hz\n"
        "正在等待后端物理快照（关闭窗口或按 Ctrl-C 退出）...",
        flush=True,
    )

    viewer = None
    exit_ok = False
    try:
        try:
            viewer = mujoco.viewer.launch_passive(model, data)
        except Exception as error:  # noqa: BLE001 - X11 / GLFW launch errors need clear hint
            print(
                f"错误: 启动 MuJoCo 窗口失败 ({type(error).__name__}): {error}\n"
                "提示: 若为 X11 权限问题，请在宿主机执行: xhost +SI:localuser:$(id -un)",
                file=sys.stderr,
                flush=True,
            )
            return 1
        viewer.cam.lookat[:] = model.stat.center
        viewer.cam.distance = max(1.0, float(model.stat.extent) * 1.5)
        viewer.cam.azimuth = 135.0
        viewer.cam.elevation = -20.0
        period_s = 1.0 / _POLL_HZ
        next_sync = time.monotonic()
        while viewer.is_running():
            rclpy.spin_once(node, timeout_sec=0.0)
            if latest is not None:
                _apply_state(model, data, latest)
                latest = None
            viewer.sync()
            next_sync += period_s
            sleep_s = next_sync - time.monotonic()
            if sleep_s > 0.0:
                time.sleep(sleep_s)
            else:
                next_sync = time.monotonic()
        exit_ok = True
    except KeyboardInterrupt:
        exit_ok = True
    finally:
        if viewer is not None:
            try:
                viewer.close()
            except Exception as error:  # noqa: BLE001 - process is already exiting
                print(f"viewer: close failed: {type(error).__name__}: {error}", file=sys.stderr)
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        sys.stdout.flush()
        sys.stderr.flush()

    # Some NVIDIA/X11 combinations crash during Python/GLFW interpreter teardown
    # after the viewer window has already closed.  All viewer/ROS cleanup above
    # is complete, so exit without re-entering third-party destructors.
    os._exit(0 if exit_ok else 1)


if __name__ == "__main__":
    raise SystemExit(main())
