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
from pathlib import Path

import mujoco
import mujoco.viewer
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray

from .backend_config import load_backend_config
from .scene import build_scene
from .scene_config import load_scene_config

_STATE_SIGNATURE = mujoco.mjtState.mjSTATE_FULLPHYSICS
_VIEWER_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument(
        "--state-topic",
        default="",
        help="defaults to <robot namespace>/visualization/mujoco_state",
    )
    parser.add_argument("--poll-hz", type=float, default=60.0)
    return parser


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
    if not 1.0 <= args.poll_hz <= 240.0:
        print("viewer: --poll-hz must be within [1, 240]", file=sys.stderr)
        return 2

    backend = load_backend_config(args.config)
    scene_config = load_scene_config(backend.scene_path)
    scene = build_scene(scene_config)
    model = scene.model
    data = scene.data
    topic = args.state_topic or f"{backend.namespace.rstrip('/')}/visualization/mujoco_state"

    rclpy.init(args=[])
    node = Node("mujoco_live_viewer")
    latest: list[float] | None = None
    first_state = True

    def on_state(message: Float64MultiArray) -> None:
        nonlocal latest, first_state
        latest = list(message.data)
        if first_state:
            print("viewer: received first physics snapshot", flush=True)
            first_state = False

    subscription = node.create_subscription(Float64MultiArray, topic, on_state, _VIEWER_QOS)
    del subscription
    print(f"viewer: listening on {topic}", flush=True)

    viewer = None
    exit_ok = False
    try:
        viewer = mujoco.viewer.launch_passive(model, data)
        viewer.cam.lookat[:] = (0.2, 0.55, 0.8)
        viewer.cam.distance = 1.55
        viewer.cam.azimuth = 135.0
        viewer.cam.elevation = -20.0
        period_s = 1.0 / args.poll_hz
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
