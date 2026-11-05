"""Real MuJoCo three-chunk run used by the v3 audit evidence test.

This helper has no ROS dependency.  It drives the production ``NativeController``
and records the actual accepted references, control events, feedback snapshots
and state captures.  EGL rendering is attempted from the same immutable capture
snapshots; a host without EGL returns ``frames=None`` and the ROS/MCAP test is
skipped rather than substituting synthetic images.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
for source in (
    ROOT / "simulation/mujoco/src",
    ROOT / "contracts/python/src",
    ROOT / "libraries/ros_support/src",
):
    if str(source) not in sys.path:
        sys.path.insert(0, str(source))
PROFILE_PATH = ROOT / "profiles/robots/panda_stack_v1/profile.json"
SCENE_PATH = ROOT / "simulation/mujoco/configs/scene/panda_stack.yaml"
CONTROL_PATH = ROOT / "simulation/mujoco/configs/control/panda_control.yaml"
NATIVE_CONFIG = ROOT / "runtime/ros2/configs/native_collect.yaml"
SCHEMAS = ROOT / "contracts/schemas"
DT_NS = 2_000_000
LEASE_NS = 10_000_000_000
SAMPLE_NS = 50_000_000
DURATION_NS = 3_000_000_000
RIGHT_JOINT1 = 8


@dataclass(frozen=True)
class PhysicalCapture:
    sequence: int
    sim_time_ns: int
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    snapshot: object
    frames: dict[str, bytes] | None
    render_error: str


@dataclass(frozen=True)
class PhysicalRun:
    context: object
    profile: object
    references: tuple[object, ...]
    events: tuple[object, ...]
    feedback: tuple[object, ...]
    terminal: object
    captures: tuple[PhysicalCapture, ...]
    render_error: str


def _plan(profile, control):
    from robot_backend_sim.reference_compiler import compile_reference_plan
    from robot_contracts.models import Target

    initial = profile.initial_positions
    targets = tuple(
        Target(
            time_ns,
            tuple(
                value
                + 0.05 * (3.0 * (time_ns / DURATION_NS) ** 2 - 2.0 * (time_ns / DURATION_NS) ** 3)
                if index == RIGHT_JOINT1
                else value
                for index, value in enumerate(initial)
            ),
        )
        for time_ns in range(SAMPLE_NS, DURATION_NS + 1, SAMPLE_NS)
    )
    return compile_reference_plan(initial, targets, profile.joints, control.max_accelerations)


def _source(plan):
    from robot_contracts.models import Target

    return tuple(
        Target(time_ns, plan.sample_interval(time_ns).positions)
        for time_ns in range(SAMPLE_NS, DURATION_NS + 1, SAMPLE_NS)
    )


def _capture(scene, profile, sequence: int, snapshot) -> PhysicalCapture:
    from robot_backend_sim.panda_io import PandaIO
    from robot_backend_sim.rendering import SceneRenderer, capture_snapshot

    if snapshot is None:
        snapshot = capture_snapshot(scene.model, scene.data)
    measurement = PandaIO(scene, profile).measure()
    frames = None
    render_error = ""
    try:
        renderer = SceneRenderer(
            scene.model,
            profile.camera_names,
            width=profile.cameras[0].width,
            height=profile.cameras[0].height,
        )
        try:
            rendered = renderer.render_rgb(snapshot)
            frames = {
                camera.name: bytes(rendered[camera.name].tobytes(order="C"))
                for camera in profile.cameras
            }
        finally:
            renderer.close()
    except Exception as error:  # EGL is optional on the agent host
        render_error = f"{type(error).__name__}: {error}"
    return PhysicalCapture(
        sequence=sequence,
        sim_time_ns=round(float(snapshot.sim_time) * 1_000_000_000),
        positions=tuple(measurement.positions),
        velocities=tuple(measurement.velocities),
        snapshot=snapshot,
        frames=frames,
        render_error=render_error,
    )


def run_three_physical_chunks() -> PhysicalRun:
    """Execute three real 1-second chunks and return actual evidence."""
    from dataclasses import replace

    from robot_backend_sim.chunk_plan import prepare_chunk
    from robot_backend_sim.control import NativeController
    from robot_backend_sim.control_state import PreparedControlCommand
    from robot_backend_sim.motion_config import load_control_config
    from robot_backend_sim.panda_io import PandaIO
    from robot_backend_sim.protocol import accepted_reference, control_event, execution_feedback
    from robot_backend_sim.rendering import capture_snapshot
    from robot_backend_sim.scene import build_scene
    from robot_backend_sim.scene_config import load_scene_config
    from robot_backend_sim.stream_config import StreamConfig
    from robot_contracts.models import Target, Trajectory
    from robot_contracts.native import NATIVE_STREAM_FIRST, NATIVE_STREAM_NEXT, NativeRequest
    from robot_contracts.profiles import load_profile

    profile = load_profile(PROFILE_PATH, SCHEMAS)
    control = load_control_config(CONTROL_PATH)
    scene = build_scene(
        load_scene_config(SCENE_PATH),
    )
    scene.reset(0)
    io = PandaIO(scene, profile)
    controller = NativeController(
        scene,
        io,
        control,
        stream_config=StreamConfig(0.25, 0.05, 5.0, 120.0, 3.0, 2.0),
    )
    context = replace(controller.context, episode_id="v3-physical", control_epoch=1)
    lease = controller.acquire(context, "v3-physical", LEASE_NS, 0)
    plan = _plan(profile, control)
    source = _source(plan)
    captures: list[PhysicalCapture] = []
    references = []
    events = []
    feedback = []
    wall = 0
    queued = 1

    def packet_for(index: int, handoff_ns: int, observation_ns: int, start_sim_ns: int | None):
        anchor = controller.chunk_anchor(context, lease, wall, start_sim_ns)
        settle = index == 2
        target_count = 20 if index < 2 else 18
        relative = tuple(
            Target(target.time_ns - observation_ns, target.positions) for target in source
        )
        request = NativeRequest(
            trajectory=Trajectory(
                context,
                lease,
                "command-physical",
                1,
                index + 1,
                10_000_000_000,
                profile.joint_names,
                relative,
            ),
            kind=NATIVE_STREAM_FIRST if index == 0 else NATIVE_STREAM_NEXT,
            anchor=anchor,
            observation_time_ns=observation_ns,
            target_count=target_count,
            settle_at_end=settle,
            final_velocities=None
            if settle
            else plan.sample_interval(handoff_ns + 1_000_000_000).velocities,
        )
        prepared = prepare_chunk(
            anchor,
            request.window,
            profile,
            control,
            final_velocities=request.final_velocities,
            settle_at_end=request.settle_at_end,
        )
        return request, PreparedControlCommand(request.command, prepared, request=request)

    first_request, first_packet = packet_for(0, 100_000_000, 0, 100_000_000)
    captures.append(_capture(scene, profile, 1, capture_snapshot(scene.model, scene.data)))
    reply = controller.submit(first_packet, 0)
    if not reply.accepted:
        raise AssertionError(f"first chunk rejected: {reply.reason}")
    references.append(accepted_reference(first_request, first_packet, reply.snapshot))
    events.extend(control_event(event) for event in reply.events)
    if reply.status is not None:
        feedback.append(execution_feedback(reply.status))
    terminal = None
    while terminal is None:
        sim_ns = round(float(scene.data.time) * 1_000_000_000)
        handoff_ns = queued * 1_000_000_000 + 100_000_000
        if queued < 3 and sim_ns >= handoff_ns - 600_000_000:
            request, packet = packet_for(queued, handoff_ns, 0, handoff_ns)
            captures.append(
                _capture(scene, profile, queued + 1, capture_snapshot(scene.model, scene.data))
            )
            queued_reply = controller.queue_chunk(packet, sim_ns)
            if not queued_reply.accepted:
                raise AssertionError(f"chunk {queued + 1} rejected: {queued_reply.reason}")
            references.append(accepted_reference(request, packet, queued_reply.snapshot))
            events.extend(control_event(event) for event in queued_reply.events)
            if queued_reply.status is not None:
                feedback.append(execution_feedback(queued_reply.status))
            queued += 1
        wall = max(wall, sim_ns)
        step = controller.step(wall)
        events.extend(control_event(event) for event in step.events)
        if step.status is not None:
            feedback.append(execution_feedback(step.status))
            if step.status.phase in {"COMPLETED", "CANCELED", "FAILED"}:
                terminal = step.status
                break
        if len(feedback) > 20_000:
            raise AssertionError("physical run exceeded the feedback bound")
    if terminal is None or queued != 3:
        raise AssertionError("physical run did not execute all three chunks")
    render_error = next((capture.render_error for capture in captures if capture.render_error), "")
    return PhysicalRun(
        context=context,
        profile=profile,
        references=tuple(references),
        events=tuple(events),
        feedback=tuple(feedback),
        terminal=terminal,
        captures=tuple(captures),
        render_error=render_error,
    )
