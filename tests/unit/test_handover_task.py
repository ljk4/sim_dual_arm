"""Handover gate, Teacher isolation and final physical proof.

These tests drive the task runtime with synthetic owner observations.  Real
MuJoCo physics and the full runtime Teacher chain are exercised by the P03
integration runs; here the measured gate logic is isolated.
"""

from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

import pytest

pytest.importorskip("mujoco")
pytest.importorskip("mink")
from robot_contracts.models import RunContext  # noqa: E402
from robot_contracts.profiles import load_profile  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "simulation/mujoco/src"))

from robot_backend_sim.config.marvin_scene_config import load_marvin_scene_config  # noqa: E402
from robot_backend_sim.config.motion_config import load_control_config  # noqa: E402
from robot_backend_sim.physics.marvin.scene import build_marvin_scene  # noqa: E402
from robot_backend_sim.tasks.handover.evaluator import HandoverTaskRuntime  # noqa: E402
from robot_backend_sim.tasks.handover.gate import HandoverFeedback  # noqa: E402
from robot_backend_sim.tasks.handover.observation import (  # noqa: E402
    FingerContact,
    HandoverBinding,
    HandoverObservation,
    SupportContact,
)
from robot_backend_sim.tasks.handover.plan import HandoverLayout, layout_from_config  # noqa: E402
from robot_backend_sim.tasks.handover.task_config import (  # noqa: E402
    load_handover_task_config,
)
from robot_backend_sim.tasks.handover.teacher import (  # noqa: E402
    HandoverPlan,
    HandoverSnapshot,
    HandoverStep,
    build_handover_plan,
    sample_handover_chunk,
)

SCENARIO = ROOT / "shared/scenarios/marvin_handover"
OPEN = 0.0814
CLOSE = 0.05

LAYOUT = HandoverLayout(
    base_position=(0.0, 0.0, 0.0),
    base_yaw_deg=0.0,
    table_top_z=0.78,
    block_name="block",
    block_half_extents=(0.03, 0.10, 0.03),
    block_initial_center=(0.55, 0.12, 0.81),
    block_initial_yaw_rad=0.0,
    box_name="box",
    box_center=(0.64, -0.27, 0.78),
    box_inner_xy=(0.296, 0.296),
    box_inner_bottom_z=0.792,
    box_rim_z=0.88,
    box_yaw_rad=0.0,
)
BINDING = HandoverBinding(
    block_name="block",
    block_body_id=1,
    block_geom_name="block_geom",
    block_qpos_address=0,
    block_dof_address=0,
    table_body_id=2,
    box_body_id=3,
    support_body_names=("table", "box"),
    left_finger_dof_addresses=(0, 1),
    right_finger_dof_addresses=(2, 3),
    left_body_ids=frozenset(),
    right_body_ids=frozenset(),
    robot_body_ids=frozenset(),
    table_top_z_m=0.78,
    block_half_extents_m=(0.03, 0.10, 0.03),
    box_inner_bottom_z_m=0.792,
    box_center_xy=(0.64, -0.27),
    box_inner_xy=(0.296, 0.296),
    box_yaw_rad=0.0,
    block_initial_center=(0.55, 0.12, 0.81),
)
PROFILE = load_profile(
    SCENARIO / "profiles/robots/marvin_handover_v1/profile.json",
    ROOT / "shared/common/interfaces/schemas",
)
CONTROL = load_control_config(SCENARIO / "profiles/control/marvin_control_v1.yaml")
TASK = load_handover_task_config(
    SCENARIO / "configs/simulation/scene/marvin_handover_task.yaml"
)
CONTEXT = RunContext("marvin", PROFILE.digest, "backend", "time", 0, "episode", 1)


def _positions(left_open: float, right_open: float) -> tuple[float, ...]:
    return (0.0,) * 7 + (left_open,) + (0.0,) * 7 + (right_open,)


def _holding() -> tuple[FingerContact, FingerContact]:
    return (FingerContact(1, 2.0, 1), FingerContact(2, 2.0, 1))


def _observation(
    time_ns: int,
    *,
    block_center: tuple[float, float, float] = (0.55, 0.12, 0.81),
    left_open: float = OPEN,
    right_open: float = OPEN,
    left_holding: bool = False,
    right_holding: bool = False,
    left_tcp_offset: tuple[float, float, float] = (0.0, 0.06, 0.035),
    right_tcp_offset: tuple[float, float, float] = (0.0, -0.06, 0.035),
    supports: tuple[SupportContact, ...] = (),
) -> HandoverObservation:
    left_tcp = tuple(block_center[i] + left_tcp_offset[i] for i in range(3))
    right_tcp = tuple(block_center[i] + right_tcp_offset[i] for i in range(3))
    return HandoverObservation(
        sim_time_ns=time_ns,
        block_center=block_center,
        block_quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
        block_linear_velocity=(0.0, 0.0, 0.0),
        block_angular_velocity=(0.0, 0.0, 0.0),
        left_positions=(0.0,) * 7,
        right_positions=(0.0,) * 7,
        left_velocities=(0.0,) * 7,
        right_velocities=(0.0,) * 7,
        left_opening_m=left_open,
        right_opening_m=right_open,
        left_finger_speed_m_s=0.0,
        right_finger_speed_m_s=0.0,
        left_contacts=_holding() if left_holding else (),
        right_contacts=_holding() if right_holding else (),
        support_contacts=supports,
        left_tcp_position=left_tcp,
        left_tcp_quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
        right_tcp_position=right_tcp,
        right_tcp_quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
        robot_robot_force_n=0.0,
        robot_box_force_n=0.0,
        _initial_center=(0.55, 0.12, 0.81),
    )


def _steps() -> tuple[HandoverStep, ...]:
    sequence = [
        ("left_ready", "left_approach", "position", (), OPEN, OPEN, False, False),
        ("left_pregrasp", "left_approach", "position", (), OPEN, OPEN, False, False),
        ("left_grasp_pose", "left_approach", "position", (), OPEN, OPEN, False, False),
        ("left_close", "left_grasp", "left_grasp", ("left",), CLOSE, OPEN, False, False),
        ("left_lift", "left_lift", "left_lift", ("left",), CLOSE, OPEN, False, False),
        ("left_handover", "handover_approach", "handover", ("left",), CLOSE, OPEN, False, False),
        ("right_wait", "handover_approach", "handover", ("left",), CLOSE, OPEN, False, False),
        ("right_pregrasp", "handover_approach", "handover", ("left",), CLOSE, OPEN, False, False),
        ("right_grasp", "handover_approach", "handover", ("left",), CLOSE, OPEN, False, False),
        ("right_close", "dual_hold", "dual_hold", ("left", "right"), CLOSE, CLOSE, False, False),
        ("left_open", "left_release", "left_release", ("right",), OPEN, CLOSE, False, False),
        ("left_retreat", "left_retreat", "left_retreat", ("right",), OPEN, CLOSE, False, False),
        ("right_preplace", "right_transport", "right_carry", ("right",), OPEN, CLOSE, False, False),
        ("right_place", "right_place", "right_place", ("right",), OPEN, CLOSE, True, False),
        ("right_open", "right_release", "released", (), OPEN, OPEN, True, False),
        ("stable", "stable", "stable", (), OPEN, OPEN, True, True),
    ]
    steps = []
    for index, (name, phase, gate, holders, left, right, box, final) in enumerate(sequence):
        positions = _positions(left, right)
        steps.append(
            HandoverStep(
                index=index,
                name=name,
                phase=phase,
                start_positions=positions,
                end_positions=positions,
                duration_s=0.2 if gate not in ("position",) else 0.0,
                gate=gate,
                holders=holders,
                allow_box_support=box,
                final=final,
            )
        )
    return tuple(steps)


def _plan(steps: tuple[HandoverStep, ...]) -> HandoverPlan:
    return HandoverPlan(
        context=CONTEXT,
        task_epoch=1,
        config_digest=TASK.config_digest,
        plan_digest="0" * 64,
        steps=steps,
        duration_ns=round(sum(step.duration_s for step in steps) * 1e9) or 1,
        open_total_m=OPEN,
        velocities=tuple(float(joint.max_velocity) for joint in PROFILE.joints),
        accelerations=(10.0,) * 16,
        ik_max_position_error_m=0.0,
        ik_max_orientation_error_rad=0.0,
        layout=LAYOUT,
    )


def _runtime(steps: tuple[HandoverStep, ...]) -> HandoverTaskRuntime:
    runtime = HandoverTaskRuntime(TASK, BINDING, LAYOUT)
    plan = _plan(steps)
    outcome = runtime.commit_prepare(
        request_id="prepare",
        content_digest="content",
        context=CONTEXT,
        plan=plan,
        plan_digest=plan.plan_digest,
        plan_duration_ns=plan.duration_ns,
        step_count=len(steps),
    )
    assert outcome.fact.state.phase == "PREPARED"
    state, idempotent = runtime.start(
        request_id="start",
        content_digest="start-content",
        context=CONTEXT,
        plan_digest=plan.plan_digest,
        task_epoch=1,
        sim_time_ns=0,
        wall_ns=0,
    )
    assert state.phase == "RUNNING" and not idempotent
    return runtime


def _run(runtime: HandoverTaskRuntime, observation_factory, seconds: float, start_ns: int):
    time_ns = start_ns
    for _ in range(int(round(seconds / 0.1))):
        time_ns += 100_000_000
        state = runtime.tick(observation_factory(time_ns), time_ns)
        if state.terminal:
            return state, time_ns
    return runtime.state, time_ns


def _airborne(time_ns: int, **kwargs) -> HandoverObservation:
    return _observation(time_ns, block_center=(0.55, 0.12, 0.93), **kwargs)


def _advance_to_dual_hold(runtime: HandoverTaskRuntime) -> int:
    time_ns = 0
    def approach(time_ns: int) -> HandoverObservation:
        return _observation(time_ns, left_open=OPEN, right_open=OPEN)


    def left_hold(time_ns: int, **kwargs) -> HandoverObservation:
        return _airborne(time_ns, left_open=CLOSE, left_holding=True, **kwargs)

    for _ in range(3):
        state, time_ns = _run(runtime, approach, 0.7, time_ns)
        assert state.phase == "RUNNING"
    # Table grasp close: the block is still on the table and the left holds it.
    state, time_ns = _run(
        runtime,
        lambda time_ns: _observation(
            time_ns, left_open=CLOSE, left_holding=True,
            supports=(SupportContact("table", 3.0, 1),),
        ),
        0.4,
        time_ns,
    )
    assert runtime.behavior_phase == "left_grasp", runtime.state.metrics
    # Lift and handover steps.
    for _ in range(5):
        state, time_ns = _run(runtime, left_hold, 0.7, time_ns)
        assert state.phase == "RUNNING", state.failure_reason
    assert runtime.behavior_phase == "dual_hold", runtime.state.metrics
    return time_ns


def _chunk(runtime: HandoverTaskRuntime, time_ns: int, gate_fact):
    return sample_handover_chunk(
        runtime.plan,
        gate=gate_fact,
        feedback=HandoverFeedback(1, time_ns, gate_fact),
        request_id="chunk",
        task_epoch=1,
        capture_seq=1,
        observation_time_ns=time_ns,
        chunk_size=60,
        period_ns=50_000_000,
        profile=PROFILE,
        control_config=CONTROL,
        start_sim_time_ns=time_ns,
    )


def test_right_not_grasped_keeps_left_closed_and_times_out():
    runtime = _runtime(_steps())
    time_ns = _advance_to_dual_hold(runtime)
    # The right approach reached the same pose but its fingers never hold.
    observation = _airborne(
        time_ns, left_open=CLOSE, left_holding=True, right_open=CLOSE, right_holding=False
    )
    state = runtime.tick(observation, time_ns)
    gate = runtime.gate_fact()
    assert gate.step == "right_close" and gate.failure == ""
    chunk = _chunk(runtime, time_ns, gate)
    assert chunk.phase == "dual_hold"
    assert all(position[7] == CLOSE for position in chunk.positions)
    state, time_ns = _run(
        runtime,
        lambda time_ns: _airborne(
            time_ns, left_open=CLOSE, left_holding=True,
            right_open=CLOSE, right_holding=False,
        ),
        TASK.gates.phase_extra_timeout_s + 0.6,
        time_ns,
    )
    assert state.phase == "FAILED", state.metrics
    assert state.failure_reason.startswith("GATE_TIMEOUT")
    metrics = state.metrics
    assert metrics["behavior_phase"] == "dual_hold"
    assert metrics["left_holding"] is True
    assert metrics["right_holding"] is False
    assert metrics["left_opening_m"] == pytest.approx(CLOSE)
    assert metrics["right_catch_sim_time_ns"] == 0
    assert metrics["left_release_sim_time_ns"] == 0


def test_left_release_only_after_right_is_measured_stable():
    runtime = _runtime(_steps())
    time_ns = _advance_to_dual_hold(runtime)
    state, time_ns = _run(
        runtime,
        lambda time_ns: _airborne(
            time_ns, left_open=CLOSE, left_holding=True,
            right_open=CLOSE, right_holding=True,
        ),
        1.0,
        time_ns,
    )
    assert runtime.behavior_phase == "left_release", state.metrics
    metrics = runtime.state.metrics
    assert metrics["right_catch_sim_time_ns"] > 0
    # The release chunk is the first one that commands the left gripper open.
    gate = runtime.gate_fact()
    chunk = _chunk(runtime, time_ns, gate)
    assert chunk.phase == "left_release"
    assert all(position[7] == pytest.approx(OPEN) for position in chunk.positions)


def test_real_plan_is_a_gated_handover_sequence():
    scene_config = load_marvin_scene_config(
        SCENARIO / "configs/simulation/scene/marvin_handover.yaml"
    )
    scene = build_marvin_scene(scene_config)
    scene.reset(0)
    address = int(scene.object_free_joint_qpos["block"])
    layout = layout_from_config(scene_config, scene.data.qpos, address)
    snapshot = HandoverSnapshot(
        context=CONTEXT,
        task_epoch=1,
        sim_time_ns=0,
        capture_seq=0,
        qpos=tuple(float(value) for value in scene.data.qpos),
        seed=0,
        controller_command_id="",
    )
    plan = build_handover_plan(
        scene.model,
        snapshot,
        PROFILE,
        CONTROL,
        TASK,
        scene_config=scene_config,
        robot=scene.indices.robot,
        robot_geometry=scene.robot_geometry,
        object_free_joint_qpos=dict(scene.object_free_joint_qpos),
        support_body_names=tuple(scene.support_body_names),
    )
    names = [step.name for step in plan.steps]
    assert names.index("left_close") < names.index("left_lift")
    assert names.index("right_close") < names.index("left_open")
    assert names.index("left_open") < names.index("right_preplace")
    assert names.index("right_place") < names.index("right_open")
    assert plan.steps[-1].final and plan.steps[-1].phase == "stable"
    assert plan.steps[names.index("left_open")].holders == ("right",)
    assert plan.duration_s < TASK.success.max_task_sim_duration_s
    assert plan.ik_max_position_error_m <= TASK.planning.ik.position_tolerance_m + 1e-12
    assert (
        plan.ik_max_orientation_error_rad
        <= TASK.planning.ik.orientation_tolerance_rad + 1e-12
    )
    # The left release command is reachable only after the right close step.
    assert plan.steps[names.index("left_close")].end_positions[7] == pytest.approx(
        TASK.teacher.left_close_opening_m
    )
    assert plan.steps[names.index("right_close")].end_positions[15] == pytest.approx(
        TASK.teacher.right_close_opening_m
    )
    assert plan.layout.block_initial_center == layout.block_initial_center


def test_right_loss_after_left_release_fails_hold():
    runtime = _runtime(_steps())
    time_ns = _advance_to_dual_hold(runtime)
    state, time_ns = _run(
        runtime,
        lambda time_ns: _airborne(
            time_ns, left_open=CLOSE, left_holding=True,
            right_open=CLOSE, right_holding=True,
        ),
        1.0,
        time_ns,
    )
    assert runtime.behavior_phase == "left_release", state.metrics
    state, _time_ns = _run(
        runtime,
        lambda time_ns: _airborne(
            time_ns, left_open=OPEN, left_holding=False,
            right_open=CLOSE, right_holding=False,
        ),
        0.6,
        time_ns,
    )
    assert state.phase == "FAILED", state.metrics
    assert state.failure_reason.startswith("HOLD_LOST")
    assert state.metrics["behavior_phase"] == "left_release"


def test_position_gate_timeout_fails_the_task():
    runtime = _runtime(_steps())

    def wrong_pose(time_ns: int) -> HandoverObservation:
        observation = _observation(time_ns)
        return replace(
            observation,
            left_positions=(0.5,) + observation.left_positions[1:],
        )

    state, _time_ns = _run(runtime, wrong_pose, 9.0, 0)
    assert state.phase == "FAILED", state.metrics
    assert state.failure_reason.startswith("GATE_TIMEOUT")
    assert state.metrics["behavior_phase"] == "left_approach"


def test_evaluation_only_plan_uses_the_same_physical_success():
    runtime = _runtime(())
    placed = (0.64, -0.27, 0.822)
    state, _time_ns = _run(
        runtime,
        lambda time_ns: _observation(
            time_ns,
            block_center=placed,
            supports=(SupportContact("box", 3.0, 1),),
        ),
        1.4,
        0,
    )
    assert state.phase == "SUCCEEDED", state.metrics
    assert state.metrics["conditions_passed"] is True


def test_final_success_needs_containment_support_and_release():
    steps = (
        HandoverStep(
            index=0,
            name="stable",
            phase="stable",
            start_positions=_positions(OPEN, OPEN),
            end_positions=_positions(OPEN, OPEN),
            duration_s=0.5,
            gate="stable",
            holders=(),
            allow_box_support=True,
            final=True,
        ),
    )
    runtime = _runtime(steps)
    time_ns = 0
    placed = (0.64, -0.27, 0.822)
    state, time_ns = _run(
        runtime,
        lambda time_ns: _observation(
            time_ns,
            block_center=placed,
            supports=(SupportContact("box", 3.0, 1),),
        ),
        1.4,
        time_ns,
    )
    assert state.phase == "SUCCEEDED", state.metrics
    assert state.metrics["block_contained"] is True
    assert state.metrics["left_released"] is True
    assert state.metrics["right_released"] is True
    assert state.metrics["box_support_force_n"] >= TASK.success.min_box_support_normal_force_n


def test_block_above_the_rim_is_not_reported_as_success():
    steps = (
        HandoverStep(
            index=0,
            name="stable",
            phase="stable",
            start_positions=_positions(OPEN, OPEN),
            end_positions=_positions(OPEN, OPEN),
            duration_s=0.5,
            gate="stable",
            holders=(),
            allow_box_support=True,
            final=True,
        ),
    )
    runtime = _runtime(steps)
    state, _time_ns = _run(
        runtime,
        lambda time_ns: _observation(
            time_ns,
            block_center=(0.64, -0.27, 0.900),
            supports=(SupportContact("box", 3.0, 1),),
        ),
        1.5,
        0,
    )
    assert state.phase == "RUNNING", state.metrics
    assert state.metrics["block_contained"] is False
