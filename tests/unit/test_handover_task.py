"""Handover gate, Teacher isolation and final physical proof.

These tests drive the task runtime with synthetic owner observations.  Real
MuJoCo contact evidence is exercised in ``test_handover_contact_evidence.py``;
here the measured gate, sequence and success logic is isolated.
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

SCENARIO = ROOT / "shared/scenarios/marvin_sim/handover"
OPEN = 0.0814
CLOSE = 0.05
AIRBORNE_CENTER = (0.55, 0.12, 0.93)
PLACED_CENTER = (0.64, -0.27, 0.822)

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
    box_bottom_geom_names=("box_bottom",),
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


def _holding(force: float = 2.0) -> tuple[FingerContact, FingerContact]:
    return (FingerContact(1, force, 1), FingerContact(2, force, 1))


def _support(
    body: str, geom: str, force: float, upward: float | None = None
) -> SupportContact:
    return SupportContact(body, geom, force, force if upward is None else upward, 1)


TABLE_SUPPORT = _support("table", "table_top", 3.0)
BOX_BOTTOM_SUPPORT = _support("box", "box_bottom", 3.0)
BOX_WALL_SUPPORT = _support("box", "box_wall_x_pos", 3.0, upward=0.0)


def _observation(
    time_ns: int,
    *,
    block_center: tuple[float, float, float] = (0.55, 0.12, 0.81),
    left_open: float = OPEN,
    right_open: float = OPEN,
    left_holding: bool = False,
    right_holding: bool = False,
    left_single: bool = False,
    right_single: bool = False,
    hold_force: float = 2.0,
    left_tcp_offset: tuple[float, float, float] = (0.0, 0.06, 0.035),
    right_tcp_offset: tuple[float, float, float] = (0.0, -0.06, 0.035),
    supports: tuple[SupportContact, ...] = (),
    robot_robot_force_n: float = 0.0,
    robot_box_force_n: float = 0.0,
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
        left_contacts=(
            _holding(hold_force)
            if left_holding
            else (FingerContact(1, hold_force, 1),)
            if left_single
            else ()
        ),
        right_contacts=(
            _holding(hold_force)
            if right_holding
            else (FingerContact(1, hold_force, 1),)
            if right_single
            else ()
        ),
        support_contacts=supports,
        left_tcp_position=left_tcp,
        left_tcp_quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
        right_tcp_position=right_tcp,
        right_tcp_quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
        robot_robot_force_n=robot_robot_force_n,
        robot_box_force_n=robot_box_force_n,
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
    return _observation(time_ns, block_center=AIRBORNE_CENTER, **kwargs)


def _placed(time_ns: int, *, right_holding: bool = False, **kwargs) -> HandoverObservation:
    return _observation(
        time_ns,
        block_center=PLACED_CENTER,
        left_open=OPEN,
        right_open=CLOSE if right_holding else OPEN,
        right_holding=right_holding,
        supports=(BOX_BOTTOM_SUPPORT,),
        **kwargs,
    )


def _left_airborne(time_ns: int) -> HandoverObservation:
    return _airborne(time_ns, left_open=CLOSE, left_holding=True)


def _dual_airborne(time_ns: int) -> HandoverObservation:
    return _airborne(
        time_ns, left_open=CLOSE, left_holding=True,
        right_open=CLOSE, right_holding=True,
    )


def _right_airborne(time_ns: int) -> HandoverObservation:
    return _airborne(
        time_ns, left_open=OPEN, left_holding=False,
        right_open=CLOSE, right_holding=True,
    )


def _complete_airborne_sequence(runtime: HandoverTaskRuntime, time_ns: int) -> int:
    for factory in (_left_airborne, _dual_airborne, _right_airborne):
        state, time_ns = _run(runtime, factory, 0.6, time_ns)
        assert state.phase == "RUNNING", state.metrics
    sequence = runtime.state.metrics["handover_sequence"]
    assert sequence["complete"] is True
    return time_ns


def _advance_to_dual_hold(runtime: HandoverTaskRuntime) -> int:
    time_ns = 0

    def approach(time_ns: int) -> HandoverObservation:
        return _observation(time_ns, left_open=OPEN, right_open=OPEN)

    for _ in range(3):
        state, time_ns = _run(runtime, approach, 0.7, time_ns)
        assert state.phase == "RUNNING"
    # Table grasp close: the block is still on the table and the left holds it.
    state, time_ns = _run(
        runtime,
        lambda time_ns: _observation(
            time_ns, left_open=CLOSE, left_holding=True, supports=(TABLE_SUPPORT,)
        ),
        0.4,
        time_ns,
    )
    assert runtime.behavior_phase == "left_grasp", runtime.state.metrics
    # Lift and handover steps.
    for _ in range(5):
        state, time_ns = _run(runtime, _left_airborne, 0.7, time_ns)
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
    assert metrics["handover_sequence"]["left_airborne_stable"] is True
    assert metrics["handover_sequence"]["dual_airborne_stable"] is False


def test_left_release_only_after_right_is_measured_stable():
    runtime = _runtime(_steps())
    time_ns = _advance_to_dual_hold(runtime)
    state, time_ns = _run(runtime, _dual_airborne, 1.0, time_ns)
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
    state, time_ns = _run(runtime, _dual_airborne, 1.0, time_ns)
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


def test_policy_plan_requires_the_complete_airborne_handover():
    runtime = _runtime(())
    state, _time_ns = _run(runtime, _placed, 1.4, 0)
    assert state.phase == "RUNNING", state.metrics
    assert state.metrics["conditions_passed"] is False
    assert state.metrics["handover_sequence"]["complete"] is False
    assert state.metrics["block_contained"] is True


def test_policy_plan_succeeds_after_the_measured_sequence():
    runtime = _runtime(())
    time_ns = _complete_airborne_sequence(runtime, 0)
    state, time_ns = _run(
        runtime, lambda time_ns: _placed(time_ns, right_holding=True), 0.1, time_ns
    )
    assert state.phase == "RUNNING", state.metrics
    state, _time_ns = _run(runtime, _placed, 1.1, time_ns)
    assert state.phase == "SUCCEEDED", state.metrics
    assert state.metrics["conditions_passed"] is True
    sequence = state.metrics["handover_sequence"]
    assert sequence["complete"] is True
    assert sequence["left_stable_sim_time_ns"] < sequence["dual_stable_sim_time_ns"]
    assert sequence["dual_stable_sim_time_ns"] <= sequence["left_release_sim_time_ns"]
    assert sequence["left_release_sim_time_ns"] < sequence["right_stable_sim_time_ns"]


def test_success_needs_the_configured_one_second_window_after_a_real_handover():
    runtime = _runtime(())
    time_ns = _complete_airborne_sequence(runtime, 0)
    placed_ns = time_ns + 100_000_000
    state = runtime.tick(_placed(placed_ns), placed_ns)
    assert state.phase == "RUNNING"
    state = runtime.tick(_placed(placed_ns + 999_000_000), placed_ns + 999_000_000)
    assert state.phase == "RUNNING"
    state = runtime.tick(_placed(placed_ns + 1_000_000_000), placed_ns + 1_000_000_000)
    assert state.phase == "SUCCEEDED"
    assert state.metrics["stable_elapsed_s"] == pytest.approx(1.0)


def test_teacher_plan_succeeds_only_after_the_same_sequence():
    runtime = _runtime(_steps())
    time_ns = _advance_to_dual_hold(runtime)
    state, time_ns = _run(runtime, _dual_airborne, 1.0, time_ns)
    assert runtime.behavior_phase == "left_release", state.metrics
    for _ in range(60):
        if runtime.behavior_phase == "right_place":
            break
        state, time_ns = _run(runtime, _right_airborne, 0.1, time_ns)
        assert state.phase == "RUNNING", state.metrics
    assert runtime.behavior_phase == "right_place", state.metrics
    for _ in range(30):
        if runtime.behavior_phase == "right_release":
            break
        state, time_ns = _run(
            runtime, lambda time_ns: _placed(time_ns, right_holding=True), 0.1, time_ns
        )
        assert state.phase == "RUNNING", state.metrics
    assert runtime.behavior_phase == "right_release", state.metrics
    state, time_ns = _run(runtime, _placed, 2.0, time_ns)
    assert state.phase == "SUCCEEDED", state.metrics
    assert state.metrics["handover_sequence"]["complete"] is True
    assert state.metrics["box_bottom_support_force_n"] >= (
        TASK.success.min_box_support_normal_force_n
    )


def test_dual_hold_without_left_sole_phase_never_completes():
    runtime = _runtime(())

    def both_from_the_start(time_ns: int) -> HandoverObservation:
        return _airborne(
            time_ns, left_open=CLOSE, left_holding=True,
            right_open=CLOSE, right_holding=True,
        )

    state, _time_ns = _run(runtime, both_from_the_start, 2.0, 0)
    assert state.phase == "RUNNING", state.metrics
    sequence = state.metrics["handover_sequence"]
    assert sequence["left_airborne_stable"] is False
    assert sequence["dual_airborne_stable"] is False
    assert sequence["complete"] is False


def test_right_single_finger_graze_is_not_dual_hold():
    runtime = _runtime(())
    time_ns = _advance_policy_left_phase(runtime)
    state, _time_ns = _run(
        runtime,
        lambda time_ns: _airborne(
            time_ns, left_open=CLOSE, left_holding=True,
            right_open=CLOSE, right_single=True,
        ),
        1.0,
        time_ns,
    )
    assert state.phase == "RUNNING", state.metrics
    assert state.metrics["handover_sequence"]["dual_airborne_stable"] is False


def test_tabletop_dual_pinch_is_not_an_airborne_handover():
    runtime = _runtime(())

    def tabletop_pinch(time_ns: int) -> HandoverObservation:
        return _observation(
            time_ns,
            left_open=CLOSE,
            right_open=CLOSE,
            left_holding=True,
            right_holding=True,
            supports=(TABLE_SUPPORT,),
        )

    state, _time_ns = _run(runtime, tabletop_pinch, 2.0, 0)
    assert state.phase == "RUNNING", state.metrics
    sequence = state.metrics["handover_sequence"]
    assert sequence["left_airborne_stable"] is False
    assert sequence["complete"] is False


def test_left_open_goal_with_contacts_is_not_release():
    runtime = _runtime(())
    time_ns = _complete_policy_dual_phase(runtime)
    state, _time_ns = _run(
        runtime,
        lambda time_ns: _airborne(
            time_ns, left_open=OPEN, left_holding=True,
            right_open=CLOSE, right_holding=True,
        ),
        1.0,
        time_ns,
    )
    assert state.phase == "RUNNING", state.metrics
    sequence = state.metrics["handover_sequence"]
    assert sequence["left_release_observed"] is False
    assert sequence["complete"] is False


def test_left_hold_loss_after_establishment_fails():
    runtime = _runtime(())
    time_ns = _advance_policy_left_phase(runtime)
    state, _time_ns = _run(
        runtime,
        lambda time_ns: _observation(time_ns, block_center=AIRBORNE_CENTER),
        0.6,
        time_ns,
    )
    assert state.phase == "FAILED", state.metrics
    assert state.failure_reason.startswith("HOLD_LOST")
    assert state.metrics["handover_sequence"]["left_airborne_stable"] is True


def test_right_hold_loss_after_dual_hold_fails():
    runtime = _runtime(())
    time_ns = _complete_policy_dual_phase(runtime)
    state, _time_ns = _run(
        runtime,
        lambda time_ns: _observation(
            time_ns, block_center=AIRBORNE_CENTER, left_open=CLOSE, left_holding=True
        ),
        0.6,
        time_ns,
    )
    assert state.phase == "FAILED", state.metrics
    assert state.failure_reason.startswith("HOLD_LOST")
    assert "right hand" in state.failure_reason
    assert state.metrics["handover_sequence"]["dual_airborne_stable"] is True


def test_policy_left_release_can_take_longer_than_hold_loss_tolerance():
    runtime = _runtime(())
    time_ns = _complete_policy_dual_phase(runtime)
    # A real opening ramp first loses one finger, then both, well before the
    # opening and low-speed release proof can pass. The right hand owns the
    # block after the already completed airborne dual-hold proof.
    state, time_ns = _run(
        runtime,
        lambda now: replace(
            _airborne(now, left_open=0.066, right_open=CLOSE, right_holding=True),
            left_finger_speed_m_s=0.014,
        ),
        0.8,
        time_ns,
    )
    assert state.phase == "RUNNING", state.metrics
    sequence = state.metrics["handover_sequence"]
    assert sequence["left_release_observed"] is False
    assert sequence["complete"] is False
    state, _time_ns = _run(runtime, _right_airborne, 0.6, time_ns)
    assert state.phase == "RUNNING", state.metrics
    assert state.metrics["handover_sequence"]["complete"] is True


def test_sequence_timer_does_not_accumulate_across_an_interruption():
    runtime = _runtime(())
    state, time_ns = _run(runtime, _left_airborne, 0.4, 0)
    state = runtime.tick(
        _observation(time_ns + 100_000_000, block_center=AIRBORNE_CENTER),
        time_ns + 100_000_000,
    )
    assert state.phase == "RUNNING"
    time_ns += 100_000_000
    state, time_ns = _run(runtime, _left_airborne, 0.4, time_ns)
    sequence = state.metrics["handover_sequence"]
    assert sequence["left_airborne_stable"] is False
    assert sequence["left_hold_elapsed_s"] < TASK.gates.stable_duration_s


def test_repeated_timestamps_do_not_accumulate_hold_time():
    runtime = _runtime(())
    state = runtime.tick(_left_airborne(0), 0)
    for _ in range(20):
        state = runtime.tick(_left_airborne(0), 0)
    assert state.phase == "RUNNING"
    sequence = state.metrics["handover_sequence"]
    assert sequence["left_airborne_stable"] is False
    assert sequence["left_hold_elapsed_s"] == pytest.approx(0.0)


def test_external_support_during_handover_fails():
    runtime = _runtime(())
    time_ns = _advance_policy_left_phase(runtime)
    state = runtime.tick(
        _airborne(
            time_ns + 100_000_000,
            left_open=CLOSE,
            left_holding=True,
            supports=(TABLE_SUPPORT,),
        ),
        time_ns + 100_000_000,
    )
    assert state.phase == "FAILED", state.metrics
    assert state.failure_reason.startswith("EXTERNAL_SUPPORT")
    assert state.metrics["handover_window"] is True
    assert state.metrics["handover_window_violation"] == "external support during handover"


def test_wall_only_box_contact_does_not_replace_bottom_support():
    runtime = _runtime(())
    time_ns = _complete_airborne_sequence(runtime, 0)
    state, _time_ns = _run(
        runtime,
        lambda time_ns: _observation(
            time_ns,
            block_center=PLACED_CENTER,
            left_open=OPEN,
            right_open=CLOSE,
            right_holding=True,
            supports=(BOX_WALL_SUPPORT,),
        ),
        1.4,
        time_ns,
    )
    assert state.phase == "RUNNING", state.metrics
    conditions = {item["name"]: item for item in state.metrics["conditions"]}
    assert conditions["box_bottom_support"]["passed"] is False
    assert conditions["box_bottom_support"]["actual"] == 0.0
    assert conditions["not_on_table"]["passed"] is True


def test_table_grasp_is_not_a_hold_loss():
    runtime = _runtime(())
    state, _time_ns = _run(
        runtime,
        lambda time_ns: _observation(
            time_ns, left_open=CLOSE, left_holding=True, supports=(TABLE_SUPPORT,)
        ),
        1.0,
        0,
    )
    assert state.phase == "RUNNING", state.metrics
    assert state.failure_reason == ""
    assert state.metrics["handover_sequence"]["left_airborne_stable"] is False


def test_short_hold_loss_within_the_original_tolerance_is_allowed():
    runtime = _runtime(())
    time_ns = _advance_policy_left_phase(runtime)
    state, time_ns = _run(
        runtime,
        lambda time_ns: _observation(time_ns, block_center=AIRBORNE_CENTER),
        0.2,
        time_ns,
    )
    assert state.phase == "RUNNING", state.metrics
    state, _time_ns = _run(runtime, _dual_airborne, 0.6, time_ns)
    assert state.phase == "RUNNING", state.metrics
    assert state.metrics["handover_sequence"]["dual_airborne_stable"] is True


@pytest.mark.parametrize("teacher", [False, True], ids=["policy", "teacher"])
def test_right_candidate_regrasp_uses_a_fresh_reference(teacher):
    runtime = _runtime(_steps() if teacher else ())
    time_ns = (
        _advance_to_dual_hold(runtime) if teacher else _advance_policy_left_phase(runtime)
    )
    left_proof = runtime.state.metrics["handover_sequence"]["left_stable_sim_time_ns"]
    runtime.tick(_dual_airborne(time_ns + 100_000_000), time_ns + 100_000_000)
    runtime.tick(_left_airborne(time_ns + 200_000_000), time_ns + 200_000_000)

    def regrasp(time_ns: int) -> HandoverObservation:
        return _airborne(
            time_ns, left_open=CLOSE, left_holding=True,
            right_open=CLOSE, right_holding=True,
            right_tcp_offset=(0.0, -0.05, 0.035),
        )

    resumed_ns = time_ns + 300_000_000
    state = runtime.tick(regrasp(resumed_ns), resumed_ns)
    state = runtime.tick(regrasp(resumed_ns + 499_000_000), resumed_ns + 499_000_000)
    assert state.phase == "RUNNING", state.metrics
    assert state.metrics["handover_sequence"]["dual_airborne_stable"] is False
    state = runtime.tick(regrasp(resumed_ns + 500_000_000), resumed_ns + 500_000_000)
    sequence = state.metrics["handover_sequence"]
    assert state.phase == "RUNNING", state.metrics
    assert sequence["dual_airborne_stable"] is True
    assert sequence["left_stable_sim_time_ns"] == left_proof
    assert sequence["dual_hold_elapsed_s"] == pytest.approx(TASK.gates.stable_duration_s)
    if teacher:
        assert runtime.step_name == "left_open", state.metrics


def test_left_candidate_regrasp_restarts_its_stable_window():
    runtime = _runtime(())
    _run(runtime, _left_airborne, 0.3, 0)
    runtime.tick(_observation(400_000_000, block_center=AIRBORNE_CENTER), 400_000_000)

    def regrasp(time_ns: int) -> HandoverObservation:
        return _airborne(
            time_ns, left_open=CLOSE, left_holding=True,
            left_tcp_offset=(0.0, 0.05, 0.035),
        )

    runtime.tick(regrasp(500_000_000), 500_000_000)
    state = runtime.tick(regrasp(999_000_000), 999_000_000)
    assert state.metrics["handover_sequence"]["left_airborne_stable"] is False
    state = runtime.tick(regrasp(1_000_000_000), 1_000_000_000)
    assert state.phase == "RUNNING", state.metrics
    assert state.metrics["handover_sequence"]["left_airborne_stable"] is True
    assert state.metrics["handover_sequence"]["left_hold_elapsed_s"] == pytest.approx(
        TASK.gates.stable_duration_s
    )


def test_teacher_grasp_candidate_restarts_after_regrasp():
    runtime = _runtime(_steps())
    time_ns = 0
    for _ in range(3):
        _state, time_ns = _run(runtime, _observation, 0.7, time_ns)
    assert runtime.behavior_phase == "left_grasp", runtime.state.metrics
    runtime.tick(
        _observation(time_ns + 100_000_000, left_open=CLOSE, left_holding=True,
                     supports=(TABLE_SUPPORT,)),
        time_ns + 100_000_000,
    )
    runtime.tick(_observation(time_ns + 200_000_000), time_ns + 200_000_000)

    def regrasp(time_ns: int) -> HandoverObservation:
        return _observation(
            time_ns, left_open=CLOSE, left_holding=True, supports=(TABLE_SUPPORT,),
            left_tcp_offset=(0.0, 0.05, 0.035),
        )

    state, _time_ns = _run(runtime, regrasp, 0.6, time_ns + 200_000_000)
    assert state.phase == "RUNNING", state.metrics
    assert runtime.step_name == "left_lift", state.metrics
    assert state.metrics["handover_sequence"]["left_airborne_stable"] is False


@pytest.mark.parametrize("side", ["left", "right"])
def test_candidate_reference_does_not_follow_continuous_slip(side):
    runtime = _runtime(())
    time_ns = _advance_policy_left_phase(runtime) if side == "right" else 0
    for index in range(1, 16):
        now_ns = time_ns + index * 100_000_000
        offsets = {
            f"{side}_tcp_offset": (
                0.0, (0.06 if side == "left" else -0.06) + index * 0.002, 0.035
            )
        }
        observation = _airborne(
            now_ns, left_open=CLOSE, left_holding=True,
            right_open=CLOSE if side == "right" else OPEN,
            right_holding=side == "right", **offsets,
        )
        state = runtime.tick(observation, now_ns)
        assert state.phase == "RUNNING", state.metrics
        assert state.metrics["handover_sequence"][
            "dual_airborne_stable" if side == "right" else "left_airborne_stable"
        ] is False


def test_repeated_candidate_regrasps_do_not_accumulate_hold_time():
    runtime = _runtime(())
    time_ns = 0
    for attempt in range(4):
        def candidate(
            time_ns: int, offset_m: float = 0.06 + attempt * 0.01
        ) -> HandoverObservation:
            return _airborne(
                time_ns, left_open=CLOSE, left_holding=True,
                left_tcp_offset=(0.0, offset_m, 0.035),
            )

        state, time_ns = _run(runtime, candidate, 0.3, time_ns)
        assert state.phase == "RUNNING", state.metrics
        assert state.metrics["handover_sequence"]["left_airborne_stable"] is False
        time_ns += 100_000_000
        state = runtime.tick(_observation(time_ns, block_center=AIRBORNE_CENTER), time_ns)
        assert state.metrics["handover_sequence"]["left_hold_elapsed_s"] == 0.0


def test_established_left_reference_is_preserved_after_a_short_loss():
    runtime = _runtime(())
    time_ns = _advance_policy_left_phase(runtime)
    left_proof = runtime.state.metrics["handover_sequence"]["left_stable_sim_time_ns"]
    _state, time_ns = _run(
        runtime, lambda t: _observation(t, block_center=AIRBORNE_CENTER), 0.2, time_ns
    )

    def slipped_grasp(time_ns: int) -> HandoverObservation:
        return _airborne(
            time_ns, left_open=CLOSE, left_holding=True,
            right_open=CLOSE, right_holding=True,
            left_tcp_offset=(0.0, 0.05, 0.035),
        )

    state, _time_ns = _run(runtime, slipped_grasp, 0.6, time_ns)
    assert state.phase == "RUNNING", state.metrics
    sequence = state.metrics["handover_sequence"]
    assert sequence["left_stable_sim_time_ns"] == left_proof
    assert sequence["dual_airborne_stable"] is False


def test_reset_clears_the_handover_proof():
    runtime = _runtime(())
    time_ns = _complete_airborne_sequence(runtime, 0)
    runtime.reset()
    metrics = runtime.state.metrics
    sequence = metrics["handover_sequence"]
    assert sequence["phase"] == "left"
    assert sequence["complete"] is False
    assert sequence["left_airborne_stable"] is False
    assert sequence["left_stable_sim_time_ns"] is None
    # The next task epoch starts without the previous proof.
    plan = _plan(())
    runtime.commit_prepare(
        request_id="prepare-2",
        content_digest="content-2",
        context=CONTEXT,
        plan=plan,
        plan_digest=plan.plan_digest,
        plan_duration_ns=plan.duration_ns,
    )
    runtime.start(
        request_id="start-2",
        content_digest="start-content-2",
        context=CONTEXT,
        plan_digest=plan.plan_digest,
        task_epoch=runtime.task_epoch,
        sim_time_ns=time_ns,
        wall_ns=0,
    )
    state, _time_ns = _run(runtime, _placed, 1.4, time_ns)
    assert state.phase == "RUNNING", state.metrics
    assert state.metrics["handover_sequence"]["complete"] is False


def test_robot_robot_contact_still_fails_the_task():
    runtime = _runtime(())
    state = runtime.tick(_observation(100_000_000, robot_robot_force_n=1.0), 100_000_000)
    assert state.phase == "FAILED", state.metrics
    assert state.failure_reason.startswith("FORBIDDEN_CONTACT")


def test_robot_box_contact_still_fails_the_task():
    runtime = _runtime(())
    state = runtime.tick(_observation(100_000_000, robot_box_force_n=1.0), 100_000_000)
    assert state.phase == "FAILED", state.metrics
    assert state.failure_reason.startswith("FORBIDDEN_CONTACT")


def test_block_below_the_workspace_still_fails_as_dropped():
    runtime = _runtime(())
    state = runtime.tick(
        _observation(100_000_000, block_center=(0.55, 0.12, 0.0)), 100_000_000
    )
    assert state.phase == "FAILED", state.metrics
    assert state.failure_reason.startswith("DROPPED")


def _advance_policy_left_phase(runtime: HandoverTaskRuntime) -> int:
    state, time_ns = _run(runtime, _left_airborne, 0.6, 0)
    assert state.phase == "RUNNING", state.metrics
    assert state.metrics["handover_sequence"]["left_airborne_stable"] is True
    return time_ns


def _complete_policy_dual_phase(runtime: HandoverTaskRuntime) -> int:
    time_ns = _advance_policy_left_phase(runtime)
    state, time_ns = _run(runtime, _dual_airborne, 0.6, time_ns)
    assert state.phase == "RUNNING", state.metrics
    assert state.metrics["handover_sequence"]["dual_airborne_stable"] is True
    return time_ns
