from __future__ import annotations

import sys
from pathlib import Path

import pytest
from model_helpers import replace

ROOT = Path(__file__).resolve().parents[2]
SCENARIO = ROOT / "shared/scenarios/marvin_sim/stack"
PROFILES = SCENARIO / "profiles"
SIM_SRC = ROOT / "simulation/mujoco/src"
CONTRACTS = ROOT / "shared/common/python/contracts/src"
for source in (SIM_SRC, CONTRACTS):
    if str(source) not in sys.path:
        sys.path.insert(0, str(source))

pytest.importorskip("mujoco")
pytest.importorskip("mink")
import numpy as np  # noqa: E402
from robot_backend_sim.marvin_sim.stack.kinematics_config import load_kinematics_config  # noqa: E402
from robot_backend_sim.common.config.motion_config import load_control_config  # noqa: E402
from robot_backend_sim.marvin_sim.stack.scene_config import load_marvin_stack_scene_config  # noqa: E402
from robot_backend_sim.common.kinematics.common import extract_object_pose  # noqa: E402
from robot_backend_sim.marvin_sim.stack.scene import build_marvin_stack_scene  # noqa: E402
from robot_backend_sim.marvin_sim.stack.config import load_task_config  # noqa: E402
from robot_backend_sim.marvin_sim.stack.gate import (  # noqa: E402
    TeacherFeedback,
    TeacherProgress,
)
from robot_backend_sim.marvin_sim.stack.teacher import (  # noqa: E402
    TeacherError,
    TeacherSnapshot,
    build_teacher_plan,
    sample_teacher_chunk,
)
from robot_contracts.models import RunContext  # noqa: E402
from robot_contracts.profiles import load_profile  # noqa: E402


@pytest.fixture(scope="module")
def fixture():
    scene_config = load_marvin_stack_scene_config(
        SCENARIO / "configs/simulation/scene/marvin_stack.yaml"
    )
    scene = build_marvin_stack_scene(
        scene_config,
    )
    scene.reset(0)
    task_config = load_task_config(SCENARIO / "configs/simulation/scene/marvin_stack_task.yaml")
    profile = load_profile(
        PROFILES / "robots/marvin_stack_v1/profile.json",
        ROOT / "shared/common/interfaces/schemas",
    )
    control = load_control_config(PROFILES / "control/marvin_control_v1.yaml")
    kinematics = load_kinematics_config(
        SCENARIO / "configs/simulation/checks/kinematics_check.yaml", scene_config
    )
    indices = scene.indices
    qpos = tuple(float(value) for value in scene.data.qpos)
    source = extract_object_pose(qpos, indices.environment.object_free_joint_qpos["source"], "source")
    snapshot = TeacherSnapshot(
        context=RunContext("robot", profile.digest, "backend", "time", 0, "episode", 1),
        task_epoch=1,
        sim_time_ns=0,
        qpos=qpos,
        seed=0,
    )
    return scene, task_config, profile, control, kinematics, snapshot, source


def test_plan_is_bounded_private_and_uses_all_phase_gates(fixture):
    scene, task_config, profile, control, kinematics, snapshot, _source = fixture
    before = np.asarray(scene.data.qpos, dtype=float).copy()
    plan = build_teacher_plan(
        scene.model,
        snapshot,
        profile,
        control,
        kinematics,
        task_config,
        scene_config=scene.config,
    )
    after = np.asarray(scene.data.qpos, dtype=float)
    assert np.array_equal(before, after)
    assert plan.duration_s < task_config.success.max_task_sim_duration_s
    assert plan.duration_s <= task_config.teacher.max_plan_duration_s + 1e-9
    assert [segment.phase for segment in plan.segments] == [
        "pregrasp",
        "grasp",
        "close",
        "lift",
        "transport",
        "lower",
        "open",
        "retreat",
        "stable",
    ]
    assert plan.segments[-1].duration_s >= task_config.success.stable_duration_s
    # 0.0514 m gripper travel at 0.04 m/s cannot be completed in one second by this
    # velocity-limited trapezoid curve.
    close = next(segment for segment in plan.segments if segment.phase == "close")
    assert close.duration_s >= 1.5
    # Home to a large-angle pregrasp is also velocity/acceleration limited.
    pregrasp = next(segment for segment in plan.segments if segment.phase == "pregrasp")
    assert pregrasp.duration_s >= 3.0
    open_segment = next(segment for segment in plan.segments if segment.phase == "open")
    assert open_segment.duration_s >= 1.5
    assert len(plan.ik_results) == 6
    assert all(result.converged for result in plan.ik_results)
    assert plan.collision_results
    assert all(result.passed for result in plan.collision_results)
    assert plan.allowed_contact_pairs == ("Arm_R10_Link:source", "Arm_R11_Link:source")
    assert (
        plan.plan_digest
        == build_teacher_plan(
            scene.model,
            snapshot,
            profile,
            control,
            kinematics,
            task_config,
            scene_config=scene.config,
        ).plan_digest
    )


def test_chunk_is_60x16_at_50ms_and_keeps_left_arm_fixed(fixture):
    scene, task_config, profile, control, kinematics, snapshot, source = fixture
    plan = build_teacher_plan(
        scene.model,
        snapshot,
        profile,
        control,
        kinematics,
        task_config,
        scene_config=scene.config,
    )
    progress = TeacherProgress(plan, task_config)
    progress.actual_start_ns = 1
    feedback = TeacherFeedback(
        capture_seq=1,
        sim_time_ns=0,
        right_positions=plan.start_positions[8:15],
        right_velocities=(0.0,) * 7,
        right_gripper_opening_m=0.0814,
        right_gripper_velocity_m_s=0.0,
        right_finger_source_contacts=0,
        right_finger_source_min_force_n=0.0,
        right_finger_source_max_speed_m_s=0.0,
        source_center=source.center,
        right_tcp_distance_m=0.5,
    )
    chunk = sample_teacher_chunk(
        plan,
        gate=progress.fact(),
        feedback=feedback,
        request_id="chunk-1",
        task_epoch=plan.task_epoch,
        capture_seq=1,
        observation_time_ns=0,
        chunk_size=60,
        period_ns=50_000_000,
        profile=profile,
        control_config=control,
    )
    assert len(chunk.positions) == 60
    assert all(len(position) == 16 for position in chunk.positions)
    assert chunk.time_from_observation_ns[0] == 50_000_000
    assert chunk.time_from_observation_ns[-1] == 3_000_000_000
    assert tuple(chunk.time_from_observation_ns[index] for index in range(60)) == tuple(
        (index + 1) * 50_000_000 for index in range(60)
    )
    for position in chunk.positions:
        assert position[:8] == plan.start_positions[:8]
        assert all(np.isfinite(position))
    # A later chunk cannot jump ahead of unpassed gates; it holds pregrasp C1.
    later = sample_teacher_chunk(
        plan,
        gate=progress.fact(),
        feedback=replace(feedback, capture_seq=20, sim_time_ns=20_000_000_000),
        request_id="chunk-2",
        task_epoch=plan.task_epoch,
        capture_seq=20,
        observation_time_ns=20_000_000_000,
        chunk_size=60,
        period_ns=50_000_000,
        profile=profile,
        control_config=control,
    )
    assert later.phase == "pregrasp"
    assert all(position == plan.segments[0].end_positions for position in later.positions)
    # Once the task reaches its stable phase, the final target stays bounded
    # while the independent evaluator finishes its 2 s window.
    progress.phase_index = len(plan.segments) - 1
    progress.actual_start_ns = 1
    stable = sample_teacher_chunk(
        plan,
        gate=progress.fact(),
        feedback=replace(feedback, capture_seq=21, sim_time_ns=25_000_000_000),
        request_id="chunk-3",
        task_epoch=plan.task_epoch,
        capture_seq=21,
        observation_time_ns=25_000_000_000,
        chunk_size=60,
        period_ns=50_000_000,
        profile=profile,
        control_config=control,
    )
    assert stable.phase == "stable"
    assert all(position == plan.segments[-1].end_positions for position in stable.positions)


def test_chunk_rejects_stale_or_future_observation(fixture):
    scene, task_config, profile, control, kinematics, snapshot, source = fixture
    plan = build_teacher_plan(
        scene.model,
        snapshot,
        profile,
        control,
        kinematics,
        task_config,
        scene_config=scene.config,
    )
    progress = TeacherProgress(plan, task_config)
    progress.actual_start_ns = 1
    feedback = TeacherFeedback(
        capture_seq=1,
        sim_time_ns=0,
        right_positions=plan.start_positions[8:15],
        right_velocities=(0.0,) * 7,
        right_gripper_opening_m=0.0814,
        right_gripper_velocity_m_s=0.0,
        right_finger_source_contacts=0,
        right_finger_source_min_force_n=0.0,
        right_finger_source_max_speed_m_s=0.0,
        source_center=source.center,
        right_tcp_distance_m=0.5,
    )
    with pytest.raises(TeacherError, match="TEACHER_FEEDBACK_MISMATCH"):
        sample_teacher_chunk(
            plan,
            gate=progress.fact(),
            feedback=feedback,
            request_id="chunk-1",
            task_epoch=1,
            capture_seq=2,
            observation_time_ns=0,
            chunk_size=60,
            period_ns=50_000_000,
            profile=profile,
            control_config=control,
        )
    progress.failure = "GATE_TIMEOUT:close"
    with pytest.raises(TeacherError, match="TEACHER_GATE_FAILED"):
        sample_teacher_chunk(
            plan,
            gate=progress.fact(),
            feedback=feedback,
            request_id="chunk-2",
            task_epoch=1,
            capture_seq=1,
            observation_time_ns=0,
            chunk_size=60,
            period_ns=50_000_000,
            profile=profile,
            control_config=control,
        )


def test_over_30s_plan_is_rejected_with_phase(fixture):
    scene, task_config, profile, control, kinematics, snapshot, _source = fixture
    strict = replace(
        task_config,
        teacher=replace(task_config.teacher, max_plan_duration_s=5.0),
    )
    with pytest.raises(TeacherError, match="TEACHER_PLAN_TOO_LONG"):
        build_teacher_plan(
            scene.model,
            snapshot,
            profile,
            control,
            kinematics,
            strict,
            scene_config=scene.config,
        )


def _feedback_for(
    plan,
    source_center,
    *,
    capture=1,
    sim_ns=0,
    right=None,
    contacts=0,
    force=0.0,
    finger_speed=0.0,
    gripper=0.0814,
    source_z=None,
    tcp_distance=0.5,
):
    return TeacherFeedback(
        capture_seq=capture,
        sim_time_ns=sim_ns,
        right_positions=tuple(plan.start_positions[8:15] if right is None else right),
        right_velocities=(0.0,) * 7,
        right_gripper_opening_m=gripper,
        right_gripper_velocity_m_s=0.0,
        right_finger_source_contacts=contacts,
        right_finger_source_min_force_n=force,
        right_finger_source_max_speed_m_s=finger_speed,
        source_center=(
            source_center
            if source_z is None
            else (source_center[0], source_center[1], source_z)
        ),
        right_tcp_distance_m=tcp_distance,
    )


def test_gate_requires_measured_arm_stability(fixture):
    scene, task_config, profile, control, kinematics, snapshot, source = fixture
    plan = build_teacher_plan(
        scene.model,
        snapshot,
        profile,
        control,
        kinematics,
        task_config,
        scene_config=scene.config,
    )
    progress = TeacherProgress(plan, task_config)
    progress.actual_start_ns = 1
    start = _feedback_for(plan, source.center, sim_ns=1_000_000_000)
    progress.update(start, "RUNNING")
    assert progress.phase_index == 0
    target = plan.segments[0].end_positions[8:15]
    end_ns = round(plan.segments[0].duration_s * 1e9) + 1
    progress.update(_feedback_for(plan, source.center, sim_ns=end_ns, right=target), "RUNNING")
    progress.update(
        _feedback_for(plan, source.center, sim_ns=end_ns + 100_000_000, right=target), "RUNNING"
    )
    assert progress.phase_index == 0
    progress.update(
        _feedback_for(plan, source.center, sim_ns=end_ns + 200_000_000, right=target), "RUNNING"
    )
    assert progress.phase_index == 1


def test_close_needs_two_finger_source_contact_before_lift(fixture):
    scene, task_config, profile, control, kinematics, snapshot, source = fixture
    plan = build_teacher_plan(
        scene.model,
        snapshot,
        profile,
        control,
        kinematics,
        task_config,
        scene_config=scene.config,
    )
    progress = TeacherProgress(plan, task_config)
    progress.phase_index = 2  # close
    progress.actual_start_ns = 1
    no_contact = _feedback_for(plan, source.center, sim_ns=4_000_000_000, contacts=0)
    for delta in (0, 100_000_000, 300_000_000):
        progress.update(replace(no_contact, sim_time_ns=4_000_000_000 + delta), "RUNNING")
    assert progress.phase_index == 2
    contact = _feedback_for(
        plan,
        source.center,
        sim_ns=5_000_000_000,
        contacts=2,
        force=1.0,
        finger_speed=0.001,
        gripper=0.04,
        right=plan.segments[2].end_positions[8:15],
    )
    progress.update(contact, "RUNNING")
    progress.update(replace(contact, sim_time_ns=5_100_000_000), "RUNNING")
    progress.update(replace(contact, sim_time_ns=5_200_000_000), "RUNNING")
    assert progress.phase_index == 3  # lift


def test_lift_requires_real_source_rise_and_grip(fixture):
    scene, task_config, profile, control, kinematics, snapshot, source = fixture
    plan = build_teacher_plan(
        scene.model,
        snapshot,
        profile,
        control,
        kinematics,
        task_config,
        scene_config=scene.config,
    )
    progress = TeacherProgress(plan, task_config)
    progress.phase_index = 3
    progress.actual_start_ns = 6_000_000_000
    low = _feedback_for(plan, source.center, sim_ns=6_000_000_000, contacts=2, force=1.0)
    progress.update(low, "RUNNING")
    progress.update(replace(low, sim_time_ns=6_200_000_000), "RUNNING")
    assert progress.phase_index == 3
    end_ns = progress.actual_start_ns + round(plan.segments[3].duration_s * 1e9)
    raised = _feedback_for(
        plan,
        source.center,
        sim_ns=end_ns,
        contacts=2,
        force=1.0,
        source_z=source.center[2] + task_config.teacher.lift_height_m + 0.01,
        right=plan.segments[3].end_positions[8:15],
    )
    progress.update(raised, "RUNNING")
    progress.update(replace(raised, sim_time_ns=end_ns + 200_000_000), "RUNNING")
    assert progress.phase_index == 4


def test_gate_timeout_is_bounded_and_clear(fixture):
    scene, task_config, profile, control, kinematics, snapshot, source = fixture
    plan = build_teacher_plan(
        scene.model,
        snapshot,
        profile,
        control,
        kinematics,
        task_config,
        scene_config=scene.config,
    )
    progress = TeacherProgress(plan, task_config)
    progress.actual_start_ns = 1
    late = _feedback_for(plan, source.center, sim_ns=30_000_000_000)
    fact = progress.update(late, "RUNNING")
    assert fact.status == "FAILED"
    assert fact.failure.startswith("GATE_TIMEOUT:pregrasp")


def test_contact_cannot_cut_the_raw_closing_ramp_short(fixture):
    scene, task_config, profile, control, kinematics, snapshot, source = fixture
    plan = build_teacher_plan(
        scene.model,
        snapshot,
        profile,
        control,
        kinematics,
        task_config,
        scene_config=scene.config,
    )
    progress = TeacherProgress(plan, task_config)
    progress.phase_index = 2
    progress.actual_start_ns = 0
    segment = plan.segments[2]
    duration_ns = round(segment.duration_s * 1e9)
    feedback = _feedback_for(
        plan,
        source.center,
        sim_ns=duration_ns // 2,
        contacts=2,
        force=1.0,
        finger_speed=0.001,
        gripper=0.04,
        right=segment.end_positions[8:15],
    )
    progress.update(feedback, "RUNNING")
    progress.update(replace(feedback, sim_time_ns=feedback.sim_time_ns + 250_000_000), "RUNNING")
    assert progress.phase_index == 2
    progress.update(replace(feedback, sim_time_ns=duration_ns), "RUNNING")
    assert progress.phase_index == 3
    assert plan.segments[3].start_positions == segment.end_positions


def test_handoff_start_sim_time_ns_holds_start_until_handoff_and_binds_gate(fixture):
    scene, task_config, profile, control, kinematics, snapshot, source = fixture
    plan = build_teacher_plan(
        scene.model,
        snapshot,
        profile,
        control,
        kinematics,
        task_config,
        scene_config=scene.config,
    )
    progress = TeacherProgress(plan, task_config)
    progress.actual_start_ns = 1_000_000_000
    progress.actual_starts_ns.append(1_000_000_000)
    handoff_ns = 1_500_000_000
    progress.bind_motion_start(0, handoff_ns)
    assert progress.actual_start_ns == handoff_ns
    feedback = _feedback_for(plan, source.center, capture=21, sim_ns=1_000_000_000)
    chunk = sample_teacher_chunk(
        plan,
        gate=progress.fact(),
        feedback=feedback,
        request_id="chunk-handoff",
        task_epoch=plan.task_epoch,
        capture_seq=21,
        observation_time_ns=1_000_000_000,
        chunk_size=60,
        period_ns=50_000_000,
        profile=profile,
        control_config=control,
        start_sim_time_ns=handoff_ns,
    )
    # Points at or before handoff_ns (indices 0..9 -> 1.05s..1.50s) hold start_positions;
    # the first selected point after handoff_ns (index 10 -> 1.55s) begins moving smoothly.
    assert chunk.phase_start_sim_time_ns == handoff_ns
    assert chunk.positions[9] == plan.segments[0].start_positions
    assert chunk.positions[10] != plan.segments[0].start_positions
