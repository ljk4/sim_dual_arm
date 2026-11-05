from __future__ import annotations

import math
import sys
import threading
from dataclasses import replace
from pathlib import Path

import pytest

mujoco = pytest.importorskip("mujoco")
pytest.importorskip("scipy")
np = pytest.importorskip("numpy")

ROOT = Path(__file__).resolve().parents[2]
for source in (ROOT / "simulation/mujoco/src", ROOT / "contracts/python/src"):
    if str(source) not in sys.path:
        sys.path.insert(0, str(source))

import robot_backend_sim.motion as motion_module  # noqa: E402
from robot_backend_sim.contacts import (  # noqa: E402
    measure_contact_evidence,
    resolve_contact_binding,
)
from robot_backend_sim.motion import (  # noqa: E402
    FailedReason,
    NativeMotion,
    Phase,
    StopReason,
    prepare_motion,
)
from robot_backend_sim.motion_config import (  # noqa: E402
    CONTROL_JOINT_NAMES,
    load_control_config,
)
from robot_backend_sim.panda_io import PandaIO  # noqa: E402
from robot_backend_sim.scene import build_scene  # noqa: E402
from robot_backend_sim.scene_config import load_scene_config  # noqa: E402
from robot_contracts.models import ContractError, Target  # noqa: E402
from robot_contracts.profiles import load_profile  # noqa: E402

CONFIG_PATH = ROOT / "simulation/mujoco/configs/scene/panda_stack.yaml"
CONTROL_PATH = ROOT / "simulation/mujoco/configs/control/panda_control.yaml"
PROFILE_PATH = ROOT / "profiles/robots/panda_stack_v1/profile.json"
SCHEMAS = ROOT / "contracts/schemas"
DT_NS = 2_000_000
TERMINAL = frozenset({Phase.COMPLETED, Phase.STOPPED, Phase.FAILED})


@pytest.fixture(scope="module")
def scene_config():
    return load_scene_config(CONFIG_PATH)


@pytest.fixture(scope="module")
def scene(scene_config):
    return build_scene(
        scene_config,
    )


@pytest.fixture(scope="module")
def other_scene(scene_config):
    return build_scene(
        scene_config,
    )


@pytest.fixture(scope="module")
def profile():
    return load_profile(PROFILE_PATH, SCHEMAS)


@pytest.fixture(scope="module")
def control_config():
    return load_control_config(CONTROL_PATH)


def fresh_io(scene, profile) -> PandaIO:
    scene.reset(0)
    return PandaIO(scene, profile)


def run_in_thread(function):
    outcome = []

    def target():
        try:
            outcome.append(function())
        except BaseException as error:  # keep the worker exception for the assertion
            outcome.append(error)

    thread = threading.Thread(target=target)
    thread.start()
    thread.join()
    return outcome[0]


def drive(motion: NativeMotion, start_wall_ns: int, steps: int):
    """Step at the scene's 2 ms cadence and stop early only on a terminal phase."""
    wall = start_wall_ns
    snapshot = motion.step(wall)
    for index in range(1, steps):
        if snapshot.phase in TERMINAL:
            break
        wall = start_wall_ns + index * DT_NS
        snapshot = motion.step(wall)
    return snapshot, wall


def target_at(base: tuple[float, ...], duration_ns: int, **changes: float) -> tuple[Target, ...]:
    positions = list(base)
    for name, value in changes.items():
        positions[CONTROL_JOINT_NAMES.index(name)] = value
    return (Target(duration_ns, tuple(positions)),)


def expect_reason(function, reason: str):
    with pytest.raises(ContractError) as caught:
        function()
    assert caught.value.reason == reason
    return caught.value


def install_contact_source(scene) -> None:
    """One pre-run fixture: centre source at the home right TCP, between 0.02 m fingers."""
    model = scene.model
    data = scene.data
    mujoco.mj_forward(model, data)
    tcp = int(scene.indices.tcp_site_ids["right_tcp"])
    position = np.asarray(data.site_xpos[tcp], dtype=float)
    matrix = np.asarray(data.site_xmat[tcp], dtype=float)
    quaternion = np.zeros(4, dtype=float)
    mujoco.mju_mat2Quat(quaternion, matrix)
    address = int(scene.indices.object_free_joint_qpos["source"])
    data.qpos[address : address + 7] = (*position, *quaternion)
    free_dof = int(model.joint("source_free").dofadr[0])
    data.qvel[free_dof : free_dof + 6] = 0.0
    for name in ("right_finger_joint1", "right_finger_joint2"):
        joint = model.joint(name)
        data.qpos[int(joint.qposadr[0])] = 0.02
        data.qvel[int(joint.dofadr[0])] = 0.0
    mujoco.mj_forward(model, data)


def test_contact_precheck_reports_expected_opening_and_per_finger_force(
    scene, profile, control_config
):
    io = fresh_io(scene, profile)
    install_contact_source(scene)
    binding = resolve_contact_binding(scene.model, tuple(obj.name for obj in scene.config.objects))
    targets = list(io.measure().positions)
    targets[15] = 0.0
    for _ in range(500):
        io.apply_targets(targets)
        mujoco.mj_step(scene.model, scene.data)
        mujoco.mj_forward(scene.model, scene.data)

    measurement = io.measure()
    assert measurement.positions[15] == pytest.approx(0.03985, abs=0.0015)
    evidence = measure_contact_evidence(
        scene.model,
        scene.data,
        binding,
        "right",
    )
    assert evidence.candidate_object == "source"
    assert evidence.ambiguous is False
    for finger in evidence.fingers:
        assert finger.force_for("source") == pytest.approx(1.0, abs=0.35)
    assert all(int(warning.number) == 0 for warning in scene.data.warning)


def test_positive_contact_completion_keeps_original_zero_target(scene, profile, control_config):
    io = fresh_io(scene, profile)
    install_contact_source(scene)
    close = list(io.measure().positions)
    close[15] = 0.0
    for _ in range(500):
        io.apply_targets(close)
        mujoco.mj_step(scene.model, scene.data)
        mujoco.mj_forward(scene.model, scene.data)
    measured_contact = io.measure()
    assert measured_contact.positions[15] == pytest.approx(0.03985, abs=0.0015)

    initial = list(profile.initial_positions)
    initial[15] = 0.0
    targets = target_at(tuple(initial), 400_000_000)
    prepared = prepare_motion(tuple(initial), targets, profile, control_config)
    motion = NativeMotion(scene, io, control_config, initial_reference=tuple(initial))
    motion.start(prepared, 0)
    snapshot, _wall = drive(motion, 0, 500)

    result = motion.result
    assert result is not None
    assert result.phase is Phase.COMPLETED
    assert result.started_sim_ns == pytest.approx(1_000_000_000, abs=1)
    assert snapshot.phase is Phase.COMPLETED
    right = snapshot.grippers[1]
    assert right.mode == "CONTACT"
    assert right.contact_object == "source"
    assert right.final_reference_m == 0.0
    assert snapshot.reference_positions[15] == pytest.approx(0.0, abs=1e-9)
    assert right.target_error_m < -control_config.position_tolerances[15]
    assert right.target_error_m == pytest.approx(-0.03985, abs=0.002)
    assert snapshot.grippers[0].mode == "POSITION"
    for finger in right.fingers:
        assert finger.force_for("source") >= control_config.contact_force_min_n
        assert finger.joint_speed_m_s <= control_config.finger_velocity_max_m_s


def test_real_no_contact_gripper_completes_in_position_mode_not_contact(
    scene, profile, control_config
):
    io = fresh_io(scene, profile)
    initial = tuple(profile.initial_positions)
    targets = target_at(initial, 3_000_000_000, right_gripper_width=0.0)
    prepared = prepare_motion(initial, targets, profile, control_config)
    motion = NativeMotion(scene, io, control_config, initial_reference=initial)
    motion.start(prepared, 0)
    snapshot, _wall = drive(motion, 0, 2000)
    assert snapshot.phase is Phase.COMPLETED
    assert snapshot.grippers[1].mode == "POSITION"
    assert snapshot.grippers[1].contact_object is None


def test_small_right_arm_and_gripper_motion_holds_left_and_completes(
    scene, profile, control_config
):
    io = fresh_io(scene, profile)
    initial = tuple(profile.initial_positions)
    targets = target_at(
        initial,
        1_600_000_000,
        right_joint2=initial[9] + 0.04,
        right_gripper_width=0.04,
    )
    prepared = prepare_motion(initial, targets, profile, control_config)
    motion = NativeMotion(scene, io, control_config, initial_reference=initial)
    motion.start(prepared, 0)
    at_horizon = None
    snapshot = None
    wall = 0
    for index in range(1100):
        wall = index * DT_NS
        snapshot = motion.step(wall)
        if index == 800:
            at_horizon = snapshot
        if snapshot.phase in TERMINAL:
            break
    assert at_horizon is not None
    assert at_horizon.phase is Phase.SETTLING
    assert motion.result is not None and motion.result.phase is Phase.COMPLETED
    assert snapshot is not None and snapshot.phase is Phase.COMPLETED
    assert snapshot.grippers[1].mode == "POSITION"
    assert snapshot.measured_positions[9] > initial[9] + 0.02
    moving_target = targets[-1].positions[9]
    assert (
        abs(snapshot.measured_positions[9] - moving_target)
        <= (control_config.position_tolerances[9])
    )
    assert abs(snapshot.measured_velocities[9]) <= control_config.velocity_tolerances[9]
    for index in (0, 1, 2, 3, 4, 5, 6, 8, 10, 11, 12, 13, 14):
        assert (
            abs(snapshot.measured_positions[index] - initial[index])
            <= (control_config.position_tolerances[index])
        )
        assert (
            abs(snapshot.measured_velocities[index]) <= (control_config.velocity_tolerances[index])
        )
    assert snapshot.measured_positions[7] == pytest.approx(0.08, abs=0.002)
    for index in range(16):
        assert snapshot.tracking_errors[index] == pytest.approx(
            snapshot.reference_positions[index] - snapshot.measured_positions[index]
        )


def test_two_non_home_motions_start_from_the_previous_frozen_hold(scene, profile, control_config):
    io = fresh_io(scene, profile)
    initial = tuple(profile.initial_positions)
    io.apply_targets(initial)
    for _ in range(200):
        mujoco.mj_step(scene.model, scene.data)
        mujoco.mj_forward(scene.model, scene.data)
    first_targets = target_at(
        initial,
        800_000_000,
        right_joint2=initial[9] + 0.02,
        right_gripper_width=0.06,
    )
    first = prepare_motion(initial, first_targets, profile, control_config)
    motion = NativeMotion(scene, io, control_config, initial_reference=initial)
    motion.start(first, 0)
    first_snapshot, wall = drive(motion, 0, 1000)
    assert first_snapshot.phase is Phase.COMPLETED
    assert motion.result is not None
    assert motion.result.started_sim_ns == pytest.approx(400_000_000, abs=1)

    second_targets = target_at(
        first.final_positions,
        800_000_000,
        right_joint2=first.final_positions[9] + 0.03,
        right_gripper_width=0.04,
    )
    second = prepare_motion(first.final_positions, second_targets, profile, control_config)
    start_wall = wall + DT_NS
    accepted = motion.start(second, start_wall)
    assert accepted.phase is Phase.ACCEPTED
    assert accepted.reference_positions == first.final_positions
    assert motion.result is None
    second_snapshot, _wall = drive(motion, start_wall, 1100)
    assert second_snapshot.phase is Phase.COMPLETED
    assert second_snapshot.reference_elapsed_ns >= second.duration_ns


def test_motion_braking_is_continuous_and_stops_on_measured_velocity(
    scene, profile, control_config
):
    io = fresh_io(scene, profile)
    initial = tuple(profile.initial_positions)
    targets = target_at(initial, 3_000_000_000, right_joint2=initial[9] + 0.1)
    prepared = prepare_motion(initial, targets, profile, control_config)
    motion = NativeMotion(scene, io, control_config, initial_reference=initial)
    motion.start(prepared, 0)
    before_snapshot, wall = drive(motion, 0, 300)
    assert before_snapshot.phase is Phase.RUNNING
    assert abs(before_snapshot.reference_velocities[9]) > 0.0

    stopping = motion.request_stop("TEST_BRAKE", wall)
    assert stopping.phase is Phase.STOPPING
    first_stop = motion.step(wall)
    assert first_stop.phase is Phase.STOPPING
    elapsed_ns = (
        first_stop.reference_application_sim_time_ns
        - before_snapshot.reference_application_sim_time_ns
    )
    assert elapsed_ns == DT_NS
    assert first_stop.reference_elapsed_ns == elapsed_ns
    # Closed-form p/v at the next actual reference timestamp. Comparing it
    # with the previous p/v directly would incorrectly require a repeated tick.
    for index, (q0, v0, limit) in enumerate(
        zip(
            before_snapshot.reference_positions,
            before_snapshot.reference_velocities,
            control_config.max_accelerations,
            strict=True,
        )
    ):
        if v0 == 0.0:
            expected_q, expected_v = q0, 0.0
        else:
            stop_ns = max(1, math.ceil(abs(v0) / limit * 1_000_000_000))
            duration = stop_ns / 1_000_000_000
            elapsed = min(elapsed_ns, stop_ns) / 1_000_000_000
            expected_q = q0 + v0 * elapsed - v0 * elapsed**2 / (2.0 * duration)
            expected_v = v0 * (1.0 - elapsed / duration)
        assert first_stop.reference_positions[index] == pytest.approx(expected_q, abs=1e-12)
        assert first_stop.reference_velocities[index] == pytest.approx(expected_v, abs=1e-12)

    snapshots = [first_stop]
    wall += DT_NS
    for _ in range(400):
        snapshot = motion.step(wall)
        snapshots.append(snapshot)
        if snapshot.phase is Phase.STOPPED:
            break
        wall += DT_NS
    assert motion.result is not None
    assert motion.result.phase is Phase.STOPPED
    assert motion.result.reason == "TEST_BRAKE"
    final = snapshots[-1]
    for index, tolerance in enumerate(control_config.velocity_tolerances):
        assert abs(final.measured_velocities[index]) <= tolerance
    for previous, current in zip(snapshots, snapshots[1:], strict=False):
        for index, acceleration in enumerate(control_config.max_accelerations):
            delta = abs(current.reference_velocities[index] - previous.reference_velocities[index])
            assert delta <= acceleration * (DT_NS / 1_000_000_000) + 1e-9


def test_stop_before_first_step_uses_hold_path(scene, profile, control_config):
    io = fresh_io(scene, profile)
    initial = tuple(profile.initial_positions)
    targets = target_at(initial, 400_000_000, right_joint2=initial[9] + 0.005)
    prepared = prepare_motion(initial, targets, profile, control_config)
    motion = NativeMotion(scene, io, control_config, initial_reference=initial)
    motion.start(prepared, 0)
    assert motion.request_stop(StopReason.STOP_REQUESTED.value, 0).phase is Phase.STOPPING
    snapshot, _wall = drive(motion, 0, 200)
    assert snapshot.phase is Phase.STOPPED
    assert snapshot.reference_positions == pytest.approx(initial)
    assert motion.result is not None
    assert motion.result.started_wall_ns == 0
    assert motion.result.started_sim_ns == 0


def test_settling_cancel_brakes_and_stops_without_original_endpoint(scene, profile, control_config):
    io = fresh_io(scene, profile)
    initial = tuple(profile.initial_positions)
    targets = target_at(initial, 400_000_000, right_joint2=initial[9] + 0.005)
    prepared = prepare_motion(initial, targets, profile, control_config)
    motion = NativeMotion(scene, io, control_config, initial_reference=initial)
    motion.start(prepared, 0)
    settling = None
    for index in range(300):
        snapshot = motion.step(index * DT_NS)
        if snapshot.phase is Phase.SETTLING:
            settling = snapshot
            wall = index * DT_NS
            break
    assert settling is not None
    stopped_request = motion.request_stop(StopReason.STOP_REQUESTED.value, wall)
    assert stopped_request.phase is Phase.STOPPING
    final, _wall = drive(motion, wall, 250)
    assert final.phase is Phase.STOPPED
    assert motion.result is not None and motion.result.phase is Phase.STOPPED


def test_repeated_stop_does_not_extend_deadline_and_stop_timeout_latches(
    scene, profile, control_config
):
    io = fresh_io(scene, profile)
    initial = tuple(profile.initial_positions)
    targets = target_at(initial, 400_000_000, right_joint2=initial[9] + 0.005)
    prepared = prepare_motion(initial, targets, profile, control_config)
    motion = NativeMotion(scene, io, control_config, initial_reference=initial)
    motion.start(prepared, 0)
    motion.step(0)
    motion.request_stop("STOP", 500_000_000)
    motion.request_stop("STOP-AGAIN", 2_900_000_000)
    terminal = motion.poll(3_600_000_000)
    assert terminal.phase is Phase.FAILED
    assert terminal.reason == FailedReason.STOP_TIMEOUT.value
    # The brake was only planned, never applied: report the real prior kind.
    assert terminal.reference_kind == "TRAJECTORY"
    expect_reason(
        lambda: motion.start(prepared, 3_700_000_000),
        "FAILED_LATCHED",
    )


def test_poll_triggers_stopping_on_wall_deadline_then_stop_timeout(scene, profile, control_config):
    io = fresh_io(scene, profile)
    initial = tuple(profile.initial_positions)
    targets = target_at(initial, 400_000_000, right_joint2=initial[9] + 0.005)
    prepared = prepare_motion(initial, targets, profile, control_config)
    motion = NativeMotion(scene, io, control_config, initial_reference=initial)
    motion.start(prepared, 0)
    motion.step(0)
    stopping = motion.poll(10_000_000_000)
    assert stopping.phase is Phase.STOPPING
    assert stopping.reason == StopReason.WALL_TIMEOUT.value
    terminal = motion.poll(13_000_000_000)
    assert terminal.phase is Phase.FAILED
    assert terminal.reason == FailedReason.STOP_TIMEOUT.value


def test_stop_plan_rejection_latches_and_keeps_last_applied_reference(
    scene, profile, control_config, monkeypatch
):
    io = fresh_io(scene, profile)
    initial = tuple(profile.initial_positions)
    targets = target_at(initial, 600_000_000, right_joint2=initial[9] + 0.01)
    prepared = prepare_motion(initial, targets, profile, control_config)
    motion = NativeMotion(scene, io, control_config, initial_reference=initial)
    motion.start(prepared, 0)
    last = motion.step(0)

    def reject(*_args, **_kwargs):
        raise ContractError("INVALID_BRAKING_INPUT", "forced invalid brake")

    monkeypatch.setattr(motion_module, "plan_braking", reject)
    terminal = motion.request_stop("STOP", DT_NS)
    assert terminal.phase is Phase.FAILED
    assert terminal.reason == FailedReason.STOP_PLAN_REJECTED.value
    assert terminal.reference_positions == pytest.approx(last.reference_positions)
    expect_reason(lambda: motion.start(prepared, 2 * DT_NS), "FAILED_LATCHED")


def test_terminal_snapshot_is_frozen_while_hold_step_still_steps_physics(
    scene, profile, control_config
):
    io = fresh_io(scene, profile)
    initial = tuple(profile.initial_positions)
    targets = target_at(initial, 400_000_000, right_joint2=initial[9] + 0.005)
    prepared = prepare_motion(initial, targets, profile, control_config)
    motion = NativeMotion(scene, io, control_config, initial_reference=initial)
    motion.start(prepared, 0)
    terminal, wall = drive(motion, 0, 500)
    assert terminal.phase is Phase.COMPLETED
    result = motion.result
    frozen = motion.terminal_snapshot
    assert result is not None and frozen is not None
    time_before = float(scene.data.time)

    held = motion.step(wall + DT_NS)
    assert held is not frozen
    assert motion.result is result
    assert motion.terminal_snapshot is frozen
    assert held.phase is terminal.phase
    assert held.reason == terminal.reason
    assert held.measurement_sim_time_ns > terminal.measurement_sim_time_ns
    assert float(scene.data.time) > time_before


def test_all_running_entries_reject_other_threads(scene, profile, control_config):
    io = fresh_io(scene, profile)
    initial = tuple(profile.initial_positions)
    targets = target_at(initial, 400_000_000, right_joint2=initial[9] + 0.005)
    prepared = prepare_motion(initial, targets, profile, control_config)
    motion = NativeMotion(scene, io, control_config, initial_reference=initial)
    ctrl_before = scene.data.ctrl.copy()

    def run_in_thread(function):
        outcome = []

        def target():
            try:
                outcome.append(function())
            except BaseException as error:  # keep the worker exception for the assertion
                outcome.append(error)

        thread = threading.Thread(target=target)
        thread.start()
        thread.join()
        return outcome[0]

    calls = (
        lambda: motion.start(prepared, 0),
        lambda: motion.step(0),
        lambda: motion.poll(0),
        lambda: motion.request_stop("STOP", 0),
        lambda: motion.result,
        lambda: motion.terminal_snapshot,
        lambda: motion.phase,
    )
    for call in calls:
        error = run_in_thread(call)
        assert isinstance(error, ContractError)
        assert error.reason == "WRONG_THREAD"
    np.testing.assert_array_equal(scene.data.ctrl, ctrl_before)


def test_wall_time_inputs_reject_bool_negative_non_integer_and_regression(
    scene, profile, control_config
):
    io = fresh_io(scene, profile)
    initial = tuple(profile.initial_positions)
    targets = target_at(initial, 400_000_000, right_joint2=initial[9] + 0.005)
    prepared = prepare_motion(initial, targets, profile, control_config)
    motion = NativeMotion(scene, io, control_config, initial_reference=initial)

    for call in (
        lambda: motion.start(prepared, True),
        lambda: motion.start(prepared, -1),
        lambda: motion.step(True),
        lambda: motion.step(-1),
        lambda: motion.step(1.5),
        lambda: motion.poll(True),
        lambda: motion.request_stop("STOP", -1),
    ):
        expect_reason(call, "INVALID_WALL_TIME")

    motion.start(prepared, 0)
    motion.step(0)
    motion.step(2 * DT_NS)
    expect_reason(lambda: motion.step(DT_NS), "WALL_TIME_REGRESSION")


def test_settle_timeout_enters_braking_and_latches_tracking_timeout(
    scene, profile, control_config, monkeypatch
):
    io = fresh_io(scene, profile)
    initial = tuple(profile.initial_positions)
    targets = target_at(initial, 400_000_000, right_joint2=initial[9] + 0.005)
    prepared = prepare_motion(initial, targets, profile, control_config)
    motion = NativeMotion(scene, io, control_config, initial_reference=initial)
    original = motion_module.NativeMotion._completions

    def forced_none(self, measurement, contacts):
        return tuple(
            replace(completion, mode="NONE", position_ok=False, contact_ok=False)
            for completion in original(self, measurement, contacts)
        )

    monkeypatch.setattr(motion_module.NativeMotion, "_completions", forced_none)
    motion.start(prepared, 0)
    entered_stopping = None
    terminal = None
    for index in range(1600):
        snapshot = motion.step(index * DT_NS)
        if snapshot.phase is Phase.STOPPING and entered_stopping is None:
            entered_stopping = snapshot
        if snapshot.phase in TERMINAL:
            terminal = snapshot
            break
    assert entered_stopping is not None
    assert terminal is not None
    assert terminal.phase is Phase.FAILED
    assert terminal.reason == FailedReason.TRACKING_TIMEOUT.value
    assert motion.result is not None
    assert motion.result.stop_reason == StopReason.TRACKING_TIMEOUT.value


def test_idle_step_holds_and_advances_real_physics_without_starting(scene, profile, control_config):
    io = fresh_io(scene, profile)
    motion = NativeMotion(scene, io, control_config)
    initial = tuple(profile.initial_positions)
    polled = motion.poll(0)
    assert polled.phase is Phase.IDLE
    live = motion.step(0)
    assert live.phase is Phase.IDLE
    assert live.reference_positions == pytest.approx(initial)
    assert live.measurement_sim_time_ns == DT_NS
    assert live.reference_elapsed_ns == 0
    assert motion.result is None
    assert live.started_sim_ns is None


def test_reference_elapsed_depends_on_simulation_not_wall_time(scene, profile, control_config):
    io = fresh_io(scene, profile)
    initial = tuple(profile.initial_positions)
    targets = target_at(initial, 400_000_000, right_joint2=initial[9] + 0.005)
    prepared = prepare_motion(initial, targets, profile, control_config)
    motion = NativeMotion(scene, io, control_config, initial_reference=initial)
    motion.start(prepared, 0)
    first = motion.step(0)
    same_wall = motion.step(0)
    jumped_wall = motion.step(1_000_000_000)
    assert first.reference_elapsed_ns == 0
    assert same_wall.reference_elapsed_ns == DT_NS
    assert jumped_wall.reference_elapsed_ns == 2 * DT_NS
    assert same_wall.reference_application_sim_time_ns == DT_NS
    assert jumped_wall.reference_application_sim_time_ns == 2 * DT_NS
    assert jumped_wall.measurement_sim_time_ns == 3 * DT_NS


def test_scene_io_identity_and_creator_thread_are_enforced(
    scene, other_scene, profile, control_config
):
    io = fresh_io(scene, profile)
    io.require_scene(scene)
    expect_reason(lambda: io.require_scene(other_scene), "SCENE_MISMATCH")
    expect_reason(
        lambda: NativeMotion(other_scene, io, control_config),
        "SCENE_MISMATCH",
    )

    thread_error = run_in_thread(lambda: io.require_scene(scene))
    assert isinstance(thread_error, ContractError)
    assert thread_error.reason == "WRONG_THREAD"
    error = run_in_thread(lambda: NativeMotion(scene, io, control_config))
    assert isinstance(error, ContractError)
    assert error.reason == "WRONG_THREAD"


def test_stop_timeout_holds_last_applied_reference_not_unexecuted_brake_end(
    scene, profile, control_config
):
    io = fresh_io(scene, profile)
    initial = tuple(profile.initial_positions)
    targets = target_at(initial, 3_000_000_000, right_joint2=initial[9] + 0.1)
    prepared = prepare_motion(initial, targets, profile, control_config)
    motion = NativeMotion(scene, io, control_config, initial_reference=initial)
    motion.start(prepared, 0)
    last, _wall = drive(motion, 0, 300)
    assert last.phase is Phase.RUNNING
    assert abs(last.reference_velocities[9]) > 0.0
    ctrl_before = scene.data.ctrl.copy()
    stopping = motion.request_stop("MANUAL_BRAKE", 600_000_000)
    assert stopping.phase is Phase.STOPPING
    terminal = motion.poll(3_600_000_000)
    assert terminal.phase is Phase.FAILED
    assert terminal.reason == FailedReason.STOP_TIMEOUT.value
    assert terminal.reference_kind == "TRAJECTORY"
    assert motion.result is not None
    assert motion.result.stop_reason == "MANUAL_BRAKE"

    live = motion.step(3_600_000_000)
    assert live.phase is Phase.FAILED
    assert live.measurement_sim_time_ns > terminal.measurement_sim_time_ns
    np.testing.assert_array_equal(scene.data.ctrl, ctrl_before)


def test_stop_timeout_reports_brake_only_after_one_was_actually_applied(
    scene, profile, control_config
):
    io = fresh_io(scene, profile)
    initial = tuple(profile.initial_positions)
    targets = target_at(initial, 3_000_000_000, right_joint2=initial[9] + 0.1)
    prepared = prepare_motion(initial, targets, profile, control_config)
    motion = NativeMotion(scene, io, control_config, initial_reference=initial)
    motion.start(prepared, 0)
    running, _wall = drive(motion, 0, 300)
    assert running.phase is Phase.RUNNING
    stop_wall = 600_000_000
    stopping = motion.request_stop("MANUAL_BRAKE", stop_wall)
    assert stopping.phase is Phase.STOPPING
    applied = motion.step(stop_wall + DT_NS)
    assert applied.phase is Phase.STOPPING
    assert applied.reference_kind == "BRAKE"
    terminal = motion.poll(stop_wall + DT_NS + 3_100_000_000)
    assert terminal.phase is Phase.FAILED
    assert terminal.reason == FailedReason.STOP_TIMEOUT.value
    assert terminal.reference_kind == "BRAKE"


def test_late_manual_stop_does_not_replace_wall_timeout(scene, profile, control_config):
    io = fresh_io(scene, profile)
    initial = tuple(profile.initial_positions)
    targets = target_at(initial, 400_000_000, right_joint2=initial[9] + 0.005)
    prepared = prepare_motion(initial, targets, profile, control_config)
    motion = NativeMotion(scene, io, control_config, initial_reference=initial)
    motion.start(prepared, 0)
    motion.step(0)
    stopping = motion.request_stop("LATE_MANUAL", 10_000_000_000)
    assert stopping.phase is Phase.STOPPING
    assert stopping.reason == StopReason.WALL_TIMEOUT.value
    terminal, _wall = drive(motion, 10_000_000_000, 250)
    assert terminal.phase is Phase.FAILED
    assert terminal.reason == FailedReason.WALL_TIMEOUT.value
    assert motion.result is not None
    assert motion.result.stop_reason == StopReason.WALL_TIMEOUT.value


def test_independent_raw_actuator_step_forward_replay_matches_qpos_qvel(
    scene, profile, control_config
):
    io = fresh_io(scene, profile)
    initial = tuple(profile.initial_positions)
    targets = target_at(
        initial,
        1_000_000_000,
        right_joint2=initial[9] + 0.02,
        right_gripper_width=0.06,
    )
    prepared = prepare_motion(initial, targets, profile, control_config)
    motion = NativeMotion(scene, io, control_config, initial_reference=initial)
    motion.start(prepared, 0)
    references = []
    snapshot = None
    for index in range(100):
        snapshot = motion.step(index * DT_NS)
        references.append(snapshot.reference_positions)
    expected_qpos = scene.data.qpos.copy()
    expected_qvel = scene.data.qvel.copy()

    scene.reset(0)
    replay_io = PandaIO(scene, profile)
    for positions in references:
        replay_io.apply_targets(positions)
        mujoco.mj_step(scene.model, scene.data)
        mujoco.mj_forward(scene.model, scene.data)
    np.testing.assert_array_equal(scene.data.qpos, expected_qpos)
    np.testing.assert_array_equal(scene.data.qvel, expected_qvel)
