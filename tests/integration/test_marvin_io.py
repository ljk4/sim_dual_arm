"""Marvin named I/O: public order, opening/closure mapping and feedforward."""

from __future__ import annotations

import math
import sys
import threading
from pathlib import Path

import pytest

mujoco = pytest.importorskip("mujoco")
np = pytest.importorskip("numpy")

ROOT = Path(__file__).resolve().parents[2]
SIM_SRC = ROOT / "simulation/mujoco/src"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))

from robot_backend_sim.marvin_sim.handover.scene_config import load_marvin_scene_config  # noqa: E402
from robot_backend_sim.common.physics.contacts import resolve_contact_binding  # noqa: E402
from robot_backend_sim.marvin_sim.io import MarvinIO  # noqa: E402
from robot_backend_sim.marvin_sim.handover.scene import build_marvin_scene  # noqa: E402
from robot_contracts.models import ContractError  # noqa: E402
from robot_contracts.profiles import load_profile  # noqa: E402

SCENARIO = ROOT / "shared/scenarios/marvin_sim/handover"
CONFIG_PATH = SCENARIO / "configs/simulation/scene/marvin_handover.yaml"
PROFILE_PATH = SCENARIO / "profiles/robots/marvin_handover/profile.json"
SCHEMAS = ROOT / "shared/common/interfaces/schemas"


@pytest.fixture(scope="module")
def config():
    return load_marvin_scene_config(CONFIG_PATH)


@pytest.fixture(scope="module")
def profile():
    return load_profile(PROFILE_PATH, SCHEMAS)


@pytest.fixture()
def scene(config):
    return build_marvin_scene(config)


def physical_state(scene):
    data = scene.data
    return {
        "qpos": data.qpos.copy(),
        "qvel": data.qvel.copy(),
        "ctrl": data.ctrl.copy(),
        "time": float(data.time),
        "act": data.act.copy(),
        "qfrc_applied": data.qfrc_applied.copy(),
        "xfrc_applied": data.xfrc_applied.copy(),
        "qacc_warmstart": data.qacc_warmstart.copy(),
    }


def assert_physical_state(scene, expected):
    data = scene.data
    np.testing.assert_array_equal(data.qpos, expected["qpos"])
    np.testing.assert_array_equal(data.qvel, expected["qvel"])
    np.testing.assert_array_equal(data.ctrl, expected["ctrl"])
    assert data.time == expected["time"]
    np.testing.assert_array_equal(data.act, expected["act"])
    np.testing.assert_array_equal(data.qfrc_applied, expected["qfrc_applied"])
    np.testing.assert_array_equal(data.xfrc_applied, expected["xfrc_applied"])
    np.testing.assert_array_equal(data.qacc_warmstart, expected["qacc_warmstart"])


def test_public_order_is_named_not_physical_qpos_adjacency(scene, profile):
    io = MarvinIO(scene, profile)
    model = scene.model
    data = scene.data
    assert model.joint("Arm_R1_Joint").qposadr[0] < model.joint("Arm_L1_Joint").qposadr[0]
    for side, prefix in (("left", "L"), ("right", "R")):
        for index in range(1, 8):
            data.qpos[model.joint(f"Arm_{prefix}{index}_Joint").qposadr[0]] = 0.01 * index * (
                1 if side == "left" else -1
            )
    measurement = io.measure()
    for index in range(1, 8):
        assert measurement.positions[index - 1] == pytest.approx(0.01 * index)
        assert measurement.positions[7 + index] == pytest.approx(-0.01 * index)


def test_gripper_opening_and_closure_run_in_opposite_directions(scene, profile):
    io = MarvinIO(scene, profile)
    model = scene.model
    data = scene.data
    assert scene.gripper_ctrl(scene.gripper_open_total) == pytest.approx(0.0)
    assert scene.gripper_ctrl(scene.gripper_closed_total) == pytest.approx(0.08)

    opened = io.measure().positions[7]
    assert opened == pytest.approx(scene.gripper_open_total, abs=5e-4)
    target = list(profile.initial_positions)
    target[7] = scene.gripper_closed_total
    io.apply_targets(target)
    closing_velocity = None
    for _ in range(1000):
        mujoco.mj_step(model, data)
        measurement = io.measure()
        if closing_velocity is None and abs(measurement.velocities[7]) > 1e-4:
            closing_velocity = measurement.velocities[7]
    assert closing_velocity is not None and closing_velocity < 0.0
    closed = io.measure().positions[7]
    assert closed == pytest.approx(scene.gripper_closed_total, abs=5e-4)
    assert closed < opened

    target = list(profile.initial_positions)
    target[7] = scene.gripper_open_total
    io.apply_targets(target)
    opening_velocity = None
    for _ in range(1000):
        mujoco.mj_step(model, data)
        measurement = io.measure()
        if opening_velocity is None and abs(measurement.velocities[7]) > 1e-4:
            opening_velocity = measurement.velocities[7]
    assert opening_velocity is not None and opening_velocity > 0.0
    assert io.measure().positions[7] == pytest.approx(scene.gripper_open_total, abs=5e-4)


def test_velocity_feedforward_has_the_right_sign_and_reduces_lag(config, profile):
    peaks = []
    for enabled in (False, True):
        scene = build_marvin_scene(config)
        io = MarvinIO(scene, profile, velocity_feedforward=enabled)
        home = profile.initial_positions
        for _ in range(250):
            io.apply_reference(home, (0.0,) * 16)
            mujoco.mj_step(scene.model, scene.data)
        peak = 0.0
        for step in range(1001):
            u = step / 1000
            positions = list(home)
            velocities = [0.0] * 16
            positions[14] += 0.6 * u * u * (3 - 2 * u)
            velocities[14] = 0.6 * 6 * u * (1 - u) / 2.0
            io.apply_reference(positions, velocities)
            actuator = scene.indices.robot.arm_actuator_ids["right"][6]
            lower, upper = scene.model.actuator_ctrlrange[actuator]
            assert lower <= scene.data.ctrl[actuator] <= upper
            mujoco.mj_step(scene.model, scene.data)
            mujoco.mj_forward(scene.model, scene.data)
            measured = io.measure()
            peak = max(peak, abs(measured.positions[14] - positions[14]))
        peaks.append(peak)
    assert peaks[0] > 0.02
    assert peaks[1] < peaks[0] * 0.5


def test_gripper_feedforward_uses_negative_opening_rate(scene, profile):
    io = MarvinIO(scene, profile, velocity_feedforward=True)
    positions = list(profile.initial_positions)
    positions[7] = 0.0414
    velocities = [0.0] * 16
    actuator = scene.indices.robot.gripper_actuator_ids["left"]
    base = scene.gripper_ctrl(positions[7])
    velocities[7] = 0.01
    io.apply_reference(positions, velocities)
    assert scene.data.ctrl[actuator] < base
    velocities[7] = -0.01
    io.apply_reference(positions, velocities)
    assert scene.data.ctrl[actuator] > base


def test_target_and_reference_rejections_write_nothing(scene, profile):
    io = MarvinIO(scene, profile, velocity_feedforward=True)
    valid = list(io.measure().positions)
    high = valid.copy()
    high[8] = 4.0
    velocities = [0.0] * 16
    velocities[14] = math.nan
    for positions, reference in ((high, (0.0,) * 16), (valid, velocities)):
        before = physical_state(scene)
        with pytest.raises(ContractError):
            if positions is high:
                io.apply_targets(positions)
            else:
                io.apply_reference(positions, reference)
        assert_physical_state(scene, before)


def test_thread_and_scene_binding_are_enforced(config, profile):
    first = build_marvin_scene(config)
    second = build_marvin_scene(config)
    io = MarvinIO(first, profile)
    errors = []
    thread = threading.Thread(target=lambda: errors.append(_measure_error(io)))
    thread.start()
    thread.join()
    assert errors and errors[0].reason == "WRONG_THREAD"
    with pytest.raises(ContractError) as caught:
        io.require_scene(second)
    assert caught.value.reason == "SCENE_MISMATCH"


def _measure_error(io):
    try:
        io.measure()
    except ContractError as error:
        return error
    raise AssertionError("cross-thread measurement was accepted")


def test_contact_binding_uses_the_named_marvin_fingers(scene, profile):
    io = MarvinIO(scene, profile)
    binding = resolve_contact_binding(
        scene.model, scene.contact_object_names, io.finger_bodies
    )
    assert tuple(name for name, _body_id in binding.object_bodies) == ("block",)
    for side, prefix in (("left", "L"), ("right", "R")):
        assert binding.finger(side, 1).body_name == f"Arm_{prefix}10_Link"
        assert binding.finger(side, 2).body_name == f"Arm_{prefix}11_Link"


def test_wrong_profile_limits_are_rejected(scene, profile):
    joints = list(profile.joints)
    joints[0] = joints[0].model_copy(update={"maximum": 3.0})
    bad = profile.model_copy(update={"joints": tuple(joints)})
    with pytest.raises(ContractError) as caught:
        MarvinIO(scene, bad)
    assert caught.value.reason == "PROFILE_MISMATCH"
