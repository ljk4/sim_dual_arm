from __future__ import annotations

import math
import sys
import threading
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest

mujoco = pytest.importorskip("mujoco")
np = pytest.importorskip("numpy")

ROOT = Path(__file__).resolve().parents[2]
SIM_SRC = ROOT / "simulation/mujoco/src"
CONTRACTS_SRC = ROOT / "contracts/python/src"
for source in (SIM_SRC, CONTRACTS_SRC):
    if str(source) not in sys.path:
        sys.path.insert(0, str(source))

from robot_backend_sim.panda_io import JOINT_NAMES, Measurement, PandaIO  # noqa: E402
from robot_backend_sim.scene import build_scene  # noqa: E402
from robot_backend_sim.scene_config import load_scene_config  # noqa: E402
from robot_contracts.models import ContractError  # noqa: E402
from robot_contracts.profiles import load_profile  # noqa: E402

CONFIG_PATH = ROOT / "simulation/mujoco/configs/scene/panda_stack.yaml"
PROFILE_PATH = ROOT / "profiles/robots/panda_stack_v1/profile.json"
SCHEMAS = ROOT / "contracts/schemas"


@pytest.fixture(scope="module")
def config():
    return load_scene_config(CONFIG_PATH)


@pytest.fixture(scope="module")
def scene(config):
    return build_scene(
        config,
    )


@pytest.fixture(scope="module")
def profile():
    return load_profile(PROFILE_PATH, SCHEMAS)


@pytest.fixture()
def io(scene, profile):
    scene.reset(0)
    return PandaIO(scene, profile)


def physical_state(scene):
    data = scene.data
    return {
        "qpos": data.qpos.copy(),
        "qvel": data.qvel.copy(),
        "time": float(data.time),
        "act": data.act.copy(),
        "qfrc_applied": data.qfrc_applied.copy(),
        "xfrc_applied": data.xfrc_applied.copy(),
        "qacc_warmstart": data.qacc_warmstart.copy(),
        "objects": {obj.name: data.body(obj.name).xpos.copy() for obj in scene.config.objects},
    }


def assert_physical_state(scene, expected):
    data = scene.data
    np.testing.assert_array_equal(data.qpos, expected["qpos"])
    np.testing.assert_array_equal(data.qvel, expected["qvel"])
    assert data.time == expected["time"]
    np.testing.assert_array_equal(data.act, expected["act"])
    np.testing.assert_array_equal(data.qfrc_applied, expected["qfrc_applied"])
    np.testing.assert_array_equal(data.xfrc_applied, expected["xfrc_applied"])
    np.testing.assert_array_equal(data.qacc_warmstart, expected["qacc_warmstart"])
    for name, position in expected["objects"].items():
        np.testing.assert_array_equal(data.body(name).xpos, position)


def replace_joint(profile, index, **changes):
    joints = list(profile.joints)
    joints[index] = replace(joints[index], **changes)
    return replace(profile, joints=tuple(joints))


def replace_camera(profile, index, **changes):
    cameras = list(profile.cameras)
    cameras[index] = replace(cameras[index], **changes)
    return replace(profile, cameras=tuple(cameras))


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


def test_initial_measurement_uses_named_qpos_and_finger_sums(io, scene, profile):
    measurement = io.measure()
    assert isinstance(measurement, Measurement)
    assert measurement.joint_names == JOINT_NAMES
    assert measurement.sim_time_ns == 0
    np.testing.assert_allclose(measurement.positions, profile.initial_positions, atol=1e-12)
    np.testing.assert_allclose(measurement.velocities, np.zeros(16), atol=0.0)

    model = scene.model
    data = scene.data
    for index, name in enumerate(JOINT_NAMES):
        if name.endswith("_gripper_width"):
            side = name.split("_", 1)[0]
            finger_names = (f"{side}_finger_joint1", f"{side}_finger_joint2")
            finger_positions = [data.qpos[model.joint(item).qposadr[0]] for item in finger_names]
            assert measurement.positions[index] == pytest.approx(sum(finger_positions))
            continue
        joint = model.joint(name)
        assert measurement.positions[index] == pytest.approx(data.qpos[joint.qposadr[0]])

    assert io.camera_names == tuple(camera.name for camera in profile.cameras)
    assert io.camera_frame_ids == tuple(camera.frame_id for camera in profile.cameras)


def test_measure_uses_seven_arm_joints_and_separate_finger_sums(io, scene):
    measurement = io.measure()
    model = scene.model
    data = scene.data
    for side, arm_start, gripper_index in (("left", 0, 7), ("right", 8, 15)):
        arm_names = JOINT_NAMES[arm_start : arm_start + 7]
        assert len(arm_names) == 7
        for local_index, name in enumerate(arm_names):
            assert name == f"{side}_joint{local_index + 1}"
            assert measurement.positions[arm_start + local_index] == pytest.approx(
                data.qpos[model.joint(name).qposadr[0]]
            )
        fingers = (f"{side}_finger_joint1", f"{side}_finger_joint2")
        expected = sum(data.qpos[model.joint(name).qposadr[0]] for name in fingers)
        assert measurement.positions[gripper_index] == pytest.approx(expected)


def test_measure_uses_dof_addresses_and_returns_independent_immutable_copies(io, scene):
    data = scene.data
    model = scene.model
    expected_velocities = []
    for index, name in enumerate(JOINT_NAMES):
        if name.endswith("_gripper_width"):
            side = name.split("_", 1)[0]
            dofs = [int(model.joint(f"{side}_finger_joint{joint}").dofadr[0]) for joint in (1, 2)]
            data.qvel[dofs[0]] = 0.0125
            data.qvel[dofs[1]] = 0.0175
            expected_velocities.append(0.03)
            continue
        dof = int(model.joint(name).dofadr[0])
        value = 0.001 * (index + 1)
        data.qvel[dof] = value
        expected_velocities.append(value)

    measurement = io.measure()
    np.testing.assert_allclose(measurement.velocities, expected_velocities, atol=1e-12)

    data.qvel[:] = 9.0
    data.time = 5.0
    np.testing.assert_allclose(measurement.velocities, expected_velocities, atol=1e-12)
    assert measurement.sim_time_ns == 0
    with pytest.raises(AttributeError):
        measurement.positions = ()
    with pytest.raises(TypeError):
        measurement.positions[0] = 1.0


def test_asymmetric_targets_write_named_ctrl_without_touching_physical_state(io, scene):
    initial = io.measure()
    targets = list(initial.positions)
    targets[0] = 0.02
    targets[7] = 0.04
    targets[8] = -0.03
    targets[15] = 0.02
    before = physical_state(scene)

    io.apply_targets(targets)

    for index in range(len(JOINT_NAMES)):
        if index == 7:
            actuator = scene.indices.gripper_actuator_ids["left"]
            assert scene.data.ctrl[actuator] == pytest.approx(127.5)
        elif index == 15:
            actuator = scene.indices.gripper_actuator_ids["right"]
            assert scene.data.ctrl[actuator] == pytest.approx(63.75)
        else:
            side = "left" if index < 8 else "right"
            local = index if index < 8 else index - 8
            actuator = scene.indices.arm_actuator_ids[side][local]
            assert scene.data.ctrl[actuator] == pytest.approx(targets[index])

    assert_physical_state(scene, before)
    measured_after_write = io.measure()
    assert measured_after_write.positions == initial.positions
    assert measured_after_write.velocities == initial.velocities
    assert measured_after_write.sim_time_ns == initial.sim_time_ns
    assert measured_after_write.positions != tuple(targets)

    mujoco.mj_step(scene.model, scene.data)
    stepped = io.measure()
    assert stepped.sim_time_ns > measured_after_write.sim_time_ns
    assert stepped.positions != measured_after_write.positions
    assert stepped.positions != tuple(targets)


def test_small_right_arm_and_gripper_motion_has_finite_real_response(io, scene):
    initial = io.measure()
    targets = list(initial.positions)
    targets[8] += 0.04
    targets[11] += 0.02
    targets[15] = 0.04
    io.apply_targets(targets)

    for _ in range(250):
        mujoco.mj_step(scene.model, scene.data)

    final = io.measure()
    assert np.isfinite(np.asarray(final.positions)).all()
    assert np.isfinite(np.asarray(final.velocities)).all()
    assert final.sim_time_ns == pytest.approx(500_000_000, abs=1)
    assert final.positions[8] > initial.positions[8] + 1e-6
    assert abs(final.positions[11] - initial.positions[11]) > 1e-6
    assert abs(final.positions[0] - initial.positions[0]) < 0.01
    assert final.positions[15] < initial.positions[15] - 1e-4
    tracking_error = [final.positions[index] - targets[index] for index in range(8, 15)]
    assert max(abs(error) for error in tracking_error) < 0.2, tracking_error


def test_wrong_profile_order_units_limits_home_are_rejected(scene, profile):
    reversed_joints = (profile.joints[1], profile.joints[0], *profile.joints[2:])
    cases = (
        ("order", replace(profile, joints=reversed_joints)),
        ("unit", replace_joint(profile, 0, unit="deg")),
        ("limits", replace_joint(profile, 0, maximum=3.0)),
        ("home", replace_joint(profile, 0, initial=0.1)),
        ("count", replace(profile, joints=profile.joints[:-1])),
    )
    for label, bad_profile in cases:
        with pytest.raises(ContractError) as caught:
            PandaIO(scene, bad_profile)
        assert caught.value.reason == "PROFILE_MISMATCH", label


def test_wrong_camera_order_size_and_calibration_digest_are_rejected(scene, profile):
    cameras = profile.cameras
    cases = (
        ("order", replace(profile, cameras=(cameras[1], cameras[0], cameras[2]))),
        ("size", replace_camera(profile, 0, width=321)),
        ("calibration", replace_camera(profile, 0, calibration_digest="0" * 64)),
    )
    for label, bad_profile in cases:
        with pytest.raises(ContractError) as caught:
            PandaIO(scene, bad_profile)
        assert caught.value.reason == "CAMERA_MISMATCH", label


def test_missing_and_wrong_actuators_are_rejected(scene, profile):
    gripper_id = scene.indices.gripper_actuator_ids["left"]
    original_range = scene.model.actuator_ctrlrange[gripper_id].copy()
    try:
        scene.model.actuator_ctrlrange[gripper_id] = (0.0, 254.0)
        with pytest.raises(ContractError, match="ctrlrange"):
            PandaIO(scene, profile)
    finally:
        scene.model.actuator_ctrlrange[gripper_id] = original_range

    arm_id = scene.indices.arm_actuator_ids["right"][0]
    original_target = int(scene.model.actuator_trnid[arm_id, 0])
    try:
        scene.model.actuator_trnid[arm_id, 0] = int(scene.model.joint("right_joint2").id)
        with pytest.raises(ContractError, match="does not drive"):
            PandaIO(scene, profile)
    finally:
        scene.model.actuator_trnid[arm_id, 0] = original_target

    class MissingActuatorModel:
        def __init__(self, model, missing):
            self._model = model
            self._missing = missing

        def __getattr__(self, name):
            return getattr(self._model, name)

        def actuator(self, name):
            if name == self._missing:
                raise KeyError(name)
            return self._model.actuator(name)

    bad_scene = SimpleNamespace(
        model=MissingActuatorModel(scene.model, "left_actuator1"),
        data=scene.data,
        config=scene.config,
        indices=scene.indices,
    )
    with pytest.raises(ContractError, match="left_actuator1"):
        PandaIO(bad_scene, profile)


def test_invalid_targets_are_rejected_before_any_ctrl_write(io, scene):
    valid = list(io.measure().positions)
    nan_target = valid.copy()
    nan_target[3] = math.nan
    inf_target = valid.copy()
    inf_target[3] = math.inf
    bool_target = valid.copy()
    bool_target[3] = True
    text_target = valid.copy()
    text_target[3] = "0.0"
    high_target = valid.copy()
    high_target[1] = 2.9
    low_target = valid.copy()
    low_target[1] = -2.9
    gripper_target = valid.copy()
    gripper_target[7] = 0.0801
    cases = (
        ("length", valid[:-1]),
        ("scalar", 0.0),
        ("nan", nan_target),
        ("inf", inf_target),
        ("bool", bool_target),
        ("text", text_target),
        ("high", high_target),
        ("low", low_target),
        ("gripper", gripper_target),
    )
    for label, target in cases:
        ctrl_before = scene.data.ctrl.copy()
        before = physical_state(scene)
        with pytest.raises(ContractError) as caught:
            io.apply_targets(target)
        assert caught.value.reason in {"INVALID_TARGET", "JOINT_LIMIT"}, label
        np.testing.assert_array_equal(scene.data.ctrl, ctrl_before)
        assert_physical_state(scene, before)


def test_creator_thread_exit_still_rejects_later_threads(scene, profile):
    created = []

    def create():
        created.append((PandaIO(scene, profile), tuple(profile.initial_positions)))

    creator = threading.Thread(target=create)
    creator.start()
    creator.join()
    assert len(created) == 1
    io, targets = created[0]
    ctrl_before = scene.data.ctrl.copy()

    for _ in range(5):
        measure_error = run_in_thread(io.measure)
        apply_error = run_in_thread(lambda: io.apply_targets(targets))
        assert isinstance(measure_error, ContractError)
        assert measure_error.reason == "WRONG_THREAD"
        assert isinstance(apply_error, ContractError)
        assert apply_error.reason == "WRONG_THREAD"
        np.testing.assert_array_equal(scene.data.ctrl, ctrl_before)


def test_cross_thread_measure_and_apply_are_rejected_without_writes(io, scene):
    before = physical_state(scene)
    ctrl_before = scene.data.ctrl.copy()
    targets = list(io.measure().positions)

    measure_error = run_in_thread(io.measure)
    apply_error = run_in_thread(lambda: io.apply_targets(targets))

    assert isinstance(measure_error, ContractError)
    assert measure_error.reason == "WRONG_THREAD"
    assert isinstance(apply_error, ContractError)
    assert apply_error.reason == "WRONG_THREAD"
    np.testing.assert_array_equal(scene.data.ctrl, ctrl_before)
    assert_physical_state(scene, before)


@pytest.mark.parametrize(
    "field", ["fovy", "position", "orientation", "parent", "resolution", "mode"]
)
def test_compiled_camera_must_match_validated_declaration(scene, profile, field):
    model = scene.model
    camera = int(model.camera("right_wrist").id)
    arrays = {
        "fovy": model.cam_fovy,
        "position": model.cam_pos,
        "orientation": model.cam_quat,
        "parent": model.cam_bodyid,
        "resolution": model.cam_resolution,
        "mode": model.cam_mode,
    }
    array = arrays[field]
    previous = array[camera].copy()
    try:
        if field in {"fovy", "parent", "mode"}:
            array[camera] += 1
        else:
            array[camera, 0] += 1
        with pytest.raises(ContractError, match="CAMERA_MISMATCH"):
            PandaIO(scene, profile)
    finally:
        array[camera] = previous


@pytest.mark.parametrize("name", ["left_actuator1", "right_actuator8"])
def test_actuator_units_require_the_native_unit_gear(scene, profile, name):
    model = scene.model
    actuator = int(model.actuator(name).id)
    previous = model.actuator_gear[actuator].copy()
    try:
        model.actuator_gear[actuator, 0] = 2.0
        with pytest.raises(ContractError, match="MODEL_MISMATCH"):
            PandaIO(scene, profile)
    finally:
        model.actuator_gear[actuator] = previous


def test_velocity_feedforward_reduces_real_joint_tracking_lag(scene, profile):
    """A wrist rotation in free space, never a complete grasp/stack trial."""
    import mujoco

    peaks = []
    for enabled in (False, True):
        scene.reset(0)
        io = PandaIO(scene, profile, velocity_feedforward=enabled)
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
            mujoco.mj_step(scene.model, scene.data)
            mujoco.mj_forward(scene.model, scene.data)
            measured = io.measure()
            peak = max(peak, abs(measured.positions[14] - positions[14]))
            actuator = scene.indices.arm_actuator_ids["right"][6]
            lower, upper = scene.model.actuator_ctrlrange[actuator]
            assert lower <= scene.data.ctrl[actuator] <= upper
        peaks.append(peak)
    assert peaks[0] > 0.03
    assert peaks[1] < 0.003


def test_velocity_feedforward_rejects_bad_velocity_without_partial_write(scene, profile):
    scene.reset(0)
    io = PandaIO(scene, profile, velocity_feedforward=True)
    before = scene.data.ctrl.copy()
    velocities = [0.0] * 16
    velocities[14] = float("nan")
    with pytest.raises(ContractError):
        io.apply_reference(profile.initial_positions, velocities)
    assert np.array_equal(before, scene.data.ctrl)
