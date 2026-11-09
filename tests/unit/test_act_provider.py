import dataclasses
import hashlib
import json
import os
import subprocess
import time
from pathlib import Path

import numpy as np
import pytest
import yaml

pytest.importorskip("torch")
pytest.importorskip("lerobot")
pb2 = pytest.importorskip("dual_arm_policy.v1.policy_pb2")
grpc = pytest.importorskip("grpc")

from act_fixtures import ACTION, OBSERVATION, PROFILE, make_act_bundle  # noqa: E402
from dual_arm_learning.act_contract import (  # noqa: E402
    ActContractError,
    load_act_settings,
    resolve_act_contract,
)
from dual_arm_learning.act_provider import (  # noqa: E402
    ActProvider,
    decode_rgb,
    load_act_provider,
)
from dual_arm_learning.act_service import build_policy_config  # noqa: E402
from dual_arm_learning.act_temporal import PERIOD_NS  # noqa: E402
from dual_arm_learning.policy_server import PolicyServer  # noqa: E402
from dual_arm_learning.policy_service import PolicyService  # noqa: E402
from dual_arm_learning.policy_types import (  # noqa: E402
    PolicyError,
    ProviderCameraSample,
    ProviderContext,
    ProviderJointSample,
    ProviderObservation,
)

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "learning/lerobot/configs/act_policy_temporal.yaml"


@pytest.fixture(scope="module")
def bundle(tmp_path_factory):
    root = tmp_path_factory.mktemp("act-bundle")
    path = root / "bundle"
    make_act_bundle(path)
    return path


@pytest.fixture(scope="module")
def contract(bundle):
    return resolve_act_contract(
        bundle=bundle,
        profile_path=PROFILE,
        observation_schema_path=OBSERVATION,
        action_schema_path=ACTION,
        require_formal=False,
    )


def settings_for(bundle: Path):
    settings = load_act_settings(CONFIG)
    return dataclasses.replace(
        settings,
        bundle=bundle,
        profile=PROFILE,
        observation_schema=OBSERVATION,
        action_schema=ACTION,
        device="cpu",
        require_formal=False,
    )


class BadWarmModel:
    device = "cpu"

    def __init__(self, mode):
        self.mode = mode

    def predict(self, images, state):
        del images, state
        if self.mode == "shape":
            return np.zeros((1, 1), dtype=np.float32)
        result = np.zeros((60, 16), dtype=np.float32)
        result[0, 0] = np.nan
        return result


class RecordingModel:
    device = "cpu"

    def __init__(self, mode="ok"):
        self.mode = mode
        self.calls = 0
        self.last_images = None
        self.last_state = None

    def predict(self, images, state):
        self.calls += 1
        self.last_images = images
        self.last_state = state
        if self.calls == 1:
            return np.zeros((60, 16), dtype=np.float32)
        if self.mode == "nan":
            result = np.zeros((60, 16), dtype=np.float32)
            result[0, 0] = np.nan
            return result
        if self.mode == "shape":
            return np.zeros((59, 16), dtype=np.float32)
        return np.full((60, 16), float(self.calls), dtype=np.float32)


class TemporalChunkModel:
    """Valid arm blocks with a small per-call offset and an optional bad call."""

    device = "cpu"

    def __init__(self, base, invalid_at=None):
        self.base = np.asarray(base, dtype=np.float64)
        self.calls = 0
        self.invalid_at = invalid_at

    def predict(self, images, state):
        del images, state
        self.calls += 1
        result = np.tile(self.base, (60, 1))
        result[:, 0] += 0.01 * self.calls
        if self.calls == self.invalid_at:
            result[:, 0] = 3.5
        return result.astype(np.float32)


def temporal_base(contract):
    result = np.asarray(
        [joint["initial"] for joint in contract.profile["joints"]], dtype=np.float64
    )
    result[7] = result[15] = 0.09
    return result


def build_temporal_provider(bundle, contract, model, directory, *, coeff=0.01):
    config = build_policy_config(settings_for(bundle), contract)
    provider = ActProvider(
        config,
        contract,
        model,
        gripper_max_correction_m=0.03,
        temporal_ensemble_coeff=coeff,
        prediction_directory=directory,
    )
    return config, provider


def artifact_path(directory, identity):
    return Path(directory) / (hashlib.sha256(identity.encode()).hexdigest() + ".npz")


def profile_documents():
    return (
        PROFILE.read_bytes(),
        OBSERVATION.read_bytes(),
        ACTION.read_bytes(),
    )


def context(config, **changes):
    values = {
        "robot_id": "panda_stack",
        "robot_profile_hash": config.robot_profile_hash,
        "backend_instance_id": "backend-1",
        "time_domain_id": "mujoco",
        "clock_epoch": 1,
        "episode_id": "episode-1",
        "control_epoch": 1,
    }
    values.update(changes)
    return ProviderContext(**values)


def observation(config, contract, observation_id=1, reference_ns=1_000_000_000):
    profile = json.loads(Path(contract.bundle_root / "metadata/profile.json").read_text())
    cameras = []
    for camera in profile["cameras"]:
        array = np.zeros((240, 320, 3), dtype=np.uint8)
        array[:, :, 0] = 255
        array[:, :, 1] = 0
        array[:, :, 2] = 0
        cameras.append(
            ProviderCameraSample(
                camera["name"],
                observation_id,
                reference_ns,
                camera["frame_id"],
                camera["calibration_digest"],
                320,
                240,
                1,
                array.tobytes(),
            )
        )
    return ProviderObservation(
        observation_id=observation_id,
        reference_robot_time_ns=reference_ns,
        joints=ProviderJointSample(
            observation_id,
            reference_ns,
            config.joint_names,
            tuple(joint["initial"] for joint in profile["joints"]),
            (0.0,) * 16,
        ),
        cameras=tuple(cameras),
        task_id=config.task_id,
        task_text=config.task_text,
    )


def build_provider(bundle, contract, model):
    config = build_policy_config(settings_for(bundle), contract)
    provider = ActProvider(config, contract, model)
    return config, provider


def test_contract_is_reloaded_identically_and_formal_rejects_synthetic(bundle, contract):
    again = resolve_act_contract(
        bundle=bundle,
        profile_path=PROFILE,
        observation_schema_path=OBSERVATION,
        action_schema_path=ACTION,
        require_formal=False,
    )
    assert again.summary() == contract.summary()
    assert again.bundle_digest == contract.bundle_digest
    assert again.processor_digest == contract.processor_digest
    with pytest.raises(ActContractError, match="synthetic"):
        resolve_act_contract(
            bundle=bundle,
            profile_path=PROFILE,
            observation_schema_path=OBSERVATION,
            action_schema_path=ACTION,
            require_formal=True,
        )


def test_contract_rejects_wrong_profile_and_tampered_hash(bundle):
    with pytest.raises(ActContractError, match="profile"):
        resolve_act_contract(
            bundle=bundle,
            profile_path=ROOT / "profiles/robots/panda_stack_v1/profile.json",
            observation_schema_path=OBSERVATION,
            action_schema_path=ACTION,
            require_formal=False,
        )
    model = bundle / "model.safetensors"
    original = model.read_bytes()
    try:
        model.write_bytes(original + b"tamper")
        with pytest.raises(ActContractError, match="hash mismatch"):
            resolve_act_contract(
                bundle=bundle,
                profile_path=PROFILE,
                observation_schema_path=OBSERVATION,
                action_schema_path=ACTION,
                require_formal=False,
            )
    finally:
        model.write_bytes(original)


def test_real_bundle_offline_reload_returns_same_fresh_chunk(bundle, contract):
    config = build_policy_config(settings_for(bundle), contract)
    first = load_act_provider(config=config, contract=contract, device="cpu")
    second = load_act_provider(config=config, contract=contract, device="cpu")
    try:
        first.open("s1", context(config), "o1", *profile_documents())
        second.open("s2", context(config), "o2", *profile_documents())
        one = first.predict("s1", "p1", context(config), observation(config, contract))
        two = second.predict("s2", "p2", context(config), observation(config, contract))
        assert len(one.targets) == len(two.targets) == 60
        assert one.targets[0].time_from_observation_ns == 50_000_000
        assert one.targets[-1].time_from_observation_ns == 3_000_000_000
        assert all(len(target.positions) == 16 for target in one.targets)
        assert np.allclose(
            np.asarray([target.positions for target in one.targets]),
            np.asarray([target.positions for target in two.targets]),
            atol=1e-6,
        )
    finally:
        first.close("s1")
        second.close("s2")


def test_rgb_decode_is_chw_float_and_image_order(bundle, contract):
    model = RecordingModel()
    config, provider = build_provider(bundle, contract, model)
    provider.open("s1", context(config), "o1", *profile_documents())
    provider.predict("s1", "p1", context(config), observation(config, contract))
    images = model.last_images
    assert tuple(images) == config.camera_names
    image = images["global"]
    assert image.shape == (3, 240, 320)
    assert image.dtype == np.float32
    assert image[0].min() == 1.0 and image[0].max() == 1.0
    assert image[1].max() == 0.0 and image[2].max() == 0.0
    # A direct HWC payload with distinct channels proves order is not swapped.
    hwc = np.zeros((2, 3, 3), dtype=np.uint8)
    hwc[0, 0, 0], hwc[0, 1, 1], hwc[0, 2, 2] = 255, 128, 64
    chw = decode_rgb(hwc.tobytes(), 3, 2)
    assert chw.shape == (3, 2, 3)
    assert chw[0, 0, 0] == 1.0
    assert chw[1, 0, 1] == pytest.approx(128 / 255)
    assert chw[2, 0, 2] == pytest.approx(64 / 255)


def test_each_new_observation_calls_fresh_60_point_chunk_and_reset_clears_state(
    bundle, contract
):
    model = RecordingModel()
    config, provider = build_provider(bundle, contract, model)
    expected_initial = np.asarray(
        [joint["initial"] for joint in contract.profile["joints"]], dtype=np.float32
    )
    assert np.allclose(model.last_state, expected_initial)
    provider.open("s1", context(config), "o1", *profile_documents())
    first = provider.predict("s1", "p1", context(config), observation(config, contract, 1))
    second = provider.predict("s1", "p2", context(config), observation(config, contract, 2))
    assert model.calls == 3  # warm-up + two observations
    assert first.targets[0].positions != second.targets[0].positions
    with pytest.raises(PolicyError, match="advance monotonically"):
        provider.predict("s1", "p3", context(config), observation(config, contract, 2))
    provider.reset("s1", "r1", context(config, clock_epoch=2, control_epoch=2))
    third = provider.predict(
        "s1",
        "p4",
        context(config, clock_epoch=2, control_epoch=2),
        observation(config, contract, 1),
    )
    assert third.targets[0].positions != first.targets[0].positions
    assert model.calls == 4


@pytest.mark.parametrize("mode", ["nan", "shape"])
def test_bad_model_output_shape_or_nonfinite_is_rejected(bundle, contract, mode):
    model = RecordingModel(mode=mode)
    config, provider = build_provider(bundle, contract, model)
    provider.open("s1", context(config), "o1", *profile_documents())
    with pytest.raises(PolicyError, match="INVALID_PREDICTION"):
        provider.predict("s1", "p1", context(config), observation(config, contract))


def test_wrong_context_and_profile_are_rejected(bundle, contract):
    model = RecordingModel()
    config, provider = build_provider(bundle, contract, model)
    provider.open("s1", context(config), "o1", *profile_documents())
    with pytest.raises(PolicyError, match="STALE_CONTEXT"):
        provider.predict(
            "s1", "p1", context(config, clock_epoch=2), observation(config, contract)
        )
    with pytest.raises(PolicyError, match="CONTEXT_MISMATCH"):
        provider.open(
            "s2",
            context(config, robot_profile_hash="0" * 64),
            "o2",
            *profile_documents(),
        )
    wrong_profile = json.loads(PROFILE.read_text())
    wrong_profile["profile_id"] = "tampered"
    with pytest.raises(PolicyError, match="SCHEMA_MISMATCH"):
        provider.open(
            "s3",
            context(config),
            "o3",
            json.dumps(wrong_profile).encode(),
            OBSERVATION.read_bytes(),
            ACTION.read_bytes(),
        )


def test_policy_service_preserves_act_rejection_reason(bundle, contract):
    model = RecordingModel(mode="nan")
    config, provider = build_provider(bundle, contract, model)
    service = PolicyService(config, provider)
    ctx = context(config)
    opened = service.open_session(
        pb2.OpenSessionRequest(
            context=pb2.RunContext(**dataclasses.asdict(ctx)),
            model_bundle_digest=config.model_bundle_digest,
            observation_schema_hash=config.observation_schema_hash,
            action_schema_hash=config.action_schema_hash,
            robot_profile_json=PROFILE.read_bytes(),
            observation_schema_json=OBSERVATION.read_bytes(),
            action_schema_json=ACTION.read_bytes(),
            open_request_id="open-1",
        )
    )
    template = observation(config, contract)
    message = pb2.Observation(
        observation_id=template.observation_id,
        reference_robot_time_ns=template.reference_robot_time_ns,
        joints=pb2.JointSample(
            state_seq=template.joints.state_seq,
            captured_at_robot_time_ns=template.joints.captured_at_robot_time_ns,
            joint_names=list(template.joints.joint_names),
            positions=list(template.joints.positions),
            velocities=list(template.joints.velocities),
        ),
        task_id=config.task_id,
        task_text=config.task_text,
    )
    for camera in template.cameras:
        message.cameras.append(
            pb2.CameraSample(
                camera_name=camera.camera_name,
                frame_seq=camera.frame_seq,
                captured_at_robot_time_ns=camera.captured_at_robot_time_ns,
                frame_id=camera.frame_id,
                calibration_digest=camera.calibration_digest,
                width=camera.width,
                height=camera.height,
                encoding=pb2.RAW_RGB8,
                payload=camera.payload,
            )
        )
    with pytest.raises(PolicyError, match="INVALID_PREDICTION"):
        service.predict(
            pb2.PredictRequest(
                session_id=opened.session_id,
                request_id="predict-1",
                context=pb2.RunContext(**dataclasses.asdict(ctx)),
                observation=message,
            )
        )


def test_grpc_server_shutdown_is_bounded(bundle, contract):
    model = RecordingModel()
    config, provider = build_provider(bundle, contract, model)
    server = PolicyServer(config, provider)
    server.start()
    started = time.monotonic()
    server.stop(0.5)
    assert time.monotonic() - started < 3.0


def test_act_entry_loads_warms_logs_and_shuts_down_cleanly(bundle, contract, tmp_path):
    config = yaml.safe_load(CONFIG.read_text())
    config["model"].update(
        {
            "bundle": str(bundle),
            "profile": str(PROFILE),
            "observation_schema": str(OBSERVATION),
            "action_schema": str(ACTION),
            "device": "cpu",
            "require_formal": False,
        }
    )
    config["service"]["port"] = 0
    config["service"]["bind_address"] = "127.0.0.1"
    config_path = tmp_path / "act_policy.yaml"
    config_path.write_text(yaml.safe_dump(config, sort_keys=False))
    log_dir = tmp_path / "act-log"
    environment = dict(os.environ)
    import dual_arm_policy

    sdk = str(Path(dual_arm_policy.__file__).parents[1])
    import lerobot

    source = os.environ.get(
        "LEARNING_SOURCE", str(Path(lerobot.__file__).resolve().parents[1])
    )
    environment["PROJECT_CONTAINER"] = "1"
    environment["PYTHONPATH"] = ":".join(
        [
            str(ROOT / "learning/lerobot/src"),
            str(ROOT / "libraries/run_logging/src"),
            sdk,
            source,
        ]
    )
    process = subprocess.Popen(
        [
            "bash",
            str(ROOT / "learning/lerobot/scripts/run.sh"),
            "policy",
            "--config",
            str(config_path),
            "--log-directory",
            str(log_dir),
            "--run-id",
            "act-entry-test",
        ],
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        deadline = time.monotonic() + 60
        listening = ""
        while time.monotonic() < deadline:
            line = process.stdout.readline()
            if line:
                listening += line
                if "listening on" in line:
                    break
            elif process.poll() is not None:
                break
            else:
                time.sleep(0.05)
        if "listening on" not in listening:
            code = process.poll()
            out, err = process.communicate(timeout=10)
            raise AssertionError(
                f"no listening line; code={code} stdout={listening + out!r} stderr={err!r}"
            )
        process.terminate()
        code = process.wait(timeout=15)
        assert code == 0
        events = (log_dir / "events.000001.jsonl").read_text()
        assert "act.model.ready" in events
        assert "policy.ready" in events
        assert "policy.closed" in events
        snapshot = json.loads((log_dir / "resolved_contract.json").read_text())
        assert snapshot["bundle"]["bundle_digest"] == contract.bundle_digest
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=5)



@pytest.mark.parametrize("mode", ["shape", "nan"])
def test_warmup_rejects_wrong_shape_or_nonfinite_before_ready(bundle, contract, mode):
    with pytest.raises(PolicyError, match="warm-up"):
        build_provider(bundle, contract, BadWarmModel(mode))


def test_startup_failure_leaves_structured_artifact(tmp_path):
    from dual_arm_learning.act_service import main as act_main

    log_dir = tmp_path / "failed-log"
    code = act_main(
        [
            "--config",
            str(CONFIG),
            "--bundle",
            str(tmp_path / "missing-bundle"),
            "--log-directory",
            str(log_dir),
            "--run-id",
            "bad-startup",
        ]
    )
    assert code == 1
    payload = json.loads((log_dir / "startup_failure.json").read_text())
    assert payload["status"] == "failed"
    assert payload["stage"] == "contract"
    assert payload["error_type"] == "ActContractError"
    assert payload["bundle"].endswith("missing-bundle")


def test_temporal_ensemble_is_off_by_default(bundle, contract):
    model = RecordingModel()
    config, provider = build_provider(bundle, contract, model)
    summary = provider.summary()
    assert summary["temporal_ensemble"] is False
    assert summary["temporal_ensemble_coeff"] is None
    assert load_act_settings(CONFIG).temporal_ensemble_coeff is None


def test_temporal_ensemble_fuses_admitted_history_and_records_npz(
    bundle, contract, tmp_path
):
    model = TemporalChunkModel(temporal_base(contract))
    directory = tmp_path / "predictions"
    config, provider = build_temporal_provider(bundle, contract, model, directory)
    assert provider.summary()["temporal_ensemble"] is True
    assert provider.summary()["temporal_ensemble_coeff"] == pytest.approx(0.01)

    provider.open("s1", context(config), "o1", *profile_documents())
    first = provider.predict(
        "s1", "p1", context(config), observation(config, contract, 1, 1_000_000_000)
    )
    assert len(first.targets) == 60
    second = provider.predict(
        "s1", "p2", context(config), observation(config, contract, 2, 1_100_000_000)
    )

    executed = np.asarray([target.positions for target in second.targets])
    with np.load(artifact_path(directory, "s1:p2"), allow_pickle=False) as stored:
        raw = stored["raw_act"]
        admitted = stored["admitted"]
        np.testing.assert_allclose(raw[:, 7], 0.09)
        np.testing.assert_allclose(raw[:, 15], 0.09)
        np.testing.assert_allclose(admitted[:, 7], 0.08)
        np.testing.assert_allclose(admitted[:, 15], 0.08)
        np.testing.assert_allclose(stored["correction"], admitted - raw)
        np.testing.assert_array_equal(stored["execution_targets"], executed)
        np.testing.assert_array_equal(stored["ensemble_counts"][:58], 2)
        np.testing.assert_array_equal(stored["ensemble_counts"][58:], 1)
        np.testing.assert_array_equal(stored["history_observation_ids"], [1])
        np.testing.assert_array_equal(
            stored["history_observation_time_ns"], [1_000_000_000]
        )

    weight = np.exp(0.01 * (2 * PERIOD_NS) / PERIOD_NS)
    base = temporal_base(contract)
    expected = (weight * (base[0] + 0.02) + (base[0] + 0.03)) / (1.0 + weight)
    assert executed[0, 0] == pytest.approx(expected)
    # Every joint uses the same rule; dimensions without a disagreement stay put.
    assert executed[0, 1] == pytest.approx(base[1], abs=1e-6)

    with pytest.raises(PolicyError, match="temporal ensemble"):
        provider.predict(
            "s1",
            "p3",
            context(config),
            observation(config, contract, 3, 1_000_000_000),
        )


def test_temporal_history_is_per_session_and_reset_discards_it(bundle, contract, tmp_path):
    model = TemporalChunkModel(temporal_base(contract))
    directory = tmp_path / "predictions"
    config, provider = build_temporal_provider(bundle, contract, model, directory)
    provider.open("s1", context(config), "o1", *profile_documents())
    provider.open("s2", context(config), "o2", *profile_documents())
    provider.predict(
        "s1", "a1", context(config), observation(config, contract, 1, 1_000_000_000)
    )
    provider.predict(
        "s2", "b1", context(config), observation(config, contract, 1, 1_000_000_000)
    )
    with np.load(artifact_path(directory, "s2:b1"), allow_pickle=False) as stored:
        np.testing.assert_array_equal(stored["ensemble_counts"], 1)

    provider.reset("s1", "r1", context(config, clock_epoch=2, control_epoch=2))
    provider.predict(
        "s1",
        "a2",
        context(config, clock_epoch=2, control_epoch=2),
        observation(config, contract, 1, 2_000_000_000),
    )
    with np.load(artifact_path(directory, "s1:a2"), allow_pickle=False) as stored:
        np.testing.assert_array_equal(stored["ensemble_counts"], 1)
        assert stored["history_observation_ids"].size == 0


def test_rejected_raw_is_not_hidden_or_added_to_temporal_history(bundle, contract, tmp_path):
    model = TemporalChunkModel(temporal_base(contract), invalid_at=3)
    directory = tmp_path / "predictions"
    config, provider = build_temporal_provider(bundle, contract, model, directory)
    provider.open("s1", context(config), "o1", *profile_documents())
    provider.predict(
        "s1", "p1", context(config), observation(config, contract, 1, 1_000_000_000)
    )
    with pytest.raises(PolicyError, match="hard limits"):
        provider.predict(
            "s1", "p2", context(config), observation(config, contract, 2, 1_100_000_000)
        )
    with np.load(artifact_path(directory, "s1:p2"), allow_pickle=False) as stored:
        assert not bool(stored["accepted"])
        assert "execution_targets" not in stored.files

    provider.predict(
        "s1", "p3", context(config), observation(config, contract, 3, 1_200_000_000)
    )
    with np.load(artifact_path(directory, "s1:p3"), allow_pickle=False) as stored:
        np.testing.assert_array_equal(stored["ensemble_counts"][:56], 2)
        np.testing.assert_array_equal(stored["ensemble_counts"][56:], 1)
        np.testing.assert_array_equal(stored["history_observation_ids"], [1])
        np.testing.assert_array_equal(
            stored["history_observation_time_ns"], [1_000_000_000]
        )
