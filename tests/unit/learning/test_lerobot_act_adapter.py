import dataclasses
import hashlib
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import pytest
import yaml

pytest.importorskip("torch")
pytest.importorskip("lerobot")
pb2 = pytest.importorskip("dual_arm_policy.policy_pb2")
grpc = pytest.importorskip("grpc")

from act_fixtures import ACTION, OBSERVATION, PROFILE, make_act_model  # noqa: E402
from dual_arm_learning.adapters.lerobot_act import (  # noqa: E402
    ActModelAdapter,
    load_act_adapter,
)
from dual_arm_learning.serving.config import (  # noqa: E402
    PolicyConfigError,
    PolicyServiceConfig,
    load_policy_config,
)
from dual_arm_learning.serving.pipeline import ChunkedPolicyPipeline, decode_rgb  # noqa: E402
from dual_arm_learning.serving.server import PolicyServer  # noqa: E402
from dual_arm_learning.serving.service import PolicyService  # noqa: E402
from dual_arm_learning.serving.temporal_ensemble import PERIOD_NS  # noqa: E402
from dual_arm_learning.serving.types import (  # noqa: E402
    PolicyError,
    ProviderCameraSample,
    ProviderJointSample,
    ProviderObservation,
    RunContext,
)
from dual_arm_learning.training.bundle import deployment_summary  # noqa: E402

ROOT = Path(__file__).resolve().parents[3]
CONFIG = ROOT / "shared/scenarios/marvin_sim/stack/configs/learning/policy.yaml"


@pytest.fixture(scope="module")
def bundle(tmp_path_factory):
    root = tmp_path_factory.mktemp("act-model")
    path = root / "act_20260101_000000"
    make_act_model(path)
    return path


def settings_for(bundle: Path) -> PolicyServiceConfig:
    return load_policy_config(
        CONFIG,
        model_path=bundle,
        robot_profile_path=PROFILE,
        observation_schema_path=OBSERVATION,
        action_schema_path=ACTION,
        device="cpu",
        require_formal=False,
    )


def plain(config: PolicyServiceConfig) -> PolicyServiceConfig:
    return config.model_copy(
        update={"max_prediction_corrections": {}, "temporal_ensemble_coeff": None}
    )


def fused(config: PolicyServiceConfig, coefficient: float = 0.01) -> PolicyServiceConfig:
    """Enable the provider-side ensemble independently of the recipe default."""
    return config.model_copy(update={"temporal_ensemble_coeff": coefficient})


def profile(bundle: Path) -> dict:
    del bundle
    return json.loads(PROFILE.read_text())


def initial_positions(bundle: Path) -> np.ndarray:
    return np.asarray([joint["initial"] for joint in profile(bundle)["joints"]], dtype=np.float64)


def cameras(bundle: Path, observation_id=1, reference_ns=1_000_000_000):
    array = np.zeros((240, 320, 3), dtype=np.uint8)
    array[:, :, 0] = 255
    return tuple(
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
        for camera in profile(bundle)["cameras"]
    )


class RecordingModel:
    policy_family = "act"
    chunk_size = 60
    action_dim = 16

    def __init__(self, bundle: Path, mode="ok"):
        self.base = initial_positions(bundle).astype(np.float32)
        self.mode = mode
        self.calls = 0
        self.resets = 0
        self.last_images = None
        self.last_state = None

    def reset(self):
        self.resets += 1

    def predict_raw_chunk(self, images, state):
        self.calls += 1
        self.last_images = images
        self.last_state = state
        if self.mode == "shape":
            return np.zeros((59, 16), dtype=np.float32)
        result = np.tile(self.base, (60, 1)).astype(np.float32)
        result[:, 0] += 0.001 * self.calls
        if self.mode == "nan":
            result[0, 0] = np.nan
        return result


class TemporalChunkModel(RecordingModel):
    def __init__(self, bundle: Path, base, invalid_at=None):
        super().__init__(bundle)
        self.base = np.asarray(base, dtype=np.float64)
        self.invalid_at = invalid_at

    def predict_raw_chunk(self, images, state):
        del images, state
        self.calls += 1
        result = np.tile(self.base, (60, 1))
        result[:, 0] += 0.01 * self.calls
        if self.calls == self.invalid_at:
            result[:, 0] = 3.5
        return result.astype(np.float32)


class BadWarmModel(ActModelAdapter):
    mode = "shape"

    def __init__(self, mode):
        super().__init__(
            None,
            device="cpu",
            chunk_size=60,
            action_dim=16,
            camera_names=("global", "left_wrist", "right_wrist"),
            camera_width=320,
            camera_height=240,
            initial_positions=(0.0,) * 16,
        )
        self.mode = mode

    def predict_raw_chunk(self, images, state):
        del images, state
        if self.mode == "shape":
            return np.zeros((1, 1), dtype=np.float32)
        result = np.zeros((60, 16), dtype=np.float32)
        result[0, 0] = np.nan
        return result


def context(config: PolicyServiceConfig, **changes) -> RunContext:
    values = {
        "robot_id": "marvin_stack",
        "robot_profile_hash": config.robot_profile_hash,
        "backend_instance_id": "backend-1",
        "time_domain_id": "mujoco",
        "clock_epoch": 1,
        "episode_id": "episode-1",
        "control_epoch": 1,
    }
    values.update(changes)
    return RunContext(**values)


def typed_observation(config, bundle, observation_id=1, reference_ns=1_000_000_000):
    profile_document = profile(bundle)
    return ProviderObservation(
        observation_id,
        reference_ns,
        ProviderJointSample(
            observation_id,
            reference_ns,
            config.joint_names,
            tuple(joint["initial"] for joint in profile_document["joints"]),
            (0.0,) * 16,
        ),
        cameras(bundle, observation_id, reference_ns),
        config.task_id,
        config.task_text,
    )


def pb2_observation(config, bundle, observation_id=1, reference_ns=1_000_000_000):
    template = typed_observation(config, bundle, observation_id, reference_ns)
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
    return message


def open_request(config, ctx, open_id="open-1"):
    return pb2.OpenSessionRequest(
        context=pb2.RunContext(**ctx.model_dump(mode="python")),
        model_bundle_digest=config.model_bundle_digest,
        observation_schema_hash=config.observation_schema_hash,
        action_schema_hash=config.action_schema_hash,
        robot_profile_json=PROFILE.read_bytes(),
        observation_schema_json=OBSERVATION.read_bytes(),
        action_schema_json=ACTION.read_bytes(),
        open_request_id=open_id,
    )


def temporal_base(bundle: Path) -> np.ndarray:
    result = initial_positions(bundle)
    joints = profile(bundle)["joints"]
    for axis in (7, 15):
        result[axis] = joints[axis]["max"] + 0.003
    return result


def artifact_path(directory, identity):
    return Path(directory) / (hashlib.sha256(identity.encode()).hexdigest() + ".npz")


def test_contract_is_reloaded_identically_and_formal_rejects_synthetic(bundle):
    first = settings_for(bundle)
    again = settings_for(bundle)
    assert first.to_document() == again.to_document()
    with pytest.raises(PolicyConfigError, match="synthetic"):
        load_policy_config(
            CONFIG,
            model_path=bundle,
            robot_profile_path=PROFILE,
            observation_schema_path=OBSERVATION,
            action_schema_path=ACTION,
            require_formal=True,
        )


def test_contract_rejects_wrong_profile_and_corrupt_model(bundle, tmp_path):
    broken_profile = tmp_path / "profile.json"
    document = json.loads(PROFILE.read_text())
    document["joints"] = document["joints"][:-1]
    broken_profile.write_text(json.dumps(document) + "\n", encoding="utf-8")
    with pytest.raises(PolicyConfigError, match="joint|profile"):
        load_policy_config(
            CONFIG,
            model_path=bundle,
            robot_profile_path=broken_profile,
            observation_schema_path=OBSERVATION,
            action_schema_path=ACTION,
            require_formal=False,
        )
    broken = bundle.parent / "act_20260101_000001"
    broken.mkdir()
    (broken / "model.safetensors").write_bytes(b"not a safetensors file")
    (broken / "training_args.yaml").write_text("", encoding="utf-8")
    with pytest.raises(PolicyConfigError, match="model"):
        load_policy_config(
            CONFIG,
            model_path=broken,
            robot_profile_path=PROFILE,
            observation_schema_path=OBSERVATION,
            action_schema_path=ACTION,
            require_formal=False,
        )


def test_real_bundle_offline_reload_returns_same_fresh_chunk(bundle):
    # This tiny random ACT emits values outside the formal profile bounds, so
    # compare the reloaded adapters directly; admission is covered separately.
    config = plain(settings_for(bundle))
    first_adapter = load_act_adapter(config)
    second_adapter = load_act_adapter(config)
    images = {
        camera.camera_name: decode_rgb(camera.payload, 320, 240)
        for camera in cameras(bundle)
    }
    state = np.asarray(initial_positions(bundle), dtype=np.float32)
    one = np.asarray(first_adapter.predict_raw_chunk(images, state))
    two = np.asarray(second_adapter.predict_raw_chunk(images, state))
    assert one.shape == two.shape == (60, 16)
    assert np.allclose(one, two, atol=1e-6)


def test_rgb_decode_is_chw_float_and_image_order(bundle):
    model = RecordingModel(bundle)
    pipeline = ChunkedPolicyPipeline(model, plain(settings_for(bundle)))
    pipeline.predict(
        identity="s1:p1",
        observation_id=1,
        observation_time_ns=1_000_000_000,
        cameras=cameras(bundle),
        joint_positions=tuple(initial_positions(bundle)),
    )
    images = model.last_images
    assert tuple(images) == ("global", "left_wrist", "right_wrist")
    image = images["global"]
    assert image.shape == (3, 240, 320)
    assert image.dtype == np.float32
    assert image[0].min() == 1.0 and image[0].max() == 1.0
    assert image[1].max() == 0.0 and image[2].max() == 0.0
    hwc = np.zeros((2, 3, 3), dtype=np.uint8)
    hwc[0, 0, 0], hwc[0, 1, 1], hwc[0, 2, 2] = 255, 128, 64
    chw = decode_rgb(hwc.tobytes(), 3, 2)
    assert chw.shape == (3, 2, 3)
    assert chw[0, 0, 0] == 1.0
    assert chw[1, 0, 1] == pytest.approx(128 / 255)
    assert chw[2, 0, 2] == pytest.approx(64 / 255)


def test_each_new_observation_calls_fresh_chunk_and_reset_clears_state(bundle):
    model = RecordingModel(bundle)
    config = plain(settings_for(bundle))
    pipeline = ChunkedPolicyPipeline(model, config)
    first = pipeline.predict(
        identity="s1:p1",
        observation_id=1,
        observation_time_ns=1_000_000_000,
        cameras=cameras(bundle, 1),
        joint_positions=tuple(initial_positions(bundle)),
    )
    second = pipeline.predict(
        identity="s1:p2",
        observation_id=2,
        observation_time_ns=1_050_000_000,
        cameras=cameras(bundle, 2),
        joint_positions=tuple(initial_positions(bundle)),
    )
    assert model.calls == 2
    assert first.targets[0, 0] != second.targets[0, 0]
    pipeline.reset()
    assert model.resets == 1


def test_service_requires_monotonic_observations_and_reset_clears_history(bundle):
    config = plain(settings_for(bundle))
    model = RecordingModel(bundle)
    service = PolicyService(config, model)
    ctx = context(config)
    opened = service.open_session(open_request(config, ctx))
    request = pb2.PredictRequest(
        session_id=opened.session_id,
        request_id="p1",
        context=pb2.RunContext(**ctx.model_dump(mode="python")),
        observation=pb2_observation(config, bundle, 1),
    )
    service.predict(request)
    with pytest.raises(PolicyError, match="advance monotonically"):
        service.predict(
            pb2.PredictRequest(
                session_id=opened.session_id,
                request_id="p2",
                context=pb2.RunContext(**ctx.model_dump(mode="python")),
                observation=pb2_observation(config, bundle, 1),
            )
        )
    service.reset_session(
        pb2.ResetSessionRequest(
            session_id=opened.session_id,
            reset_id="r1",
            new_context=pb2.RunContext(**context(config, clock_epoch=2).model_dump(mode="python")),
        )
    )
    service.predict(
        pb2.PredictRequest(
            session_id=opened.session_id,
            request_id="p3",
            context=pb2.RunContext(**context(config, clock_epoch=2).model_dump(mode="python")),
            observation=pb2_observation(config, bundle, 1),
        )
    )
    assert model.resets == 1


@pytest.mark.parametrize("mode", ["nan", "shape"])
def test_bad_model_output_shape_or_nonfinite_is_rejected(bundle, mode):
    model = RecordingModel(bundle, mode=mode)
    pipeline = ChunkedPolicyPipeline(model, plain(settings_for(bundle)))
    with pytest.raises(PolicyError, match="INVALID_PREDICTION"):
        pipeline.predict(
            identity="s1:p1",
            observation_id=1,
            observation_time_ns=1_000_000_000,
            cameras=cameras(bundle),
            joint_positions=tuple(initial_positions(bundle)),
        )


def test_wrong_context_and_profile_are_rejected(bundle):
    config = plain(settings_for(bundle))
    service = PolicyService(config, RecordingModel(bundle))
    ctx = context(config)
    opened = service.open_session(open_request(config, ctx))
    with pytest.raises(PolicyError, match="STALE_CONTEXT"):
        service.predict(
            pb2.PredictRequest(
                session_id=opened.session_id,
                request_id="p1",
                context=pb2.RunContext(**context(config, clock_epoch=2).model_dump(mode="python")),
                observation=pb2_observation(config, bundle),
            )
        )
    with pytest.raises(PolicyError, match="CONTEXT_MISMATCH"):
        service.open_session(
            open_request(config, context(config, robot_profile_hash="0" * 64), "open-2")
        )
    wrong_profile = json.loads(PROFILE.read_text())
    wrong_profile["profile_id"] = "tampered"
    with pytest.raises(PolicyError, match="SCHEMA_MISMATCH"):
        service.open_session(
            pb2.OpenSessionRequest(
                context=pb2.RunContext(**ctx.model_dump(mode="python")),
                model_bundle_digest=config.model_bundle_digest,
                observation_schema_hash=config.observation_schema_hash,
                action_schema_hash=config.action_schema_hash,
                robot_profile_json=json.dumps(wrong_profile).encode(),
                observation_schema_json=OBSERVATION.read_bytes(),
                action_schema_json=ACTION.read_bytes(),
                open_request_id="open-3",
            )
        )


def test_policy_service_preserves_act_rejection_reason(bundle):
    config = plain(settings_for(bundle))
    service = PolicyService(config, RecordingModel(bundle, mode="nan"))
    ctx = context(config)
    opened = service.open_session(open_request(config, ctx))
    with pytest.raises(PolicyError, match="INVALID_PREDICTION"):
        service.predict(
            pb2.PredictRequest(
                session_id=opened.session_id,
                request_id="predict-1",
                context=pb2.RunContext(**ctx.model_dump(mode="python")),
                observation=pb2_observation(config, bundle),
            )
        )


def test_grpc_server_shutdown_is_bounded(bundle):
    config = plain(settings_for(bundle))
    server = PolicyServer(config, RecordingModel(bundle))
    server.start()
    started = time.monotonic()
    server.stop(0.5)
    assert time.monotonic() - started < 3.0


def test_act_entry_loads_warms_logs_and_shuts_down_cleanly(bundle, tmp_path):
    document = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    document["model"].update(
        {
            "model_path": str(bundle),
            "robot_profile_path": str(PROFILE),
            "observation_schema_path": str(OBSERVATION),
            "action_schema_path": str(ACTION),
            "device": "cpu",
            "require_formal": False,
        }
    )
    document["service"]["port"] = 0
    document["service"]["bind_address"] = "127.0.0.1"
    config_path = tmp_path / "policy.yaml"
    config_path.write_text(yaml.safe_dump(document, sort_keys=False))
    log_dir = tmp_path / "act-log"
    environment = dict(os.environ)
    import dual_arm_policy
    import lerobot

    sdk = str(Path(dual_arm_policy.__file__).parents[1])
    source = os.environ.get("LEARNING_SOURCE", str(Path(lerobot.__file__).resolve().parents[1]))
    environment["PROJECT_CONTAINER"] = "1"
    environment["POLICY_SDK_ROOT"] = sdk
    environment["PATH"] = str(Path(sys.executable).parent) + ":" + environment["PATH"]
    inherited = environment.get("PYTHONPATH", "")
    environment["PYTHONPATH"] = ":".join(
        [
            str(ROOT / "learning/lerobot/src"),
            str(ROOT / "shared/common/python/run_logging/src"),
            str(ROOT / "shared/common/python/contracts/src"),
            sdk,
            source,
            *([inherited] if inherited else []),
        ]
    )
    process = subprocess.Popen(
        [
            "bash",
            str(ROOT / "learning/lerobot/scripts/server_policy.sh"),
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
        summary = deployment_summary(settings_for(bundle))
        assert snapshot["model"]["model_bundle_digest"] == summary["model_bundle_digest"]
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=5)


@pytest.mark.parametrize("mode", ["shape", "nan"])
def test_warmup_rejects_wrong_shape_or_nonfinite_before_ready(mode):
    with pytest.raises(PolicyError, match="warm-up"):
        BadWarmModel(mode).warm_up()


def test_startup_failure_leaves_structured_artifact(tmp_path):
    from dual_arm_learning.serving.cli import main as service_main

    log_dir = tmp_path / "failed-log"
    code = service_main(
        [
            "--config",
            str(CONFIG),
            "--model",
            str(tmp_path / "missing-model"),
            "--log-directory",
            str(log_dir),
            "--run-id",
            "bad-startup",
        ]
    )
    assert code == 1
    payload = json.loads((log_dir / "startup_failure.json").read_text())
    assert payload["status"] == "failed"
    assert payload["stage"] == "config"
    assert payload["error_type"] == "PolicyConfigError"
    assert payload["model"].endswith("missing-model")


def test_temporal_ensemble_is_off_by_default(bundle):
    assert PolicyServiceConfig.model_fields["temporal_ensemble_coeff"].default is None
    pipeline = ChunkedPolicyPipeline(RecordingModel(bundle), plain(settings_for(bundle)))
    assert pipeline._ensemble is None


def test_temporal_ensemble_fuses_admitted_history_and_records_npz(bundle, tmp_path):
    model = TemporalChunkModel(bundle, temporal_base(bundle))
    directory = tmp_path / "predictions"
    config = fused(settings_for(bundle))
    pipeline = ChunkedPolicyPipeline(model, config, prediction_directory=directory)
    first = pipeline.predict(
        identity="s1:p1",
        observation_id=1,
        observation_time_ns=1_000_000_000,
        cameras=cameras(bundle, 1),
        joint_positions=tuple(temporal_base(bundle)),
    )
    pipeline.commit(first)
    second = pipeline.predict(
        identity="s1:p2",
        observation_id=2,
        observation_time_ns=1_100_000_000,
        cameras=cameras(bundle, 2),
        joint_positions=tuple(temporal_base(bundle)),
    )
    pipeline.commit(second)

    executed = second.targets
    with np.load(artifact_path(directory, "s1:p2"), allow_pickle=False) as stored:
        raw = stored["raw_act"]
        admitted = stored["admitted"]
        for axis in (7, 15):
            np.testing.assert_allclose(raw[:, axis], config.joint_upper_bounds[axis] + 0.003)
            np.testing.assert_allclose(admitted[:, axis], config.joint_upper_bounds[axis])
        np.testing.assert_allclose(stored["correction"], admitted - raw)
        np.testing.assert_array_equal(stored["execution_targets"], executed)
        np.testing.assert_array_equal(stored["ensemble_counts"][:58], 2)
        np.testing.assert_array_equal(stored["ensemble_counts"][58:], 1)
        np.testing.assert_array_equal(stored["history_observation_ids"], [1])
        np.testing.assert_array_equal(stored["history_observation_time_ns"], [1_000_000_000])

    weight = np.exp(0.01 * (2 * PERIOD_NS) / PERIOD_NS)
    base = temporal_base(bundle)
    expected = (weight * (base[0] + 0.01) + (base[0] + 0.02)) / (1.0 + weight)
    assert executed[0, 0] == pytest.approx(expected)
    assert executed[0, 1] == pytest.approx(base[1], abs=1e-6)
    with pytest.raises(PolicyError, match="temporal ensemble"):
        pipeline.predict(
            identity="s1:p3",
            observation_id=3,
            observation_time_ns=1_000_000_000,
            cameras=cameras(bundle, 3),
            joint_positions=tuple(temporal_base(bundle)),
        )


def test_temporal_history_is_per_session_and_reset_discards_it(bundle, tmp_path):
    model = TemporalChunkModel(bundle, temporal_base(bundle))
    directory = tmp_path / "predictions"
    config = fused(settings_for(bundle))
    first = ChunkedPolicyPipeline(model, config, prediction_directory=directory)
    second = ChunkedPolicyPipeline(model, config, prediction_directory=directory)
    one = first.predict(
        identity="s1:a1",
        observation_id=1,
        observation_time_ns=1_000_000_000,
        cameras=cameras(bundle, 1),
        joint_positions=tuple(temporal_base(bundle)),
    )
    first.commit(one)
    other = second.predict(
        identity="s2:b1",
        observation_id=1,
        observation_time_ns=1_000_000_000,
        cameras=cameras(bundle, 1),
        joint_positions=tuple(temporal_base(bundle)),
    )
    second.commit(other)
    with np.load(artifact_path(directory, "s2:b1"), allow_pickle=False) as stored:
        np.testing.assert_array_equal(stored["ensemble_counts"], 1)
    first.reset()
    again = first.predict(
        identity="s1:a2",
        observation_id=1,
        observation_time_ns=2_000_000_000,
        cameras=cameras(bundle, 1),
        joint_positions=tuple(temporal_base(bundle)),
    )
    first.commit(again)
    with np.load(artifact_path(directory, "s1:a2"), allow_pickle=False) as stored:
        np.testing.assert_array_equal(stored["ensemble_counts"], 1)
        assert stored["history_observation_ids"].size == 0


class AxisChunkModel(RecordingModel):
    """Deterministic chunk whose scripted axis values change per call."""

    def __init__(self, bundle: Path, base, overrides):
        super().__init__(bundle)
        self.base = np.asarray(base, dtype=np.float64)
        self.overrides = overrides

    def predict_raw_chunk(self, images, state):
        del images, state
        self.calls += 1
        result = np.tile(self.base, (60, 1))
        axis, value = self.overrides.get(self.calls, (None, None))
        if axis is not None:
            result[:, axis] = value
        return result.astype(np.float32)


def marvin_like_corrections() -> dict[str, float]:
    return {
        "left_joint7": 0.01,
        "left_gripper_width": 0.005,
        "right_gripper_width": 0.005,
    }


def test_real_recipe_quota_reaches_the_pipeline_and_legal_chunks_are_unchanged(
    bundle, tmp_path
):
    config = settings_for(bundle)
    assert config.max_prediction_corrections == {
        "left_gripper_width": 0.005,
        "right_gripper_width": 0.005,
    }
    assert "gripper_max_correction_m" not in config.to_document()
    directory = tmp_path / "predictions"
    pipeline = ChunkedPolicyPipeline(
        AxisChunkModel(bundle, initial_positions(bundle), {}),
        config,
        prediction_directory=directory,
    )
    legal = pipeline.predict(
        identity="s1:legal",
        observation_id=1,
        observation_time_ns=1_000_000_000,
        cameras=cameras(bundle, 1),
        joint_positions=tuple(initial_positions(bundle)),
    )
    assert np.array_equal(legal.targets, legal.raw)
    with np.load(artifact_path(directory, "s1:legal"), allow_pickle=False) as stored:
        np.testing.assert_array_equal(stored["raw_act"], legal.raw)
        np.testing.assert_array_equal(stored["admitted"], legal.admitted)
        assert not stored["correction"].any() and bool(stored["accepted"])

    gripper = config.action_joint_names.index("left_gripper_width")
    upper = config.joint_upper_bounds[gripper]
    pipeline = ChunkedPolicyPipeline(
        AxisChunkModel(bundle, initial_positions(bundle), {1: (gripper, upper + 0.003)}),
        config,
        prediction_directory=tmp_path / "projected",
    )
    projected = pipeline.predict(
        identity="s1:projected",
        observation_id=1,
        observation_time_ns=1_000_000_000,
        cameras=cameras(bundle, 1),
        joint_positions=tuple(initial_positions(bundle)),
    )
    assert np.all(projected.admitted[:, gripper] == upper)
    assert projected.raw[0, gripper] == pytest.approx(upper + 0.003, abs=1e-6)
    assert projected.correction[0, gripper] == pytest.approx(-0.003, abs=1e-6)


def test_named_axis_is_projected_in_both_signs_within_its_own_quota(bundle, tmp_path):
    config = settings_for(bundle).model_copy(
        update={"max_prediction_corrections": marvin_like_corrections()}
    )
    lower, upper = config.joint_lower_bounds, config.joint_upper_bounds
    axis = config.action_joint_names.index("left_joint7")
    directory = tmp_path / "predictions"
    pipeline = ChunkedPolicyPipeline(
        AxisChunkModel(bundle, initial_positions(bundle), {1: (axis, upper[axis] + 0.0062)}),
        config,
        prediction_directory=directory,
    )
    positive = pipeline.predict(
        identity="s1:positive",
        observation_id=1,
        observation_time_ns=1_000_000_000,
        cameras=cameras(bundle, 1),
        joint_positions=tuple(initial_positions(bundle)),
    )
    assert positive.admitted[:, axis].min() == upper[axis]
    assert positive.correction[0, axis] == pytest.approx(-0.0062, abs=1e-6)
    with np.load(artifact_path(directory, "s1:positive"), allow_pickle=False) as stored:
        np.testing.assert_array_equal(stored["raw_act"], positive.raw)
        np.testing.assert_array_equal(stored["admitted"], positive.admitted)
        np.testing.assert_array_equal(stored["correction"], positive.admitted - positive.raw)

    pipeline = ChunkedPolicyPipeline(
        AxisChunkModel(bundle, initial_positions(bundle), {1: (axis, lower[axis] - 0.0062)}),
        config,
        prediction_directory=tmp_path / "negative",
    )
    negative = pipeline.predict(
        identity="s1:negative",
        observation_id=1,
        observation_time_ns=1_000_000_000,
        cameras=cameras(bundle, 1),
        joint_positions=tuple(initial_positions(bundle)),
    )
    assert negative.admitted[:, axis].max() == lower[axis]
    assert negative.correction[0, axis] == pytest.approx(0.0062, abs=1e-6)


def test_over_quota_in_the_final_unselected_point_still_rejects_the_chunk(bundle, tmp_path):
    config = settings_for(bundle).model_copy(
        update={"max_prediction_corrections": marvin_like_corrections()}
    )
    base = initial_positions(bundle)
    axis = config.action_joint_names.index("left_joint7")

    class TailModel(AxisChunkModel):
        def predict_raw_chunk(self, images, state):
            result = super().predict_raw_chunk(images, state)
            result[-1, axis] = 3.5  # Only the last, normally unselected point overruns.
            return result

    directory = tmp_path / "predictions"
    pipeline = ChunkedPolicyPipeline(
        TailModel(bundle, base, {}), config, prediction_directory=directory
    )
    with pytest.raises(PolicyError, match="exceeds 0.01"):
        pipeline.predict(
            identity="s1:tail",
            observation_id=1,
            observation_time_ns=1_000_000_000,
            cameras=cameras(bundle, 1),
            joint_positions=tuple(base),
        )
    with np.load(artifact_path(directory, "s1:tail"), allow_pickle=False) as stored:
        assert not bool(stored["accepted"])
        assert stored["raw_act"][-1, axis] == pytest.approx(3.5)
        np.testing.assert_array_equal(
            stored["correction"], stored["admitted"] - stored["raw_act"]
        )
        assert "execution_targets" not in stored.files


def test_policy_service_returns_only_profile_bounded_targets_after_projection(bundle, tmp_path):
    config = settings_for(bundle).model_copy(
        update={"max_prediction_corrections": marvin_like_corrections()}
    )
    axis = config.action_joint_names.index("left_joint7")
    service = PolicyService(
        config,
        AxisChunkModel(
            bundle, initial_positions(bundle), {1: (axis, config.joint_upper_bounds[axis] + 0.0062)}
        ),
        prediction_directory=tmp_path / "predictions",
    )
    ctx = context(config)
    opened = service.open_session(open_request(config, ctx))
    reply = service.predict(
        pb2.PredictRequest(
            session_id=opened.session_id,
            request_id="predict-1",
            context=pb2.RunContext(**ctx.model_dump(mode="python")),
            observation=pb2_observation(config, bundle),
        )
    )
    assert len(reply.action.targets) == 60
    for target in reply.action.targets:
        for index, value in enumerate(target.positions):
            assert config.joint_lower_bounds[index] <= value <= config.joint_upper_bounds[index]
    assert reply.action.targets[0].positions[axis] == pytest.approx(
        config.joint_upper_bounds[axis], abs=1e-6
    )


def test_policy_service_rejects_an_over_quota_axis_without_caching_a_reply(bundle, tmp_path):
    config = settings_for(bundle).model_copy(
        update={"max_prediction_corrections": marvin_like_corrections()}
    )
    axis = config.action_joint_names.index("left_joint7")
    model = AxisChunkModel(
        bundle, initial_positions(bundle), {1: (axis, config.joint_upper_bounds[axis] + 0.02)}
    )
    service = PolicyService(config, model, prediction_directory=tmp_path / "predictions")
    ctx = context(config)
    opened = service.open_session(open_request(config, ctx))
    request = pb2.PredictRequest(
        session_id=opened.session_id,
        request_id="predict-1",
        context=pb2.RunContext(**ctx.model_dump(mode="python")),
        observation=pb2_observation(config, bundle),
    )
    with pytest.raises(PolicyError, match="exceeds 0.01"):
        service.predict(request)
    assert model.calls == 1
    session = service._sessions[opened.session_id]
    assert "predict-1" not in session.request_cache
    assert session.last_observation_id == 0 and session.predictions == 0


def test_quota_rejection_cannot_be_hidden_by_temporal_averaging(bundle, tmp_path):
    base = temporal_base(bundle)
    directory = tmp_path / "predictions"
    config = fused(settings_for(bundle)).model_copy(
        update={"max_prediction_corrections": marvin_like_corrections()}
    )
    axis = config.action_joint_names.index("left_joint7")
    out_of_range = config.joint_upper_bounds[axis] + 0.02
    pipeline = ChunkedPolicyPipeline(
        AxisChunkModel(bundle, base, {2: (axis, out_of_range)}),
        config,
        prediction_directory=directory,
    )
    first = pipeline.predict(
        identity="s1:p1",
        observation_id=1,
        observation_time_ns=1_000_000_000,
        cameras=cameras(bundle, 1),
        joint_positions=tuple(base),
    )
    pipeline.commit(first)

    # The fused value would be inside the profile bounds, but the individual
    # raw chunk is not: admission must run before fusion on the raw chunk.
    weight = np.exp(0.01 * 100_000_000 / PERIOD_NS)
    averaged = (weight * base[axis] + out_of_range) / (1.0 + weight)
    assert config.joint_lower_bounds[axis] <= averaged <= config.joint_upper_bounds[axis]

    with pytest.raises(PolicyError, match="exceeds 0.01"):
        pipeline.predict(
            identity="s1:p2",
            observation_id=2,
            observation_time_ns=1_100_000_000,
            cameras=cameras(bundle, 2),
            joint_positions=tuple(base),
        )
    with np.load(artifact_path(directory, "s1:p2"), allow_pickle=False) as stored:
        assert not bool(stored["accepted"])
        assert stored["raw_act"][0, axis] == pytest.approx(out_of_range)
        assert "execution_targets" not in stored.files

    third = pipeline.predict(
        identity="s1:p3",
        observation_id=3,
        observation_time_ns=1_200_000_000,
        cameras=cameras(bundle, 3),
        joint_positions=tuple(base),
    )
    pipeline.commit(third)
    with np.load(artifact_path(directory, "s1:p3"), allow_pickle=False) as stored:
        np.testing.assert_array_equal(stored["ensemble_counts"][:56], 2)
        np.testing.assert_array_equal(stored["ensemble_counts"][56:], 1)
        np.testing.assert_array_equal(stored["history_observation_ids"], [1])
        np.testing.assert_array_equal(stored["execution_targets"], third.targets)


def test_rejected_raw_is_not_hidden_or_added_to_temporal_history(bundle, tmp_path):
    model = TemporalChunkModel(bundle, temporal_base(bundle), invalid_at=2)
    directory = tmp_path / "predictions"
    config = fused(settings_for(bundle))
    pipeline = ChunkedPolicyPipeline(model, config, prediction_directory=directory)
    first = pipeline.predict(
        identity="s1:p1",
        observation_id=1,
        observation_time_ns=1_000_000_000,
        cameras=cameras(bundle, 1),
        joint_positions=tuple(temporal_base(bundle)),
    )
    pipeline.commit(first)
    with pytest.raises(PolicyError, match="hard limits"):
        pipeline.predict(
            identity="s1:p2",
            observation_id=2,
            observation_time_ns=1_100_000_000,
            cameras=cameras(bundle, 2),
            joint_positions=tuple(temporal_base(bundle)),
        )
    with np.load(artifact_path(directory, "s1:p2"), allow_pickle=False) as stored:
        assert not bool(stored["accepted"])
        assert "execution_targets" not in stored.files
    third = pipeline.predict(
        identity="s1:p3",
        observation_id=3,
        observation_time_ns=1_200_000_000,
        cameras=cameras(bundle, 3),
        joint_positions=tuple(temporal_base(bundle)),
    )
    pipeline.commit(third)
    with np.load(artifact_path(directory, "s1:p3"), allow_pickle=False) as stored:
        np.testing.assert_array_equal(stored["ensemble_counts"][:56], 2)
        np.testing.assert_array_equal(stored["ensemble_counts"][56:], 1)
        np.testing.assert_array_equal(stored["history_observation_ids"], [1])
        np.testing.assert_array_equal(stored["history_observation_time_ns"], [1_000_000_000])
