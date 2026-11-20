import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
PROFILES = ROOT / "shared/scenarios/marvin_sim/stack/profiles"
pytest.importorskip("grpc")
pb2 = pytest.importorskip("dual_arm_policy.policy_pb2")
from dual_arm_learning.policy_service import PolicyService  # noqa: E402
from dual_arm_learning.policy_types import PolicyError  # noqa: E402
from fake_provider import FakeProvider, load_fake_config  # noqa: E402
from scenario_fixtures import MARVIN_SHARED_PROFILES  # noqa: E402

CONFIG = ROOT / "tests/fakes/policy/fake_policy.yaml"


def documents():
    return (
        (PROFILES / "robots/marvin_stack/profile.json").read_bytes(),
        (MARVIN_SHARED_PROFILES / "observations/marvin_rgb_joint.json").read_bytes(),
        (MARVIN_SHARED_PROFILES / "actions/marvin_joint_position.json").read_bytes(),
    )


def context(config, **changes):
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
    return pb2.RunContext(**values)


def open_request(config, ctx, open_id="open-1"):
    profile, observation, action = documents()
    return pb2.OpenSessionRequest(
        context=ctx,
        model_bundle_digest=config.model_bundle_digest,
        observation_schema_hash=config.observation_schema_hash,
        action_schema_hash=config.action_schema_hash,
        robot_profile_json=profile,
        observation_schema_json=observation,
        action_schema_json=action,
        open_request_id=open_id,
    )


def observation(config, reference_ns=1_000_000_000):
    profile = json.loads((PROFILES / "robots/marvin_stack/profile.json").read_text())
    message = pb2.Observation(
        observation_id=1,
        reference_robot_time_ns=reference_ns,
        joints=pb2.JointSample(
            state_seq=1,
            captured_at_robot_time_ns=reference_ns,
            joint_names=list(config.joint_names),
            positions=[joint["initial"] for joint in profile["joints"]],
            velocities=[0.0] * 16,
        ),
        task_id=config.task_id,
        task_text=config.task_text,
    )
    for camera in profile["cameras"]:
        message.cameras.append(
            pb2.CameraSample(
                camera_name=camera["name"],
                frame_seq=1,
                captured_at_robot_time_ns=reference_ns,
                frame_id=camera["frame_id"],
                calibration_digest=camera["calibration_digest"],
                width=320,
                height=240,
                encoding=pb2.RAW_RGB8,
                payload=b"\x01" * (320 * 240 * 3),
            )
        )
    return message


def build(fault=None, settings=None):
    config, configured = load_fake_config(CONFIG)
    config = config.model_copy(update={"port": 0})
    provider = FakeProvider(config, settings or configured, fault=fault)
    return config, provider, PolicyService(config, provider)


def test_full_session_is_bounded_idempotent_and_resettable():
    config, provider, service = build()
    ctx = context(config)
    opened = service.open_session(open_request(config, ctx))
    assert opened.session_id
    assert opened.policy.chunk_size == 60
    # The same open request returns the same session without a second provider open.
    again = service.open_session(open_request(config, ctx))
    assert again.session_id == opened.session_id
    with pytest.raises(PolicyError, match="REQUEST_ID_CONFLICT"):
        service.open_session(open_request(config, context(config, clock_epoch=9), "open-1"))
    predict = pb2.PredictRequest(
        session_id=opened.session_id,
        request_id="predict-1",
        context=ctx,
        observation=observation(config),
    )
    first = service.predict(predict)
    second = service.predict(predict)
    assert first.SerializeToString() == second.SerializeToString()
    assert len(first.action.targets) == 60
    assert first.action.targets[0].time_from_observation_ns == 50_000_000
    assert first.action.targets[-1].time_from_observation_ns == 3_000_000_000
    assert all(len(target.positions) == 16 for target in first.action.targets)
    new_context = context(config, clock_epoch=2, episode_id="episode-2")
    reset = service.reset_session(
        pb2.ResetSessionRequest(
            session_id=opened.session_id, reset_id="reset-1", new_context=new_context
        )
    )
    assert reset.ready
    with pytest.raises(PolicyError, match="STALE_CONTEXT"):
        service.predict(predict)
    assert service.close_session(pb2.CloseSessionRequest(session_id=opened.session_id)).closed
    assert service.close_session(pb2.CloseSessionRequest(session_id=opened.session_id)).closed
    assert provider.closed == [opened.session_id]


def test_schema_and_provider_faults_are_rejected_before_execution():
    for fault in (
        "short_chunk",
        "non_uniform_time",
        "nan_position",
        "wrong_joint_order",
        "wrong_model_digest",
        "wrong_observation",
        "wrong_session",
        "wrong_context",
    ):
        config, _provider, service = build(fault=fault)
        ctx = context(config)
        opened = service.open_session(open_request(config, ctx))
        request = pb2.PredictRequest(
            session_id=opened.session_id,
            request_id=f"predict-{fault}",
            context=ctx,
            observation=observation(config),
        )
        with pytest.raises(PolicyError):
            service.predict(request)


def test_bad_camera_payload_and_old_context_are_rejected():
    config, _provider, service = build()
    ctx = context(config)
    opened = service.open_session(open_request(config, ctx))
    bad = observation(config)
    bad.cameras[0].payload = b"\x00"
    with pytest.raises(PolicyError, match="RGB payload"):
        service.predict(
            pb2.PredictRequest(
                session_id=opened.session_id,
                request_id="bad-camera",
                context=ctx,
                observation=bad,
            )
        )
    stale = context(config, clock_epoch=0)
    with pytest.raises(PolicyError, match="STALE_CONTEXT"):
        service.predict(
            pb2.PredictRequest(
                session_id=opened.session_id,
                request_id="old-context",
                context=stale,
                observation=observation(config),
            )
        )
