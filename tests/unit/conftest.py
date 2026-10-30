from dataclasses import replace
from pathlib import Path

import pytest
from robot_backend_core.engine import BackendEngine
from robot_contracts.models import RunContext, Target, Trajectory
from robot_contracts.profiles import load_profile

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture
def profile():
    return load_profile(
        ROOT / "profiles/robots/mock_dual_arm_v1/profile.json",
        ROOT / "contracts/schemas/v1",
    )


@pytest.fixture
def controlled(profile):
    engine = BackendEngine(profile)
    context = replace(engine.context, episode_id="episode-test", control_epoch=1)
    lease = engine.acquire(context, "runtime-test", 2_000_000_000, 0)
    return engine, context, lease


def command(context: RunContext, lease: str, **changes) -> Trajectory:
    value = Trajectory(
        context,
        lease,
        "command-1",
        1,
        1,
        500_000_000,
        ("left_j1", "left_j2", "right_j1", "right_j2", "left_gripper", "right_gripper"),
        (Target(1_000_000_000, (0.2, -0.2, 0.1, -0.1, 0.02, 0.02)),),
    )
    return replace(value, **changes)
