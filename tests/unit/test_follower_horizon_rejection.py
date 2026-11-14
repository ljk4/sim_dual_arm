"""Classify long Ruckig curves before the unchanged reference export contract."""

from types import SimpleNamespace

import pytest
from robot_contracts.models import ContractError, Target

ruckig = pytest.importorskip("ruckig")

from robot_backend_sim.follower import follower_plan  # noqa: E402
from robot_backend_sim.motion.ruckig_export import export_ruckig_reference  # noqa: E402


def long_profile():
    inputs = ruckig.InputParameter(1)
    inputs.current_position = [0.0]
    inputs.current_velocity = [0.0]
    inputs.current_acceleration = [0.0]
    inputs.target_position = [1.0]
    inputs.target_velocity = [0.0]
    inputs.target_acceleration = [0.0]
    inputs.max_velocity = [1.0]
    inputs.max_acceleration = [2.0]
    inputs.max_jerk = [float("inf")]
    inputs.minimum_duration = 4.0
    trajectory = ruckig.Trajectory(1)
    assert int(ruckig.Ruckig(1).calculate(inputs, trajectory)) >= 0
    return trajectory


def test_long_profile_is_classified_before_export(monkeypatch):
    trajectory = long_profile()
    monkeypatch.setattr(follower_plan, "_calculate_trajectory", lambda *_args: trajectory)
    window = SimpleNamespace(targets=(Target(250_000_000, (0.1,)),))
    limits = SimpleNamespace(joints=(), max_accelerations=())
    with pytest.raises(ContractError, match="FOLLOWER_HORIZON_EXCEEDED"):
        follower_plan._calculate(None, window, limits, (), 250_000_000, 2_000_000)


@pytest.mark.parametrize("truncate_ns", [None, 250_000_000])
def test_export_contract_still_rejects_a_long_full_curve(truncate_ns):
    with pytest.raises(ContractError, match="INVALID_RUCKIG_EXPORT"):
        export_ruckig_reference(long_profile(), ("axis",), truncate_ns=truncate_ns)
