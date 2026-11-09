"""Unit tests for the explicit follower command kind and boundary normalization."""

from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

import pytest
from robot_contracts.action_chunks import ActionWindow, ChunkAnchor
from robot_contracts.control import ControlCommand
from robot_contracts.follower import FOLLOWER_ALGORITHM_VERSION, PreparedFollowerPlan
from robot_contracts.follower_math import normalize_follower_positions
from robot_contracts.models import ContractError, JointSpec, RunContext, Target
from robot_contracts.reference import ReferenceSample

ROOT = Path(__file__).resolve().parents[2]
for source in (ROOT / "simulation/mujoco/src", ROOT / "contracts/python/src"):
    if str(source) not in sys.path:
        sys.path.insert(0, str(source))

import robot_backend_sim.control_state as control_state_module  # noqa: E402
from robot_backend_sim.control_state import PreparedControlCommand  # noqa: E402
from robot_backend_sim.follower_config import load_follower_config  # noqa: E402
from robot_backend_sim.follower_plan import prepare_follower_plan  # noqa: E402
from robot_backend_sim.motion_config import load_control_config  # noqa: E402
from robot_contracts.profiles import load_profile  # noqa: E402

ruckig = pytest.importorskip("ruckig")

PROFILE = load_profile(
    ROOT / "profiles/robots/panda_stack_v1/profile.json",
    ROOT / "contracts/schemas/v1",
)
CONTROL = load_control_config(ROOT / "simulation/mujoco/configs/control/panda_control.yaml")
FOLLOWER = load_follower_config(ROOT / "simulation/mujoco/configs/control/panda_follower.yaml")
BASE = PROFILE.initial_positions
JOINT_COUNT = len(BASE)
GRID_NS = 2_000_000

def context() -> RunContext:
    return RunContext(
        PROFILE.robot_id,
        PROFILE.digest,
        "backend-instance",
        "mujoco-sim",
        0,
        episode_id="episode-follower",
        control_epoch=1,
    )


def command(observation_id: int = 1) -> ControlCommand:
    return ControlCommand(
        context(),
        "lease-1",
        "command-follower",
        1,
        observation_id,
        10_000_000_000,
    )


def linear_source(amplitude: float = 1.0, *, tail: float | None = None) -> tuple[Target, ...]:
    targets = []
    for index in range(60):
        seconds = (index + 1) * 0.05
        position = amplitude * seconds / 3.0
        if tail is not None and index == 59:
            position = tail
        values = list(BASE)
        values[0] = position
        targets.append(Target(round(seconds * 1_000_000_000), tuple(values)))
    return tuple(targets)


def window(
    source: tuple[Target, ...],
    *,
    handoff_ns: int = 250_000_000,
    observation_id: int = 1,
) -> ActionWindow:
    return ActionWindow(
        observation_id,
        0,
        handoff_ns,
        source,
        FOLLOWER.target_count,
    )


def anchor(
    action_window: ActionWindow,
    *,
    positions: tuple[float, ...] = BASE,
    velocities: tuple[float, ...] = (0.0,) * JOINT_COUNT,
    seq: int = 0,
    stream_id: str = "follower-unit",
) -> ChunkAnchor:
    return ChunkAnchor(
        stream_id,
        seq,
        action_window.handoff_time_ns,
        ReferenceSample(positions, velocities, (0.0,) * JOINT_COUNT),
        PROFILE.digest,
    )


def follower_plan(
    *,
    source: tuple[Target, ...] | None = None,
    handoff_ns: int = 250_000_000,
    is_final: bool = False,
    follower_config=FOLLOWER,
    observation_id: int = 1,
    positions: tuple[float, ...] = BASE,
) -> PreparedFollowerPlan:
    source = linear_source() if source is None else source
    action_window = window(
        source, handoff_ns=handoff_ns, observation_id=observation_id
    )
    return prepare_follower_plan(
        anchor(action_window, positions=positions),
        action_window,
        PROFILE,
        CONTROL,
        follower_config,
        is_final=is_final,
        duration_grid_ns=GRID_NS,
    )


def packet(prepared, *, observation_id: int = 1) -> PreparedControlCommand:
    return PreparedControlCommand(command(observation_id), prepared)


def test_follower_is_a_distinct_kind_and_settle_alias_is_derived() -> None:
    prepared = follower_plan()
    assert isinstance(prepared, PreparedFollowerPlan)
    assert prepared.settle_at_end is prepared.is_final is False
    assert packet(prepared).kind == "FOLLOWER"
    assert packet(prepared).request is None
    assert follower_plan(
        source=final_extension_source(), handoff_ns=0, is_final=True
    ).settle_at_end is True


def test_follower_fingerprint_covers_unselected_tail_and_follower_limits() -> None:
    source = linear_source()
    changed_tail = linear_source(tail=0.9)
    first = packet(follower_plan(source=source))
    tail = packet(follower_plan(source=changed_tail))
    limits = packet(
        follower_plan(
            source=source,
            follower_config=replace(
                FOLLOWER,
                max_target_deviations=(0.149,) + FOLLOWER.max_target_deviations[1:],
            ),
        )
    )
    assert first.prepared.window.targets == tail.prepared.window.targets
    assert first.prepared.plan == tail.prepared.plan
    assert first.fingerprint != tail.fingerprint
    assert first.fingerprint != limits.fingerprint


def final_extension_source() -> tuple[Target, ...]:
    targets = []
    for index in range(60):
        seconds = (index + 1) * 0.05
        values = list(BASE)
        values[0] = min(0.05, seconds / 0.25 * 0.05)
        targets.append(Target(round(seconds * 1_000_000_000), tuple(values)))
    return tuple(targets)


def test_follower_fingerprint_covers_coefficients_anchor_and_extension() -> None:
    first = packet(follower_plan())
    shifted = packet(follower_plan(source=linear_source(0.8), handoff_ns=300_000_000))
    final = packet(follower_plan(source=final_extension_source(), handoff_ns=0, is_final=True))
    assert first.fingerprint != shifted.fingerprint
    assert first.fingerprint != final.fingerprint
    assert final.prepared.extension_ns > 0
    assert final.prepared.plan.duration_ns % GRID_NS == 0
    assert final.prepared.duration_grid_ns == GRID_NS


def test_follower_fingerprint_covers_algorithm_version(monkeypatch) -> None:
    prepared = follower_plan()
    original = packet(prepared).fingerprint
    monkeypatch.setattr(control_state_module, "FOLLOWER_ALGORITHM_VERSION", "other-version")
    assert packet(prepared).fingerprint != original
    assert FOLLOWER_ALGORITHM_VERSION == "bounded-target-follower/ruckig-0.15.3/v1"


def test_follower_fingerprint_payload_keeps_all_terminal_and_deviation_evidence() -> None:
    closing = []
    for index, opening in enumerate((0.0016, 0.0012, 0.0008, 0.0004, 0.0)):
        values = list(BASE)
        values[7] = opening
        closing.append(Target((index + 1) * 50_000_000, tuple(values)))
    for index in range(5, 60):
        values = list(BASE)
        values[7] = 0.0
        closing.append(Target((index + 1) * 50_000_000, tuple(values)))
    start = list(BASE)
    start[7] = 0.002
    prepared = follower_plan(source=tuple(closing), handoff_ns=0, positions=tuple(start))
    payload = control_state_module._follower_payload(prepared)
    assert payload["kind"] == "FOLLOWER"
    assert payload["algorithm"] == FOLLOWER_ALGORITHM_VERSION
    assert payload["raw_terminal_velocities"] == list(prepared.raw_terminal_velocities)
    assert payload["restricted_terminal_velocities"] == list(
        prepared.restricted_terminal_velocities
    )
    assert payload["raw_terminal_velocities"][7] != payload[
        "restricted_terminal_velocities"
    ][7]
    assert payload["max_target_deviations"] == list(prepared.max_target_deviations)
    assert payload["max_target_deviation_limits"] == list(
        prepared.max_target_deviation_limits
    )
    assert payload["extension_ns"] == prepared.extension_ns
    assert payload["window"]["source_targets"][-1] == {
        "time_ns": 3_000_000_000,
        "positions": list(closing[-1].positions),
    }
    assert payload["anchor"]["reference"]["positions"] == list(start)


def test_follower_grid_is_strictly_validated() -> None:
    action_window = window(linear_source(), handoff_ns=251_000_000)
    for bad in (True, 0, -1, 1.5, "2000000"):
        with pytest.raises(ContractError) as caught:
            prepare_follower_plan(
                anchor(action_window),
                action_window,
                PROFILE,
                CONTROL,
                FOLLOWER,
                is_final=False,
                duration_grid_ns=bad,
            )
        assert caught.value.reason == "INVALID_FOLLOWER_REQUEST"
    with pytest.raises(ContractError) as caught:
        prepare_follower_plan(
            anchor(action_window),
            action_window,
            PROFILE,
            CONTROL,
            FOLLOWER,
            is_final=False,
            duration_grid_ns=GRID_NS,
        )
    assert caught.value.reason == "INVALID_FOLLOWER_REQUEST"


def test_boundary_normalization_covers_confirmed_roundoff_values() -> None:
    gripper = (
        JointSpec("closed", "m", 0.0, 0.08, 1.0, 0.0),
        JointSpec("other", "rad", -1.0, 1.0, 1.0, 0.0),
    )
    near_closed = normalize_follower_positions((-2.849e-11, 0.0), gripper)
    assert near_closed.sampled_positions[0] == -2.849e-11
    assert near_closed.applied_positions[0] == 0.0
    assert near_closed.corrections[0] == -2.849e-11
    assert near_closed.has_correction

    near_open = normalize_follower_positions((0.08000000002849, 0.0), gripper)
    assert near_open.applied_positions[0] == 0.08
    assert near_open.corrections[0] == pytest.approx(2.849e-11)
    assert all(
        joint.minimum <= value <= joint.maximum
        for value, joint in zip(near_open.applied_positions, gripper, strict=True)
    )

    inside = normalize_follower_positions((0.0, 0.5), gripper)
    assert inside.applied_positions == (0.0, 0.5)
    assert inside.corrections == (0.0, 0.0)
    assert not inside.has_correction

    for outside in ((-1e-6, 0.0), (0.080001, 0.0)):
        with pytest.raises(ContractError) as caught:
            normalize_follower_positions(outside, gripper)
        assert caught.value.reason == "FOLLOWER_BOUNDARY_MISMATCH"
