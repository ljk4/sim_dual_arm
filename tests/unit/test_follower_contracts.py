from __future__ import annotations

import copy
import math
import sys
from pathlib import Path

import pytest
import yaml
from model_helpers import replace
from pydantic import ValidationError
from robot_contracts.action_chunks import ActionWindow, ChunkAnchor
from robot_contracts.models import ContractError, JointSpec, Target
from robot_contracts.trajectory.follower import FollowerLimitFlags
from robot_contracts.trajectory.spline import CubicSegment, ReferencePlan, ReferenceSample
from robot_trajectory.follower import PreparedFollowerPlan
from robot_trajectory.follower_math import compute_follower_deviations, restrict_terminal_velocities

ROOT = Path(__file__).resolve().parents[2]
SIM_SRC = ROOT / "simulation/mujoco/src"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))

CONFIG_PATH = ROOT / "shared/scenarios/panda_sim/stack/profiles/control/panda_follower_v1.yaml"
PRODUCER = "fixture-producer/v1"


def joint(name: str, *, minimum: float = -10.0, maximum: float = 10.0, velocity: float = 1.0):
    return JointSpec(name, "rad", minimum, maximum, velocity, 0.0)


def two_joint_window(
    positions: tuple[float, ...] = (0.0, 0.0),
    velocities: tuple[float, ...] = (0.0, 0.0),
) -> tuple[ChunkAnchor, ActionWindow]:
    targets = (
        Target(300_000_000, (0.02, 0.0)),
        Target(400_000_000, (0.03, 0.0)),
    )
    window = ActionWindow(
        observation_id=1,
        observation_time_ns=0,
        handoff_time_ns=250_000_000,
        source_targets=targets,
        target_count=2,
    )
    anchor = ChunkAnchor(
        "test-stream",
        0,
        window.handoff_time_ns,
        ReferenceSample(positions, velocities, (0.0, 0.0)),
        "profile-hash",
    )
    return anchor, window


def non_final_plan() -> ReferencePlan:
    return ReferencePlan(
        ("joint1", "joint2"),
        (
            CubicSegment(100_000_000, (0.0, 0.0), (0.0, 0.0), (1.0, 0.0), (0.0, 0.0)),
            CubicSegment(50_000_000, (0.01, 0.0), (0.2, 0.0), (1.0, 0.0), (0.0, 0.0)),
        ),
    )


def make_non_final() -> tuple[PreparedFollowerPlan, tuple[JointSpec, ...]]:
    anchor, window = two_joint_window()
    joints = (joint("joint1"), joint("joint2"))
    plan = PreparedFollowerPlan(
        anchor=anchor,
        window=window,
        plan=non_final_plan(),
        joints=joints,
        max_accelerations=(2.0, 1.0),
        max_target_deviation_limits=(0.15, 0.15),
        raw_terminal_velocities=(0.1, 0.0),
        restricted_terminal_velocities=(0.1, 0.0),
        is_final=False,
        requested_window_ns=window.end_time_ns,
        curve_duration_ns=window.end_time_ns,
        algorithm_version=PRODUCER,
    )
    return plan, joints


def test_non_final_plan_is_immutable_and_serialises_every_source_target():
    prepared, _ = make_non_final()
    assert prepared.duration_ns == 150_000_000
    assert prepared.extension_ns == 0
    assert prepared.actual_terminal_positions == pytest.approx((0.0225, 0.0))
    assert prepared.actual_terminal_velocities == pytest.approx((0.3, 0.0))
    assert prepared.max_target_deviations[0] > 0.0
    assert prepared.max_target_deviations[1] == 0.0
    assert prepared.flags.acceleration_limited[0] is True
    assert prepared.flags.boundary_preserved[0] is False
    assert prepared.flags.velocity_limited == (False, False)

    document = prepared.to_document()
    assert document["kind"] == "bounded_target_follower"
    assert document["is_final"] is False
    assert document["window"]["source_targets"] == [
        {"time_ns": 300_000_000, "positions": [0.02, 0.0]},
        {"time_ns": 400_000_000, "positions": [0.03, 0.0]},
    ]
    assert document["window"]["desired_targets"] == [
        {"time_ns": 50_000_000, "positions": [0.02, 0.0]},
        {"time_ns": 150_000_000, "positions": [0.03, 0.0]},
    ]
    assert len(document["plan"]["segments"]) == 2
    assert document["plan"]["segments"][0]["c0"] == [0.0, 0.0]
    assert len(document["max_target_deviations"]) == 2
    assert document["flags"]["any_acceleration_limited"] is True
    assert "actual_terminal_positions" in document

    with pytest.raises(ValidationError):
        prepared.is_final = True  # type: ignore[misc]


def test_final_plan_extended_tail_reaches_target_at_zero_velocity():
    anchor, window = two_joint_window()
    duration_s = 0.15
    target = 0.03
    c2 = 3.0 * target / duration_s**2
    c3 = -2.0 * target / duration_s**3
    plan = ReferencePlan(
        ("joint1", "joint2"),
        (
            CubicSegment(150_000_000, (0.0, 0.0), (0.0, 0.0), (c2, 0.0), (c3, 0.0)),
            CubicSegment(50_000_000, (target, 0.0), (0.0, 0.0), (0.0, 0.0), (0.0, 0.0)),
        ),
    )
    prepared = PreparedFollowerPlan(
        anchor=anchor,
        window=window,
        plan=plan,
        joints=(joint("joint1"), joint("joint2")),
        max_accelerations=(10.0, 1.0),
        max_target_deviation_limits=(0.15, 0.15),
        raw_terminal_velocities=(0.1, 0.0),
        restricted_terminal_velocities=(0.0, 0.0),
        is_final=True,
        requested_window_ns=window.end_time_ns,
        curve_duration_ns=plan.duration_ns,
        algorithm_version=PRODUCER,
    )
    assert prepared.duration_ns == 200_000_000
    assert prepared.extension_ns == 50_000_000
    assert prepared.actual_terminal_positions == pytest.approx((0.03, 0.0))
    assert prepared.actual_terminal_velocities == pytest.approx((0.0, 0.0))
    assert prepared.flags.boundary_preserved == (True, True)
    assert prepared.flags.velocity_limited == (True, False)


def test_exact_cubic_deviation_is_not_a_tick_sample():
    plan = ReferencePlan(
        ("joint1",),
        (CubicSegment(1_000_000_000, (0.0,), (0.0,), (0.0,), (1.0,)),),
    )
    deviation = compute_follower_deviations(
        plan,
        (0.0,),
        (Target(1_000_000_000, (1.0,)),),
        False,
    )[0]
    assert deviation == pytest.approx(2.0 / (3.0 * math.sqrt(3.0)), rel=1e-12)
    sampled = max(
        abs((t / 1_000_000_000) ** 3 - t / 1_000_000_000)
        for t in range(0, 1_000_000_001, 1_000_000)
    )
    assert deviation > sampled


@pytest.mark.parametrize(
    "change",
    (
        {
            "plan": ReferencePlan(
                ("joint1",),
                (CubicSegment(150_000_000, (0.0,), (0.0,), (1.0,), (0.0,)),),
            )
        },
        {"max_accelerations": (2.0,)},
        {"max_target_deviation_limits": (0.0, 0.15)},
        {"raw_terminal_velocities": (0.1,)},
        {"restricted_terminal_velocities": (2.0, 0.0)},
        {"is_final": True},
        {"curve_duration_ns": 100_000_000},
        {"requested_window_ns": 100_000_000},
    ),
)
def test_invalid_or_inconsistent_plan_is_rejected(change):
    anchor, window = two_joint_window()
    fields = {
        "anchor": anchor,
        "window": window,
        "plan": non_final_plan(),
        "joints": (joint("joint1"), joint("joint2")),
        "max_accelerations": (2.0, 1.0),
        "max_target_deviation_limits": (0.15, 0.15),
        "raw_terminal_velocities": (0.1, 0.0),
        "restricted_terminal_velocities": (0.1, 0.0),
        "is_final": False,
        "requested_window_ns": window.end_time_ns,
        "curve_duration_ns": window.end_time_ns,
        "algorithm_version": PRODUCER,
    }
    fields.update(change)
    with pytest.raises(ContractError):
        PreparedFollowerPlan(**fields)


def test_discontinuous_plan_and_exceeded_deviation_are_rejected():
    anchor, window = two_joint_window()
    broken = ReferencePlan(
        ("joint1", "joint2"),
        (
            CubicSegment(100_000_000, (0.0, 0.0), (0.0, 0.0), (1.0, 0.0), (0.0, 0.0)),
            CubicSegment(50_000_000, (0.02, 0.0), (0.2, 0.0), (1.0, 0.0), (0.0, 0.0)),
        ),
    )
    with pytest.raises(ContractError, match="C1"):
        PreparedFollowerPlan(
            anchor=anchor,
            window=window,
            plan=broken,
            joints=(joint("joint1"), joint("joint2")),
            max_accelerations=(2.0, 1.0),
            max_target_deviation_limits=(0.15, 0.15),
            raw_terminal_velocities=(0.1, 0.0),
            restricted_terminal_velocities=(0.1, 0.0),
            is_final=False,
            requested_window_ns=window.end_time_ns,
            curve_duration_ns=window.end_time_ns,
            algorithm_version=PRODUCER,
        )
    with pytest.raises(ContractError) as caught:
        PreparedFollowerPlan(
            anchor=anchor,
            window=window,
            plan=non_final_plan(),
            joints=(joint("joint1"), joint("joint2")),
            max_accelerations=(2.0, 1.0),
            max_target_deviation_limits=(1e-6, 0.15),
            raw_terminal_velocities=(0.1, 0.0),
            restricted_terminal_velocities=(0.1, 0.0),
            is_final=False,
            requested_window_ns=window.end_time_ns,
            curve_duration_ns=window.end_time_ns,
            algorithm_version=PRODUCER,
        )
    assert caught.value.reason == "FOLLOWER_DEVIATION_EXCEEDED"


def test_flags_validate_shape_and_types():
    flags = FollowerLimitFlags((True, False), (False, True), (True, True))
    assert flags.any_velocity_limited is True
    assert flags.any_acceleration_limited is True
    assert flags.all_boundaries_preserved is True
    with pytest.raises(ContractError):
        FollowerLimitFlags((True,), (False, True), (True, True))
    with pytest.raises(ContractError):
        FollowerLimitFlags((True, 1), (False, True), (True, True))


DEFAULT_DOCUMENT = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))


def changed(**changes: object) -> dict[str, object]:
    document = copy.deepcopy(DEFAULT_DOCUMENT)
    document.update(changes)
    return document


def test_follower_config_values_and_round_trip(tmp_path):
    from robot_backend_sim.config.follower_config import load_follower_config, parse_follower_config

    config = load_follower_config(CONFIG_PATH)
    assert config.target_count == 20
    assert config.max_target_deviations == (
        (0.30,) * 6 + (0.40, 0.04) + (0.30,) * 6 + (0.40, 0.04)
    )
    assert config.max_terminal_tail_duration_s == pytest.approx(3.0)
    assert config.max_terminal_tail_duration_ns == 3_000_000_000
    snapshot = config.to_document()
    snapshot["max_target_deviations"][0] = 99.0
    assert config.max_target_deviations[0] == 0.30
    assert parse_follower_config(config.to_document(), "<snapshot>") == config

    path = tmp_path / "follower.yaml"
    path.write_text(yaml.safe_dump(config.to_document(), sort_keys=False), encoding="utf-8")
    assert load_follower_config(path) == config


@pytest.mark.parametrize(
    ("mutate", "field"),
    (
        (lambda d: d.update({"unknown": 1}), "unknown"),
        (lambda d: d.pop("target_count"), "target_count"),
        (lambda d: d.update({"target_count": True}), "target_count"),
        (lambda d: d.update({"target_count": 1}), "target_count"),
        (lambda d: d.update({"target_count": 1.5}), "target_count"),
        (lambda d: d.update({"max_terminal_tail_duration_s": math.nan}),
         "max_terminal_tail_duration_s"),
        (lambda d: d.update({"max_terminal_tail_duration_s": math.inf}),
         "max_terminal_tail_duration_s"),
        (lambda d: d.update({"max_terminal_tail_duration_s": False}),
         "max_terminal_tail_duration_s"),
        (lambda d: d.update({"max_terminal_tail_duration_s": 0}), "max_terminal_tail_duration_s"),
        (lambda d: d.update({"max_terminal_tail_duration_s": 4}), "max_terminal_tail_duration_s"),
        (lambda d: d["max_target_deviations"].pop(), "max_target_deviations"),
        (lambda d: d["max_target_deviations"].__setitem__(0, True), "max_target_deviations[0]"),
        (lambda d: d["max_target_deviations"].__setitem__(0, math.nan), "max_target_deviations[0]"),
        (lambda d: d["max_target_deviations"].__setitem__(0, 0.0), "max_target_deviations[0]"),
        (lambda d: d["joint_names"].__setitem__(0, d["joint_names"][1]), "joint_names"),
    ),
)
def test_follower_config_strictly_rejects_bad_documents(mutate, field):
    from robot_backend_sim.config.follower_config import FollowerConfigError, parse_follower_config

    document = changed()
    mutate(document)
    with pytest.raises(FollowerConfigError) as caught:
        parse_follower_config(document, "<test>")
    assert caught.value.field == field


def test_follower_joint_order_is_bound_to_the_selected_robot_profile():
    from robot_backend_sim.config.follower_config import parse_follower_config

    document = changed()
    expected_order = tuple(document["joint_names"])
    document["joint_names"].reverse()
    config = parse_follower_config(document, "<test>")
    config.require_joint_order(tuple(document["joint_names"]))
    with pytest.raises(ContractError, match="robot profile"):
        config.require_joint_order(expected_order)


def test_follower_config_rejects_duplicate_keys(tmp_path):
    from robot_backend_sim.config.follower_config import FollowerConfigError, load_follower_config

    path = tmp_path / "duplicate.yaml"
    path.write_text(
        CONFIG_PATH.read_text(encoding="utf-8") + "\ntarget_count: 10\n",
        encoding="utf-8",
    )
    with pytest.raises(FollowerConfigError) as caught:
        load_follower_config(path)
    assert caught.value.field == "target_count"


def test_dto_rejects_forged_terminal_velocity_metadata():
    prepared, _ = make_non_final()
    with pytest.raises(ContractError, match="raw_terminal_velocities"):
        replace(prepared, raw_terminal_velocities=(0.0, 0.0))
    with pytest.raises(ContractError, match="restricted_terminal_velocities"):
        replace(prepared, restricted_terminal_velocities=(0.0, 0.0))


def test_generic_restriction_uses_each_joint_position_hard_boundary():
    joints = []
    initial_positions = []
    target_positions = []
    raw_velocities = []
    expected = []
    for index in range(16):
        if index < 8:
            minimum, maximum = 0.0, 0.08
            goal = maximum - 1e-4
            anchor = goal - 0.01
            raw = 0.5
            expected.append(math.sqrt(2.0 * 0.5 * (maximum - goal)))
        else:
            minimum, maximum = -1.0, 1.0
            goal = minimum + 1e-4
            anchor = goal + 0.01
            raw = -0.5
            expected.append(-math.sqrt(2.0 * 0.5 * (goal - minimum)))
        joints.append(JointSpec(f"j{index}", "rad", minimum, maximum, 0.04, anchor))
        initial_positions.append(anchor)
        target_positions.append(goal)
        raw_velocities.append(raw)
    restricted = restrict_terminal_velocities(
        raw_velocities,
        initial_positions,
        (0.0,) * 16,
        target_positions,
        joints,
        (0.5,) * 16,
        is_final=False,
    )
    assert restricted == pytest.approx(tuple(expected))

    at_limits = list(target_positions)
    for index in range(16):
        at_limits[index] = joints[index].maximum if index < 8 else joints[index].minimum
    stopped = restrict_terminal_velocities(
        raw_velocities,
        initial_positions,
        (0.0,) * 16,
        at_limits,
        joints,
        (0.5,) * 16,
        is_final=False,
    )
    assert stopped == (0.0,) * 16


def test_dto_rejects_endpoint_that_cannot_brake_inside_joint_boundary():
    targets = (
        Target(50_000_000, (0.99, 0.0)),
        Target(100_000_000, (0.99, 0.0)),
    )
    window = ActionWindow(
        observation_id=1,
        observation_time_ns=0,
        handoff_time_ns=0,
        source_targets=targets,
        target_count=2,
    )
    anchor = ChunkAnchor(
        "braking-test",
        0,
        0,
        ReferenceSample((0.9, 0.0), (0.0, 0.0), (0.0, 0.0)),
        "profile-hash",
    )
    plan = ReferencePlan(
        ("joint1", "joint2"),
        (
            CubicSegment(
                100_000_000,
                (0.9, 0.0),
                (0.0, 0.0),
                (25.0, 0.0),
                (-150.0, 0.0),
            ),
        ),
    )
    with pytest.raises(ContractError) as caught:
        PreparedFollowerPlan(
            anchor=anchor,
            window=window,
            plan=plan,
            joints=(
                joint("joint1", minimum=0.0, maximum=1.0, velocity=2.0),
                joint("joint2"),
            ),
            max_accelerations=(100.0, 1.0),
            max_target_deviation_limits=(0.15, 0.15),
            raw_terminal_velocities=(0.0, 0.0),
            restricted_terminal_velocities=(0.0, 0.0),
            is_final=False,
            requested_window_ns=window.end_time_ns,
            curve_duration_ns=window.end_time_ns,
            algorithm_version=PRODUCER,
        )
    assert caught.value.reason == "FOLLOWER_BRAKING_INFEASIBLE"
