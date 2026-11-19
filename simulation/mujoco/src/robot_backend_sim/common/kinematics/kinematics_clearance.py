"""Collision-pair enumeration and conservative clearance measurement."""

from __future__ import annotations

import math
from dataclasses import dataclass


class KinematicsError(RuntimeError):
    """A stable kinematics/clearance failure with a bounded reason."""

    def __init__(self, reason: str, message: str = "") -> None:
        super().__init__(message or reason)
        self.reason = reason
        self.message = message or reason


@dataclass(frozen=True, slots=True)
class ClearancePoint:
    distance_m: float
    geom1: int | None
    geom2: int | None
    body1: str | None
    body2: str | None
    geom1_name: str | None
    geom2_name: str | None
    sample_index: int
    fraction: float
    measured_pairs: int
    broadphase_skipped: int
    lower_bound: bool


@dataclass(frozen=True, slots=True)
class PathResult:
    segment: str
    sample_count: int
    requested_max_step_rad: float
    actual_max_step_rad: float
    minimum: ClearancePoint
    passed: bool
    elapsed_s: float
    failure_reason: str | None = None
    failure_message: str | None = None


@dataclass(frozen=True, slots=True)
class CollisionPair:
    geom1: int | None
    geom2: int | None
    body1: str | None
    body2: str | None
    geom1_name: str | None
    geom2_name: str | None
    category: str


@dataclass(frozen=True, slots=True)
class CollisionPairSet:
    pairs: tuple[CollisionPair, ...]
    categories: dict[str, int]
    exclusions: dict[str, int]
    total_physical_pairs: int
    explicit_pair_count: int


def _geometry_names(model: object) -> tuple[list[str], list[str]]:
    body_names = [str(model.body(index).name) or f"body[{index}]" for index in range(model.nbody)]
    geom_names = [str(model.geom(index).name) or f"geom[{index}]" for index in range(model.ngeom)]
    return body_names, geom_names


def _body_movable(model: object, body_id: int) -> bool:
    current = body_id
    while current > 0:
        if int(model.body_jntnum[current]) > 0:
            return True
        current = int(model.body_parentid[current])
    return False


def _matches_prefix(name: str, prefixes: tuple[str, ...]) -> bool:
    return any(name.startswith(prefix) for prefix in prefixes)


def _side_of(name: str, sides: dict[str, tuple[str, ...]]) -> str | None:
    for side, prefixes in sides.items():
        if _matches_prefix(name, prefixes):
            return side
    return None


def _robot_body(name: str, prefixes: tuple[str, ...]) -> bool:
    return _matches_prefix(name, prefixes)


def _geometry_state(model: object):
    body_names, geom_names = _geometry_names(model)
    return {
        "body_names": body_names,
        "geom_names": geom_names,
        "geom_body": [int(value) for value in model.geom_bodyid],
        "contype": [int(value) for value in model.geom_contype],
        "conaffinity": [int(value) for value in model.geom_conaffinity],
        "welded": [int(value) for value in model.body_weldid],
        "parent": [int(value) for value in model.body_parentid],
        "movable": [_body_movable(model, body) for body in range(model.nbody)],
        "exclude_signatures": {int(value) for value in model.exclude_signature},
    }


def _explicit_pairs(model: object) -> set[tuple[int, int]]:
    explicit: set[tuple[int, int]] = set()
    for index in range(model.npair):
        first = int(model.pair_geom1[index])
        second = int(model.pair_geom2[index])
        explicit.add((min(first, second), max(first, second)))
    return explicit


def _pair_exclusion(
    first: int,
    second: int,
    state: dict,
    explicit: set[tuple[int, int]],
) -> tuple[str | None, bool]:
    geom_body = state["geom_body"]
    first_body = geom_body[first]
    second_body = geom_body[second]
    if first_body == second_body:
        return "same_body", False
    welded = state["welded"]
    if welded[first_body] == welded[second_body]:
        return "welded_body", False
    key = (first, second)
    is_explicit = key in explicit
    contype = state["contype"]
    conaffinity = state["conaffinity"]
    if not is_explicit and not (
        contype[first] & conaffinity[second] or contype[second] & conaffinity[first]
    ):
        return "visual_or_no_affinity", False
    if not is_explicit:
        # Match mink/MuJoCo's welded-body parent-child rule rather than
        # only the direct body parent link.
        parent = state["parent"]
        first_weld = welded[first_body]
        second_weld = welded[second_body]
        first_parent_weld = welded[parent[first_weld]]
        second_parent_weld = welded[parent[second_weld]]
        if first_weld == second_parent_weld or second_weld == first_parent_weld:
            return "parent_child", False
    if not is_explicit:
        signature = (min(first_body, second_body) << 16) | max(first_body, second_body)
        if signature in state["exclude_signatures"]:
            return "explicit_exclude", False
    return None, is_explicit


def _pair_category(
    first_name: str,
    second_name: str,
    *,
    robot_prefixes: tuple[str, ...],
    robot_sides: dict[str, tuple[str, ...]],
    object_names: tuple[str, ...],
) -> str | None:
    first_robot = _robot_body(first_name, robot_prefixes)
    second_robot = _robot_body(second_name, robot_prefixes)
    if first_robot and second_robot:
        first_side = _side_of(first_name, robot_sides)
        second_side = _side_of(second_name, robot_sides)
        return (
            "robot_self"
            if first_side is not None and first_side == second_side
            else "robot_cross"
        )
    if first_robot or second_robot:
        other = second_name if first_robot else first_name
        return "robot_object" if other in object_names else "robot_scene"
    return None


def build_collision_pairs(
    model: object,
    *,
    robot_prefixes: tuple[str, ...],
    robot_sides: dict[str, tuple[str, ...]],
    object_names: tuple[str, ...],
) -> CollisionPairSet:
    """Enumerate effective robot-involving physical geom pairs natively.

    The robot adapter passes its own body groups.  All native default
    exclusions are accounted for explicitly; no global "robot collision off"
    switch is used and no model margin is modified.
    """
    sides = dict(robot_sides)
    state = _geometry_state(model)
    body_names = state["body_names"]
    geom_names = state["geom_names"]
    movable = state["movable"]
    explicit = _explicit_pairs(model)
    categories: dict[str, int] = {}
    exclusions: dict[str, int] = {}
    pairs: list[CollisionPair] = []
    physical = 0
    explicit_count = 0
    for first in range(model.ngeom):
        for second in range(first + 1, model.ngeom):
            excluded, is_explicit = _pair_exclusion(first, second, state, explicit)
            if excluded is not None:
                exclusions[excluded] = exclusions.get(excluded, 0) + 1
                continue
            if is_explicit:
                explicit_count += 1
            physical += 1
            first_body = state["geom_body"][first]
            second_body = state["geom_body"][second]
            first_name = body_names[first_body]
            second_name = body_names[second_body]
            if not _robot_body(first_name, robot_prefixes) and not _robot_body(
                second_name, robot_prefixes
            ):
                exclusions["non_robot_pair"] = exclusions.get("non_robot_pair", 0) + 1
                continue
            if not movable[first_body] and not movable[second_body]:
                exclusions["fixed_connection"] = exclusions.get("fixed_connection", 0) + 1
                continue
            category = _pair_category(
                first_name,
                second_name,
                robot_prefixes=robot_prefixes,
                robot_sides=sides,
                object_names=object_names,
            )
            if category is None:  # pragma: no cover - guarded above
                continue
            pairs.append(
                CollisionPair(
                    geom1=first,
                    geom2=second,
                    body1=first_name,
                    body2=second_name,
                    geom1_name=geom_names[first],
                    geom2_name=geom_names[second],
                    category=category,
                )
            )
            categories[category] = categories.get(category, 0) + 1
    if not pairs:
        raise KinematicsError("collision_pairs_empty", "no effective robot collision pairs found")
    return CollisionPairSet(
        pairs=tuple(pairs),
        categories=categories,
        exclusions=exclusions,
        total_physical_pairs=physical,
        explicit_pair_count=explicit_count,
    )


def build_object_collision_pairs(
    model: object,
    object_names: tuple[str, ...],
    *,
    excluded_body_names: tuple[str, ...] = (),
) -> CollisionPairSet:
    """Enumerate carried-object pairs outside the object and excluded bodies.

    A carried object is checked against every other collidable body: the
    environment and robot links, except the named bodies that are allowed to
    hold it.  The exclusion set is explicit because it changes per path
    segment (left- or right-hand carry).
    """
    if not object_names:
        raise KinematicsError("collision_pairs_empty", "no carried object was configured")
    state = _geometry_state(model)
    body_names = state["body_names"]
    geom_names = state["geom_names"]
    excluded = frozenset(excluded_body_names)
    object_ids = set()
    for name in object_names:
        try:
            object_ids.add(int(model.body(name).id))
        except KeyError as error:
            raise KinematicsError(
                "scene_model_missing_name", f"carried object {name!r} is missing"
            ) from error
    explicit = _explicit_pairs(model)
    pairs: list[CollisionPair] = []
    exclusions: dict[str, int] = {}
    physical = 0
    for first in range(model.ngeom):
        for second in range(first + 1, model.ngeom):
            first_body = int(state["geom_body"][first])
            second_body = int(state["geom_body"][second])
            if first_body in object_ids and second_body in object_ids:
                exclusions["same_object"] = exclusions.get("same_object", 0) + 1
                continue
            if first_body in object_ids:
                other_body = second_body
            elif second_body in object_ids:
                other_body = first_body
            else:
                continue
            if body_names[other_body] in excluded:
                exclusions["holding_finger"] = exclusions.get("holding_finger", 0) + 1
                continue
            excluded_reason, _ = _pair_exclusion(first, second, state, explicit)
            if excluded_reason is not None:
                exclusions[excluded_reason] = exclusions.get(excluded_reason, 0) + 1
                continue
            physical += 1
            pairs.append(
                CollisionPair(
                    geom1=first,
                    geom2=second,
                    body1=body_names[first_body],
                    body2=body_names[second_body],
                    geom1_name=geom_names[first],
                    geom2_name=geom_names[second],
                    category="carried_object",
                )
            )
    if not pairs:
        raise KinematicsError(
            "collision_pairs_empty", "no carried-object collision pairs found"
        )
    return CollisionPairSet(
        pairs=tuple(pairs),
        categories={"carried_object": len(pairs)},
        exclusions=exclusions,
        total_physical_pairs=physical,
        explicit_pair_count=0,
    )


def _frozen_deviation(qpos: object, initial: object, free_addresses: set[int]) -> float:
    deviation = 0.0
    for index in range(len(initial)):  # type: ignore[arg-type]
        if index in free_addresses:
            continue
        difference = abs(float(qpos[index]) - float(initial[index]))  # type: ignore[index]
        if difference > deviation:
            deviation = difference
    return deviation


def _check_finite_and_frozen(
    model: object,
    qpos: object,
    initial_qpos: object,
    free_qpos_addresses: tuple[int, ...],
    joint_ids: tuple[int, ...],
) -> None:
    import numpy as np

    values = np.asarray(qpos, dtype=np.float64)
    if values.shape != (model.nq,) or not np.isfinite(values).all():
        raise KinematicsError(
            "ik_state_invalid", "solved qpos is not a finite vector of model.nq values"
        )
    free_addresses = set(free_qpos_addresses)
    deviation = _frozen_deviation(values, np.asarray(initial_qpos), free_addresses)
    if deviation > 1e-8:
        raise KinematicsError(
            "frozen_qpos_changed",
            f"frozen qpos changed by {deviation:.3e} (quaternion rounding allowed)",
        )
    for joint_id in joint_ids:
        if int(model.jnt_limited[joint_id]) == 0:
            continue
        address = int(model.jnt_qposadr[joint_id])
        value = float(values[address])
        lower = float(model.jnt_range[joint_id, 0])
        upper = float(model.jnt_range[joint_id, 1])
        if value < lower - 1e-9 or value > upper + 1e-9:
            raise KinematicsError(
                "joint_limit_violation",
                f"joint {model.joint(joint_id).name!r} value {value:.6f} outside "
                f"[{lower:.6f}, {upper:.6f}]",
            )


def measure_clearance(
    model: object,
    data: object,
    pair_set: CollisionPairSet,
    *,
    detection_distance_m: float,
    sample_index: int,
    fraction: float,
) -> ClearancePoint:
    """Measure the smallest signed distance in a conservative broadphase."""
    import mujoco
    import numpy as np

    detection = float(detection_distance_m)
    if not math.isfinite(detection) or detection <= 0.0:
        raise KinematicsError("clearance_config_invalid", "detection distance must be positive")
    positions = np.asarray(data.geom_xpos)
    radii = np.asarray(model.geom_rbound)
    minimum = detection
    worst: CollisionPair | None = None
    measured = 0
    if not np.isfinite(positions).all() or not np.isfinite(radii).all():
        raise KinematicsError("clearance_non_finite", "geom positions and bounds must be finite")
    pair_ids = np.asarray([(pair.geom1, pair.geom2) for pair in pair_set.pairs], dtype=np.intp)
    first, second = pair_ids[:, 0], pair_ids[:, 1]
    bounded = (radii[first] > 0.0) & (radii[second] > 0.0)
    displacement = positions[first] - positions[second]
    distance_squared = np.einsum("ij,ij->i", displacement, displacement)
    maximum_radius = radii[first] + radii[second] + detection
    # Unbounded geoms are always queried; finite bounding spheres only cull
    # pairs that provably cannot be within the detection distance.
    candidates = np.flatnonzero(~bounded | (distance_squared <= maximum_radius**2))
    skipped = len(pair_set.pairs) - len(candidates)
    for index in candidates:
        pair = pair_set.pairs[int(index)]
        fromto = np.zeros(6, dtype=np.float64)
        distance = float(
            mujoco.mj_geomDistance(model, data, pair.geom1, pair.geom2, detection, fromto)
        )
        measured += 1
        if not math.isfinite(distance):
            raise KinematicsError(
                "clearance_non_finite",
                f"mj_geomDistance returned a non-finite value for "
                f"{pair.geom1_name}/{pair.geom2_name}",
            )
        if distance < minimum:
            minimum = distance
            worst = pair
    if worst is None:
        return ClearancePoint(
            distance_m=detection,
            geom1=None,
            geom2=None,
            body1=None,
            body2=None,
            geom1_name=None,
            geom2_name=None,
            sample_index=sample_index,
            fraction=fraction,
            measured_pairs=measured,
            broadphase_skipped=skipped,
            lower_bound=True,
        )
    return ClearancePoint(
        distance_m=minimum,
        geom1=worst.geom1,
        geom2=worst.geom2,
        body1=worst.body1,
        body2=worst.body2,
        geom1_name=worst.geom1_name,
        geom2_name=worst.geom2_name,
        sample_index=sample_index,
        fraction=fraction,
        measured_pairs=measured,
        broadphase_skipped=skipped,
        lower_bound=minimum >= detection - 1e-12,
    )
