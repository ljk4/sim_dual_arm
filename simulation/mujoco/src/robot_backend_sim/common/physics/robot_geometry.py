"""Robot-adapter description consumed by offline planning and observation.

The robot assembly owns the mapping from public sides to compiled element
names.  Task planning and contact observation consume this description and do
not infer a robot family from body-name prefixes.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

__all__ = ["ArmChain", "RobotGeometry"]


@dataclass(frozen=True, slots=True)
class ArmChain:
    """One arm's named joints and the TCP site used as its kinematic frame."""

    side: str
    joint_names: tuple[str, ...]
    tcp_site: str


@dataclass(frozen=True, slots=True)
class RobotGeometry:
    """Named robot geometry shared by IK, contact facts and path clearance."""

    chains: tuple[ArmChain, ...]
    body_prefixes: tuple[str, ...]
    side_prefixes: Mapping[str, tuple[str, ...]]
    finger_bodies: Mapping[str, tuple[tuple[str, str], ...]]
