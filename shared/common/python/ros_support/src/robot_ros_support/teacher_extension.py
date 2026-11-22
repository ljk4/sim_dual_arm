"""Shared ROS helper for loading scenario Teacher extension packages."""

from __future__ import annotations

import importlib
import re
from typing import Any

from robot_contracts.models import ContractError

__all__ = ["CAPABILITY_PATTERN", "teacher_extension_modules"]

CAPABILITY_PATTERN = re.compile(r"^([a-z][a-z0-9_]*)\.teacher_chunk$")


def teacher_extension_modules(capability: str) -> tuple[str, Any, Any]:
    """Resolve and verify ``(package, TeacherCapability, GetTeacherChunk)`` for a capability."""
    match = CAPABILITY_PATTERN.fullmatch(str(capability))
    if match is None:
        raise ContractError(
            "UNSUPPORTED_TEACHER_CAPABILITY",
            f"expected <package>.teacher_chunk, got {capability!r}",
        )
    package = match.group(1)
    try:
        messages = importlib.import_module(f"{package}.msg")
        services = importlib.import_module(f"{package}.srv")
        capability_type = messages.TeacherCapability
        service_type = services.GetTeacherChunk
    except (ImportError, AttributeError) as error:
        raise ContractError(
            "TEACHER_EXTENSION_MISSING",
            f"teacher capability {capability!r} requires the built "
            f"{package} extension: {error}",
        ) from error
    if str(capability_type.ID) != capability:
        raise ContractError(
            "UNSUPPORTED_TEACHER_CAPABILITY",
            f"selected {capability!r}, extension implements {str(capability_type.ID)!r}",
        )
    return package, capability_type, service_type
