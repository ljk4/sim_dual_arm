"""Generic simulation-side registration of a scenario Teacher extension.

The wire type is selected by the explicit capability identifier, never by a
scenario or robot name: ``<package>.teacher_chunk`` resolves to that ROS
package's generated ``TeacherCapability``/``GetTeacherChunk`` types.  The
generated modules are imported only when teacher mode is enabled.  Tasks and
the common contracts never import a concrete extension package.
"""

from __future__ import annotations

import importlib
import re
from typing import Any

from robot_contracts.models import ContractError

__all__ = ["teacher_extension_modules", "register_teacher_service"]

_CAPABILITY = re.compile(r"^([a-z][a-z0-9_]*)\.teacher_chunk$")


def teacher_extension_modules(capability: str) -> tuple[Any, Any]:
    """Resolve ``(TeacherCapability, GetTeacherChunk)`` for one capability."""
    match = _CAPABILITY.fullmatch(str(capability))
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
            f"teacher capability {capability!r} requires the built {package} "
            f"extension: {error}",
        ) from error
    if str(capability_type.ID) != capability:
        raise ContractError(
            "UNSUPPORTED_TEACHER_CAPABILITY",
            f"runtime selected {capability!r}, extension implements "
            f"{str(capability_type.ID)!r}",
        )
    return capability_type, service_type


def register_teacher_service(
    node: Any,
    *,
    capability: str,
    root: str,
    callback: Any,
    callback_group: Any,
) -> str:
    """Create ``<root>/get_task_chunk`` for the selected extension capability."""
    capability_type, service_type = teacher_extension_modules(capability)
    node.create_service(
        service_type,
        f"{root.rstrip('/')}/get_task_chunk",
        callback,
        callback_group=callback_group,
    )
    return str(capability_type.ID)
