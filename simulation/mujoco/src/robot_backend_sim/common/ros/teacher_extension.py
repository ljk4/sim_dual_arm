"""Generic simulation-side registration of a scenario Teacher extension.

The wire type is selected by the explicit capability identifier, never by a
scenario or robot name: ``<package>.teacher_chunk`` resolves to that ROS
package's generated ``TeacherCapability``/``GetTeacherChunk`` types.  The
generated modules are imported only when teacher mode is enabled.  Tasks and
the common contracts never import a concrete extension package.
"""

from __future__ import annotations

from typing import Any

from robot_ros_support.teacher_extension import teacher_extension_modules as _resolve_modules

__all__ = ["teacher_extension_modules", "register_teacher_service"]


def teacher_extension_modules(capability: str) -> tuple[Any, Any]:
    """Resolve ``(TeacherCapability, GetTeacherChunk)`` for one capability."""
    _package, capability_type, service_type = _resolve_modules(capability)
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
