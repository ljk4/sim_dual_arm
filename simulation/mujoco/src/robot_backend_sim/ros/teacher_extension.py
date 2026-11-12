"""Simulation-side adapter for the panda_stack Teacher extension.

The generated ``panda_stack_api_msgs`` modules are imported here, and only from
:func:`register_teacher_service`, which the backend calls when teacher mode is
actually enabled.  Ordinary control and the common contracts never import this
package or this module's ROS types.
"""

from __future__ import annotations

from typing import Any

__all__ = ["register_teacher_service"]


def register_teacher_service(
    node: Any,
    *,
    root: str,
    callback: Any,
    callback_group: Any,
) -> str:
    """Create ``<root>/get_task_chunk`` and return its capability identifier."""
    from panda_stack_api_msgs.msg import TeacherCapability
    from panda_stack_api_msgs.srv import GetTeacherChunk

    node.create_service(
        GetTeacherChunk,
        f"{root.rstrip('/')}/get_task_chunk",
        callback,
        callback_group=callback_group,
    )
    capability = str(TeacherCapability.ID)
    if not capability:
        raise RuntimeError("panda_stack_api_msgs declares an empty Teacher capability")
    return capability
