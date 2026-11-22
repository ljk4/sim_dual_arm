"""Validate simulation recipe references before Runtime startup."""

from __future__ import annotations

from typing import Any

from robot_contracts.binding import ResolvedBinding
from robot_contracts.io import read_yaml_unique_mapping

from dual_arm_runtime.simulation.options import check_teacher_budget


def configure_simulation_scenario(
    document: dict[str, Any], resolved: ResolvedBinding, *, role: str
) -> str | None:
    reset = document.get("reset")
    policy = document.get("policy")
    uses_simulation = (
        role == "teacher"
        or document.get("action_source") == "teacher"
        or (isinstance(reset, dict) and reset.get("enabled") is True)
        or (isinstance(policy, dict) and policy.get("termination_mode") == "task")
    )
    if uses_simulation and resolved.binding.backend.kind != "simulation":
        raise ValueError("reset, managed tasks, and Teacher require a simulation binding")
    if role != "teacher":
        return None
    capability = _check_teacher_capability(document, resolved)
    _check_teacher_budget(document, resolved)
    return capability


def _check_teacher_capability(
    document: dict[str, Any], resolved: ResolvedBinding
) -> str:
    teacher_api = resolved.binding.interfaces.teacher_api
    if not teacher_api:
        raise ValueError(
            "scenario teacher selection requires interfaces.teacher_api"
        )
    declared = document.get("teacher_api")
    if declared is not None and declared != teacher_api:
        raise ValueError(
            f"runtime recipe teacher_api {declared!r} differs from binding {teacher_api!r}"
        )
    document["teacher_api"] = teacher_api
    return teacher_api


def _check_teacher_budget(document: dict[str, Any], resolved: ResolvedBinding) -> None:
    teacher = document.get("teacher")
    if not isinstance(teacher, dict) or "task_sim_budget_ns" not in teacher:
        return
    reference = resolved.references.get("recipes.simulation_teacher")
    if reference is None:
        raise ValueError(
            "scenario declares no simulation_teacher recipe for teacher selection"
        )
    backend_path = reference.path
    backend = read_yaml_unique_mapping(
        backend_path, label="simulation teacher recipe", error_cls=ValueError
    )
    check_teacher_budget(document, backend, backend_path, error_cls=ValueError)
