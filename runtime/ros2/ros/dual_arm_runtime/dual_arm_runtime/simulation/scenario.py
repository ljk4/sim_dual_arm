"""Validate simulation recipe references before Runtime startup."""

from __future__ import annotations

from typing import Any

import yaml
from robot_contracts.binding import ResolvedBinding


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
    try:
        backend = yaml.safe_load(backend_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(f"cannot read simulation teacher recipe: {error}") from error
    robot = backend.get("robot") if isinstance(backend, dict) else None
    if not isinstance(robot, dict) or not isinstance(robot.get("task"), str):
        raise ValueError(f"simulation teacher recipe has no robot.task ({backend_path})")
    task_path = (backend_path.parent / robot["task"]).resolve()
    if not task_path.is_file():
        raise ValueError(f"simulation task recipe is missing: {task_path}")
    try:
        task = yaml.safe_load(task_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(f"cannot read simulation task recipe: {error}") from error
    success = task.get("success") if isinstance(task, dict) else None
    task_teacher = task.get("teacher") if isinstance(task, dict) else None
    if not isinstance(success, dict) or "max_task_sim_duration_s" not in success:
        return
    if not isinstance(task_teacher, dict):
        raise ValueError(f"simulation task recipe is incomplete: {task_path}")
    try:
        budget_ns = round(float(success["max_task_sim_duration_s"]) * 1e9)
        chunk_size = int(task_teacher["chunk_size"])
        period_ns = int(task_teacher["action_period_ns"])
        runtime_budget_ns = int(teacher["task_sim_budget_ns"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"teacher budget fields are not numeric: {error}") from error
    if runtime_budget_ns != budget_ns:
        raise ValueError(
            f"runtime teacher.task_sim_budget_ns {runtime_budget_ns} differs from "
            f"simulation success.max_task_sim_duration_s ({budget_ns})"
        )
    if int(teacher.get("chunk_size", -1)) != chunk_size:
        raise ValueError("runtime teacher.chunk_size differs from the simulation task")
    if int(teacher.get("action_period_ns", -1)) != period_ns:
        raise ValueError(
            "runtime teacher.action_period_ns differs from the simulation task"
        )
