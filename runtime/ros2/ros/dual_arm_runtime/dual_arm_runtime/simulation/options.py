"""Runtime options and negotiation for simulation-only features."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Annotated, Any, Literal

from pydantic import Field
from robot_contracts.io import read_yaml_unique_mapping
from robot_contracts.models import ContractError
from robot_contracts.validation import (
    NonEmptyStr,
    NonNegativeInt,
    PositiveFloat,
    PositiveInt,
    StrictMappingModel,
)

INT64_MAX = 2**63 - 1
MAX_CHUNKS = 2048
MAX_TASK_BUDGET_NS = 300_000_000_000
STREAM_CHUNK_SIZE = 60
STREAM_PERIOD_NS = 50_000_000

if TYPE_CHECKING:
    from dual_arm_runtime.runtime_config import RuntimeRecipe


class ResetConfig(StrictMappingModel):
    enabled: bool = False
    seed: Annotated[NonNegativeInt, Field(le=INT64_MAX)] = 0


class TeacherConfig(StrictMappingModel):
    """Program-teacher stream identity, independent of policy RPC settings."""

    chunk_size: Annotated[int, Field(strict=True, ge=STREAM_CHUNK_SIZE, le=STREAM_CHUNK_SIZE)] = (
        STREAM_CHUNK_SIZE
    )
    action_period_ns: Annotated[
        int, Field(strict=True, ge=STREAM_PERIOD_NS, le=STREAM_PERIOD_NS)
    ] = STREAM_PERIOD_NS
    initial_lead_ns: PositiveInt = 250_000_000
    max_chunks: Annotated[PositiveInt, Field(le=MAX_CHUNKS)] = 128
    total_sim_deadline_ns: PositiveInt = 30_000_000_000
    total_wall_deadline_s: PositiveFloat = 60.0
    prepare_timeout_s: PositiveFloat = 12.0
    start_timeout_s: PositiveFloat = 5.0
    chunk_deadline_s: PositiveFloat = 2.0
    poll_interval_s: PositiveFloat = 0.02
    termination_mode: Literal["finite_chunks", "task"] = "task"
    task_stop_margin_s: PositiveFloat = 1.0
    task_sim_budget_ns: Annotated[PositiveInt, Field(le=MAX_TASK_BUDGET_NS)] = 30_000_000_000
    task_id: NonEmptyStr = "teacher"


def task_enabled(recipe: RuntimeRecipe) -> bool:
    return recipe.teacher is not None or (
        recipe.policy is not None and recipe.policy.termination_mode == "task"
    )


def required_capabilities(
    *, reset_enabled: bool, task_enabled: bool, teacher_api: str | None
) -> tuple[str, ...]:
    capabilities = ("simulation_reset",) if reset_enabled else ()
    if task_enabled:
        capabilities += ("task_plugin", "task_state")
    if teacher_api is not None:
        capabilities += (teacher_api,)
    return capabilities


def require_simulation_backend(backend_kind: str) -> None:
    if backend_kind != "mujoco":
        raise ContractError(
            "UNSUPPORTED_CAPABILITY",
            f"simulation features require a MuJoCo backend (received {backend_kind!r})",
        )


def check_teacher_budget(
    runtime_document: dict[str, Any],
    backend_document: dict[str, Any],
    backend_path: Path,
    *,
    error_cls: type[Exception] = ValueError,
) -> None:
    """Runtime stream clock and budget must equal the simulation task judge."""
    teacher = runtime_document.get("teacher")
    robot = backend_document.get("robot")
    if not isinstance(teacher, dict) or "task_sim_budget_ns" not in teacher:
        return
    if not isinstance(robot, dict) or not isinstance(robot.get("task"), str):
        raise error_cls(f"simulation teacher recipe must declare robot.task ({backend_path})")
    task_path = (backend_path.parent / robot["task"]).resolve()
    if not task_path.is_file():
        raise error_cls(f"simulation task recipe is missing: {task_path}")
    task = read_yaml_unique_mapping(
        task_path, label="simulation task recipe", error_cls=error_cls
    )
    success = task.get("success") if isinstance(task, dict) else None
    if not isinstance(success, dict) or "max_task_sim_duration_s" not in success:
        return
    task_teacher = task.get("teacher") if isinstance(task, dict) else None
    if not isinstance(task_teacher, dict):
        raise error_cls(f"simulation task recipe is incomplete: {task_path}")
    try:
        budget_ns = round(float(success["max_task_sim_duration_s"]) * 1e9)
        chunk_size = int(task_teacher["chunk_size"])
        period_ns = int(task_teacher["action_period_ns"])
        runtime_budget_ns = int(teacher["task_sim_budget_ns"])
    except (KeyError, TypeError, ValueError) as error:
        raise error_cls(f"teacher budget fields are not numeric: {error}") from error
    if runtime_budget_ns != budget_ns:
        raise error_cls(
            f"runtime teacher.task_sim_budget_ns {runtime_budget_ns} differs from "
            f"simulation success.max_task_sim_duration_s ({budget_ns})"
        )
    if int(teacher.get("chunk_size", -1)) != chunk_size:
        raise error_cls("runtime teacher.chunk_size differs from the simulation task")
    if int(teacher.get("action_period_ns", -1)) != period_ns:
        raise error_cls(
            "runtime teacher.action_period_ns differs from the simulation task"
        )
