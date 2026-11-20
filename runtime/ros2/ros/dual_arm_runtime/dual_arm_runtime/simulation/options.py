"""Runtime options and negotiation for simulation-only features."""

from __future__ import annotations

from typing import TYPE_CHECKING, Annotated, Literal

from pydantic import Field
from robot_contracts.models import ContractError
from robot_contracts.validation import (
    NonEmptyStr,
    NonNegativeInt,
    PositiveFloat,
    PositiveInt,
    StrictMappingModel,
)

from dual_arm_runtime.runtime_limits import (
    INT64_MAX,
    MAX_CHUNKS,
    MAX_TASK_BUDGET_NS,
    STREAM_CHUNK_SIZE,
    STREAM_PERIOD_NS,
)

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
