"""Strict, lightweight, ROS-free scenario binding loading and path resolution.

A binding is a single YAML document that selects existing public profiles and
end-private recipes.  This module validates that document, resolves every
declared path against the binding directory and explicitly declared roots,
and evaluates stage requirements.  ``references`` name independent dependency
bindings for explicit cross-scenario reuse; they are loaded and cycle-checked
but never merged into the referring binding.  This module never imports a
scenario implementation, loads model weights, connects to ROS, starts a
process or writes a file.
"""

from __future__ import annotations

import os
import re
from collections.abc import Mapping
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from types import MappingProxyType
from typing import Annotated, Any, Literal

from pydantic import BeforeValidator, Field, model_validator

from .io import sha256_file
from .models import ContractError
from .profiles import (
    load_document,
    profile_from_document,
    semantic_hash,
    validate_bindings,
)
from .validation import (
    NonEmptyStr,
    StrictContractModel,
    as_tuple,
)

__all__ = [
    "BindingError",
    "BindingStage",
    "ROBOT_REFERENCE_FIELDS",
    "ResolvedBinding",
    "ResolvedReference",
    "ScenarioBinding",
    "load_binding",
    "resolve_binding_path",
    "stage_requirements",
]

ROBOT_REFERENCE_FIELDS = ("profile", "schemas", "scene", "control", "stream", "follower", "task")

PathList = Annotated[tuple[NonEmptyStr, ...], BeforeValidator(as_tuple)]
# The template directory is a documentation scenario, so its id may start
# with an underscore.  Paths are never derived from this value.
ScenarioId = Annotated[
    str,
    Field(
        strict=True,
        min_length=1,
        max_length=128,
        pattern=r"^[A-Za-z_][A-Za-z0-9_.:-]{0,127}$",
    ),
]
_SCENARIO_ID_PATTERN = re.compile(r"^[A-Za-z_][A-Za-z0-9_.:-]{0,127}$")
RESERVED_ROBOT_IDS = frozenset({"build"})
BackendKind = Literal["simulation", "hardware"]
ReferenceKind = Literal["file", "directory"]

_RECIPE_FIELDS = (
    "simulation_teacher",
    "simulation_policy",
    "runtime_teacher",
    "runtime_policy",
    "campaign",
    "dataset",
    "training",
    "policy_service",
    "evaluation",
)
_PROFILE_FIELDS = ("robot", "observation", "action")
_BASE_REQUIREMENTS = tuple(f"profiles.{name}" for name in _PROFILE_FIELDS)
_STAGE_REQUIREMENTS: dict[str, tuple[str, ...]] = {
    "collection": ("recipes.runtime_teacher", "recipes.campaign"),
    "learning": ("learning.adapter", "recipes.dataset", "recipes.training"),
    "inference": (
        "learning.adapter",
        "recipes.runtime_policy",
        "recipes.policy_service",
        "interfaces.policy_api",
    ),
}
_SIMULATION_REQUIREMENTS: dict[str, tuple[str, ...]] = {
    "collection": ("recipes.simulation_teacher",),
    "inference": ("recipes.simulation_policy",),
}


class BindingError(ContractError):
    """A binding document or one of its declared references is invalid.

    ``reason`` is one of ``BINDING_SCHEMA``, ``BINDING_REFERENCE``,
    ``BINDING_CYCLE`` or ``BINDING_STAGE``.
    """


class BindingStage(str, Enum):
    collection = "collection"
    learning = "learning"
    inference = "inference"


class BackendSpec(StrictContractModel):
    kind: BackendKind
    implementation: NonEmptyStr
    task_type: NonEmptyStr | None = None


class RuntimeSpec(StrictContractModel):
    implementation: NonEmptyStr


class LearningSpec(StrictContractModel):
    implementation: NonEmptyStr
    adapter: NonEmptyStr | None = None


class ProfilesSpec(StrictContractModel):
    robot: NonEmptyStr
    observation: NonEmptyStr
    action: NonEmptyStr


class RecipesSpec(StrictContractModel):
    simulation_teacher: NonEmptyStr | None = None
    simulation_policy: NonEmptyStr | None = None
    runtime_teacher: NonEmptyStr | None = None
    runtime_policy: NonEmptyStr | None = None
    campaign: NonEmptyStr | None = None
    dataset: NonEmptyStr | None = None
    training: NonEmptyStr | None = None
    policy_service: NonEmptyStr | None = None
    evaluation: NonEmptyStr | None = None


class InterfacesSpec(StrictContractModel):
    common_ros: PathList = ()
    extension_ros: PathList = ()
    teacher_api: NonEmptyStr | None = None
    policy_api: NonEmptyStr | None = None


class ScenarioBinding(StrictContractModel):
    """A strictly validated scenario binding.

    A scenario is the ``(robot_id, backend_id, task_id)`` triple that owns
    ``<shared>/scenarios/<robot_id>/<backend_id>/<task_id>/binding.yaml``.
    ``scenario_id`` is the canonical ``"<robot_id>/<backend_id>/<task_id>"``
    display identifier for summaries and logs.
    """

    robot_id: ScenarioId
    task_id: ScenarioId
    description: NonEmptyStr | None = None
    backend: BackendSpec
    runtime: RuntimeSpec
    learning: LearningSpec
    profiles: ProfilesSpec
    recipes: RecipesSpec = Field(default_factory=RecipesSpec)
    interfaces: InterfacesSpec = Field(default_factory=InterfacesSpec)
    source_roots: PathList = ()
    artifact_roots: PathList = ()

    @model_validator(mode="after")
    def _validate_robot_id(self) -> ScenarioBinding:
        if self.robot_id in RESERVED_ROBOT_IDS:
            raise BindingError(
                "BINDING_SCHEMA",
                f"robot_id: {self.robot_id!r} is reserved and cannot be used as a scenario root",
            )
        return self

    @property
    def scenario_id(self) -> str:
        return f"{self.robot_id}/{self.backend.implementation}/{self.task_id}"


@dataclass(frozen=True)
class ResolvedReference:
    """One declared file or directory reference after root and type checks."""

    field: str
    declared: str
    path: Path
    kind: ReferenceKind
    sha256: str | None = None
    semantic_sha256: str | None = None

    def record(self) -> dict[str, Any]:
        """Declared identity of this reference for selection manifests."""
        record: dict[str, Any] = {
            "field": self.field,
            "declared": self.declared,
            "path": str(self.path),
            "sha256": self.sha256,
        }
        if self.semantic_sha256 is not None:
            record["semantic_sha256"] = self.semantic_sha256
        return record


@dataclass(frozen=True)
class ResolvedBinding:
    """A validated binding with resolved paths and stage requirements."""

    root: Path
    path: Path
    sha256: str
    binding: ScenarioBinding
    source_roots: tuple[Path, ...]
    artifact_roots: tuple[Path, ...]
    references: Mapping[str, ResolvedReference]

    def requirements(self, stage: BindingStage | str) -> tuple[str, ...]:
        return stage_requirements(self.binding, stage)

    def missing(self, stage: BindingStage | str) -> tuple[str, ...]:
        stage = _coerce_stage(stage)
        return tuple(
            field for field in self.requirements(stage) if not self._satisfied(field)
        )

    def validate_stage(self, stage: BindingStage | str) -> None:
        stage = _coerce_stage(stage)
        missing = self.missing(stage)
        if missing:
            raise BindingError(
                "BINDING_STAGE",
                f"{self.path}: {stage.value}: missing required fields: {', '.join(missing)}",
            )

    def _satisfied(self, field: str) -> bool:
        section, name = field.split(".", 1)
        return getattr(getattr(self.binding, section), name) is not None


def stage_requirements(binding: ScenarioBinding, stage: BindingStage | str) -> tuple[str, ...]:
    """Return the ordered structural requirements for one stage."""
    stage = _coerce_stage(stage)
    requirements = (*_BASE_REQUIREMENTS, *_STAGE_REQUIREMENTS[stage.value])
    if binding.backend.kind == "simulation":
        requirements += _SIMULATION_REQUIREMENTS.get(stage.value, ())
    return tuple(dict.fromkeys(requirements))


def _coerce_stage(stage: BindingStage | str) -> BindingStage:
    if isinstance(stage, BindingStage):
        return stage
    try:
        return BindingStage(stage)
    except (TypeError, ValueError) as error:
        choices = ", ".join(item.value for item in BindingStage)
        raise BindingError(
            "BINDING_STAGE", f"{stage!r}: unknown stage; expected {choices}"
        ) from error


def _resolved(path: Path) -> Path:
    return path.resolve(strict=False)


def _is_within(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
    except ValueError:
        return False
    return True


def _inside_roots(path: Path, roots: tuple[Path, ...]) -> bool:
    return any(_is_within(path, root) for root in roots)


def _resolve_artifact_path(path: Path, binding_dir: Path) -> Path:
    if path.is_absolute():
        data_root = os.environ.get("DATA_ROOT")
        container_data = Path("/data")
        if data_root and (path == container_data or _is_within(path, container_data)):
            return _resolved(Path(data_root) / path.relative_to(container_data))
        return _resolved(path)
    return _resolved(binding_dir / path)


def _root_list(
    source: Path, roots: tuple[str, ...], kind: str, binding_dir: Path
) -> tuple[Path, ...]:
    resolved: list[Path] = []
    for index, declared in enumerate(roots):
        field = f"{kind}_roots[{index}]"
        path = Path(declared)
        if kind == "source" and path.is_absolute():
            raise BindingError(
                "BINDING_REFERENCE",
                f"{source}: {field}: source roots must be relative to the binding directory",
            )
        candidate = (
            _resolve_artifact_path(path, binding_dir)
            if kind == "artifact"
            else _resolved(binding_dir / path)
        )
        if not candidate.is_dir():
            raise BindingError(
                "BINDING_REFERENCE", f"{source}: {field}: root is not a directory: {candidate}"
            )
        if candidate not in resolved:
            resolved.append(candidate)
    return tuple(resolved)


def _resolve_source_reference(
    source: Path,
    field: str,
    declared: str,
    *,
    kind: ReferenceKind,
    roots: tuple[Path, ...],
    binding_dir: Path,
) -> Path:
    path = Path(declared)
    if path.is_absolute():
        raise BindingError(
            "BINDING_REFERENCE",
            f"{source}: {field}: source references must be relative to the binding directory: "
            f"{declared}",
        )
    candidate = _resolved(binding_dir / path)
    if not _inside_roots(candidate, roots):
        raise BindingError(
            "BINDING_REFERENCE",
            f"{source}: {field}: path is outside the declared source roots: {candidate}",
        )
    if kind == "file":
        if candidate.is_dir():
            raise BindingError(
                "BINDING_REFERENCE",
                f"{source}: {field}: expected a file, found a directory: {candidate}",
            )
        if not candidate.is_file():
            raise BindingError(
                "BINDING_REFERENCE",
                f"{source}: {field}: file does not exist: {candidate}",
            )
    else:
        if candidate.is_file():
            raise BindingError(
                "BINDING_REFERENCE",
                f"{source}: {field}: expected a directory, found a file: {candidate}",
            )
        if not candidate.is_dir():
            raise BindingError(
                "BINDING_REFERENCE",
                f"{source}: {field}: directory does not exist: {candidate}",
            )
    return candidate


def _reference(
    field: str, declared: str, path: Path, kind: ReferenceKind
) -> ResolvedReference:
    digest = sha256_file(path) if kind == "file" else None
    return ResolvedReference(field, declared, path, kind, sha256=digest)


def _load_profile_documents(
    source: Path,
    binding: ScenarioBinding,
    binding_dir: Path,
    source_roots: tuple[Path, ...],
    schema_root: Path,
) -> dict[str, ResolvedReference]:
    roots = (binding_dir, *source_roots)
    documents: dict[str, dict[str, Any]] = {}
    resolved: dict[str, tuple[str, Path]] = {}
    for name in _PROFILE_FIELDS:
        field = f"profiles.{name}"
        declared = getattr(binding.profiles, name)
        path = _resolve_source_reference(
            source, field, declared, kind="file", roots=roots, binding_dir=binding_dir
        )
        try:
            documents[name] = load_document(path)
        except ContractError as error:
            raise BindingError("BINDING_SCHEMA", f"{field}: {error}") from error
        resolved[name] = (declared, path)
    try:
        profile = profile_from_document(
            documents["robot"], schema_root, source=resolved["robot"][1]
        )
        validate_bindings(
            profile, documents["observation"], documents["action"], schema_root
        )
    except ContractError as error:
        raise BindingError("BINDING_SCHEMA", f"{source}: profiles: {error}") from error
    except OSError as error:
        raise BindingError(
            "BINDING_REFERENCE", f"{source}: profiles: schema is unreadable: {error}"
        ) from error
    references: dict[str, ResolvedReference] = {}
    for name in _PROFILE_FIELDS:
        field = f"profiles.{name}"
        declared, path = resolved[name]
        try:
            semantic_digest = semantic_hash(documents[name])
        except (TypeError, ValueError) as error:
            raise BindingError(
                "BINDING_SCHEMA", f"{source}: {field}: cannot hash document: {error}"
            ) from error
        references[field] = ResolvedReference(
            field, declared, path, "file",
            sha256=sha256_file(path), semantic_sha256=semantic_digest,
        )
    return references


def _parse_binding(source: Path) -> ScenarioBinding:
    try:
        document = load_document(source)
    except ContractError as error:
        raise BindingError("BINDING_SCHEMA", str(error)) from error
    try:
        return ScenarioBinding.validate_dict(document, source=str(source))
    except ContractError as error:
        detail = str(error).removeprefix(f"{error.reason}: ")
        raise BindingError("BINDING_SCHEMA", detail) from error


def _load(
    source: Path,
    *,
    schema_root: Path,
) -> ResolvedBinding:
    source = Path(source)
    if source.is_dir():
        raise BindingError(
            "BINDING_REFERENCE", f"{source}: expected a binding file, found a directory"
        )
    if not source.is_file():
        raise BindingError("BINDING_REFERENCE", f"{source}: binding file does not exist")
    resolved_source = _resolved(source)

    binding = _parse_binding(source)
    binding_dir = resolved_source.parent
    source_roots = _root_list(source, binding.source_roots, "source", binding_dir)
    artifact_roots = _root_list(source, binding.artifact_roots, "artifact", binding_dir)
    source_reference_roots = (binding_dir, *source_roots)

    references = _load_profile_documents(
        source, binding, binding_dir, source_roots, schema_root
    )

    for name in _RECIPE_FIELDS:
        declared = getattr(binding.recipes, name)
        if declared is None:
            continue
        field = f"recipes.{name}"
        path = _resolve_source_reference(
            source, field, declared, kind="file",
            roots=source_reference_roots, binding_dir=binding_dir,
        )
        references[field] = _reference(field, declared, path, "file")

    for index, declared in enumerate(binding.interfaces.extension_ros):
        field = f"interfaces.extension_ros[{index}]"
        path = _resolve_source_reference(
            source, field, declared, kind="directory",
            roots=source_reference_roots, binding_dir=binding_dir,
        )
        references[field] = _reference(field, declared, path, "directory")

    return ResolvedBinding(
        root=binding_dir,
        path=resolved_source,
        sha256=sha256_file(resolved_source),
        binding=binding,
        source_roots=source_roots,
        artifact_roots=artifact_roots,
        references=MappingProxyType(references),
    )


def resolve_binding_path(
    shared_root: Path | str, robot_id: str, backend_id: str, task_id: str
) -> Path:
    """Return ``<shared_root>/scenarios/<robot_id>/<backend_id>/<task_id>/binding.yaml``.

    All three ids are validated before any path is built, so a caller cannot
    escape the scenarios tree with a crafted robot, backend, or task name.
    """
    for field, value in (
        ("robot_id", robot_id),
        ("backend_id", backend_id),
        ("task_id", task_id),
    ):
        if not isinstance(value, str) or _SCENARIO_ID_PATTERN.fullmatch(value) is None:
            raise BindingError(
                "BINDING_REFERENCE",
                f"{field}: invalid scenario id {value!r}; expected 1..128 safe characters",
            )
    if robot_id in RESERVED_ROBOT_IDS:
        raise BindingError(
            "BINDING_REFERENCE",
            f"robot_id: {robot_id!r} is reserved and cannot be used as a scenario root",
        )
    return Path(shared_root) / "scenarios" / robot_id / backend_id / task_id / "binding.yaml"


def load_binding(
    path: Path | str,
    *,
    shared_root: Path | str,
) -> ResolvedBinding:
    """Load and fully validate one binding without executing any scenario code."""
    shared_root = Path(shared_root)
    schema_root = shared_root / "common" / "interfaces" / "schemas"
    if not schema_root.is_dir():
        raise BindingError(
            "BINDING_REFERENCE",
            f"{shared_root}: shared schema directory does not exist: {schema_root}",
        )
    return _load(Path(path), schema_root=schema_root)
