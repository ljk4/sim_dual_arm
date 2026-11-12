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

import hashlib
from collections.abc import Mapping
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from types import MappingProxyType
from typing import Annotated, Any, Literal

from pydantic import BeforeValidator, Field, model_validator

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
    contract_error,
)

__all__ = [
    "BindingError",
    "BindingStage",
    "ResolvedBinding",
    "ResolvedBundle",
    "ResolvedReference",
    "ScenarioBinding",
    "load_binding",
    "stage_requirements",
]

PathList = Annotated[tuple[NonEmptyStr, ...], BeforeValidator(as_tuple)]
SchemaVersion = Annotated[int, Field(strict=True, ge=1, le=1)]
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
BackendKind = Literal["simulation", "hardware"]
ReferenceKind = Literal["file", "directory"]
BundleSource = Literal["binding", "override", "none"]

_RECIPE_FIELDS = (
    "simulation_teacher",
    "simulation_policy",
    "runtime_teacher",
    "runtime_policy",
    "campaign",
    "dataset",
    "training",
    "policy_service",
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
        "models.bundle",
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


class InterfacesSpec(StrictContractModel):
    common_ros: PathList = ()
    extension_ros: PathList = ()
    teacher_api: NonEmptyStr | None = None
    policy_api: NonEmptyStr | None = None


class ModelsSpec(StrictContractModel):
    bundle: NonEmptyStr | None = None


class ScenarioBinding(StrictContractModel):
    """The strict v1 binding document.  Unknown fields are rejected."""

    schema_version: SchemaVersion
    scenario_id: ScenarioId
    description: NonEmptyStr | None = None
    backend: BackendSpec
    runtime: RuntimeSpec
    learning: LearningSpec
    profiles: ProfilesSpec
    recipes: RecipesSpec = Field(default_factory=RecipesSpec)
    interfaces: InterfacesSpec = Field(default_factory=InterfacesSpec)
    models: ModelsSpec = Field(default_factory=ModelsSpec)
    source_roots: PathList = ()
    artifact_roots: PathList = ()
    references: PathList = ()

    @model_validator(mode="before")
    @classmethod
    def _check_schema_version(cls, data: Any) -> Any:
        if isinstance(data, dict) and "schema_version" in data:
            version = data["schema_version"]
            if type(version) is not int or version != 1:
                raise contract_error(
                    "schema_version",
                    f"unsupported binding schema_version {version!r}; expected 1",
                )
        return data


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

    def summary(self) -> dict[str, Any]:
        record: dict[str, Any] = {
            "declared": self.declared,
            "path": str(self.path),
            "kind": self.kind,
        }
        if self.sha256 is not None:
            record["sha256"] = self.sha256
        if self.semantic_sha256 is not None:
            record["semantic_sha256"] = self.semantic_sha256
        return record


@dataclass(frozen=True)
class ResolvedBundle:
    """The effective model bundle reference.

    Only existence and directory type are checked here.  Bundle contents are
    deliberately left to the learning-side verifier; ``digest`` is always
    ``None`` so the public layer never invents a placeholder identity.
    """

    declared: str | None
    effective: str | None
    path: Path | None
    source: BundleSource

    def summary(self) -> dict[str, Any]:
        return {
            "declared": self.declared,
            "effective": self.effective,
            "source": self.source,
            "path": None if self.path is None else str(self.path),
            "kind": None if self.path is None else "directory",
            "exists": self.path is not None and self.path.is_dir(),
            "digest": None,
            "validated_by": "learning.verify_bundle",
        }


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
    dependencies: tuple[ResolvedBinding, ...]
    bundle: ResolvedBundle

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
        if field == "models.bundle":
            return self.bundle.path is not None
        section, name = field.split(".", 1)
        return getattr(getattr(self.binding, section), name) is not None

    def effective_bundle(self) -> dict[str, Any]:
        """Return the real effective source, never a placeholder digest."""
        return self.bundle.summary()

    def summary(self) -> dict[str, Any]:
        """JSON-safe, deterministic view used by the read-only CLI."""
        stages: dict[str, Any] = {}
        for stage in BindingStage:
            missing = self.missing(stage)
            stages[stage.value] = {
                "requirements": list(self.requirements(stage)),
                "missing": list(missing),
                "satisfied": not missing,
            }
        return {
            "binding": {
                "path": str(self.path),
                "sha256": self.sha256,
                "schema_version": self.binding.schema_version,
                "scenario_id": self.binding.scenario_id,
                "description": self.binding.description,
            },
            "declarations": self.binding.model_dump(mode="json"),
            "roots": {
                "binding": str(self.root),
                "source": [str(path) for path in self.source_roots],
                "artifact": [str(path) for path in self.artifact_roots],
            },
            "references": {
                field: reference.summary() for field, reference in self.references.items()
            },
            "models": self.bundle.summary(),
            "dependencies": [
                {
                    "path": str(dependency.path),
                    "sha256": dependency.sha256,
                    "scenario_id": dependency.binding.scenario_id,
                }
                for dependency in self.dependencies
            ],
            "stages": stages,
        }


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


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


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
        candidate = _resolved(path if path.is_absolute() else binding_dir / path)
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


def _resolve_model_reference(
    source: Path,
    field: str,
    declared: str,
    *,
    roots: tuple[Path, ...],
    binding_dir: Path,
) -> Path:
    path = Path(declared)
    candidate = _resolved(path if path.is_absolute() else binding_dir / path)
    if not _inside_roots(candidate, roots):
        raise BindingError(
            "BINDING_REFERENCE",
            f"{source}: {field}: path is outside the declared roots: {candidate}",
        )
    if candidate.is_file():
        raise BindingError(
            "BINDING_REFERENCE",
            f"{source}: {field}: expected a directory, found a file: {candidate}",
        )
    if not candidate.is_dir():
        raise BindingError(
            "BINDING_REFERENCE", f"{source}: {field}: directory does not exist: {candidate}"
        )
    return candidate


def _reference(
    field: str, declared: str, path: Path, kind: ReferenceKind
) -> ResolvedReference:
    digest = _sha256_file(path) if kind == "file" else None
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
            sha256=_sha256_file(path), semantic_sha256=semantic_digest,
        )
    return references


def _load_binding_document(source: Path) -> dict[str, Any]:
    try:
        return load_document(source)
    except ContractError as error:
        raise BindingError("BINDING_SCHEMA", str(error)) from error


def _parse_binding(source: Path) -> ScenarioBinding:
    document = _load_binding_document(source)
    try:
        return ScenarioBinding.validate_dict(document, source=str(source))
    except ContractError as error:
        detail = str(error).removeprefix(f"{error.reason}: ")
        raise BindingError("BINDING_SCHEMA", detail) from error


def _load(
    source: Path,
    *,
    schema_root: Path,
    bundle: Path | None,
    stack: tuple[Path, ...],
) -> ResolvedBinding:
    source = Path(source)
    if source.is_dir():
        raise BindingError(
            "BINDING_REFERENCE", f"{source}: expected a binding file, found a directory"
        )
    if not source.is_file():
        raise BindingError("BINDING_REFERENCE", f"{source}: binding file does not exist")
    resolved_source = _resolved(source)
    if resolved_source in stack:
        chain = " -> ".join(str(path) for path in (*stack, resolved_source))
        raise BindingError("BINDING_CYCLE", chain)
    stack = (*stack, resolved_source)

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

    dependencies: list[ResolvedBinding] = []
    for index, declared in enumerate(binding.references):
        field = f"references[{index}]"
        path = _resolve_source_reference(
            source, field, declared, kind="file",
            roots=source_reference_roots, binding_dir=binding_dir,
        )
        references[field] = _reference(field, declared, path, "file")
        dependencies.append(
            _load(path, schema_root=schema_root, bundle=None, stack=stack)
        )

    declared_bundle = binding.models.bundle
    model_roots = (binding_dir, *source_roots, *artifact_roots)
    if bundle is not None:
        effective_path = _resolve_model_reference(
            source, "--bundle", str(bundle), roots=model_roots, binding_dir=binding_dir
        )
        bundle_record = ResolvedBundle(
            declared=declared_bundle,
            effective=str(effective_path),
            path=effective_path,
            source="override",
        )
    elif declared_bundle is not None:
        effective_path = _resolve_model_reference(
            source, "models.bundle", declared_bundle,
            roots=model_roots, binding_dir=binding_dir,
        )
        bundle_record = ResolvedBundle(
            declared=declared_bundle,
            effective=str(effective_path),
            path=effective_path,
            source="binding",
        )
    else:
        bundle_record = ResolvedBundle(None, None, None, "none")

    return ResolvedBinding(
        root=binding_dir,
        path=resolved_source,
        sha256=_sha256_file(resolved_source),
        binding=binding,
        source_roots=source_roots,
        artifact_roots=artifact_roots,
        references=MappingProxyType(references),
        dependencies=tuple(dependencies),
        bundle=bundle_record,
    )


def load_binding(
    path: Path | str,
    *,
    shared_root: Path | str,
    bundle: Path | str | None = None,
) -> ResolvedBinding:
    """Load and fully validate one binding without executing any scenario code.

    ``shared_root`` is the explicit shared tree (for example ``/shared`` or the
    repository's ``shared/``); profile schemas are read from
    ``common/interfaces/schemas`` below it.  All binding-internal paths resolve
    against the binding directory and declared roots, never the current working
    directory.  ``bundle`` is an explicit caller path resolved against the
    current working directory and overrides ``models.bundle``.
    """
    shared_root = Path(shared_root)
    schema_root = shared_root / "common" / "interfaces" / "schemas"
    if not schema_root.is_dir():
        raise BindingError(
            "BINDING_REFERENCE",
            f"{shared_root}: shared schema directory does not exist: {schema_root}",
        )
    override = None if bundle is None else _resolved(Path(bundle))
    return _load(Path(path), schema_root=schema_root, bundle=override, stack=())
