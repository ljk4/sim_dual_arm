"""Shared worker preparation and container path mapping for parallel collection/inference."""

from __future__ import annotations

from pathlib import Path

from dual_arm_runtime.offline.collection.campaign_ledger import CampaignError

SIMULATION_REFERENCE_FIELDS = (
    "profile",
    "schemas",
    "scene",
    "control",
    "stream",
    "follower",
    "task",
)


def simulation_path(path: Path, project: Path) -> str:
    """Map a host/runtime path to the simulation container mount."""
    for container in ("/shared", "/data", "/app"):
        root = Path(container)
        if path == root or path.is_relative_to(root):
            return str(path)
    for source, destination in (
        (project / "simulation/mujoco", Path("/app")),
        (project / "shared", Path("/shared")),
        (project / "var", Path("/data")),
    ):
        if path == source or path.is_relative_to(source):
            return str(destination / path.relative_to(source))
    raise CampaignError(f"path is not mounted in simulation: {path}")


def backend_config_for_simulation(backend_template: Path, project: Path) -> str:
    """Return the simulation container path for the read-only backend template."""
    try:
        return simulation_path(backend_template, project)
    except CampaignError:
        if backend_template.is_relative_to(project):
            return str(backend_template)
        raise


def validate_backend_references(
    backend_document: dict, backend_template: Path, project: Path
) -> dict[str, str]:
    """Validate that all robot.* references inside backend_template map into simulation."""
    robot = backend_document.get("robot")
    if not isinstance(robot, dict):
        raise CampaignError(f"backend template has no robot mapping: {backend_template}")
    resolved: dict[str, str] = {}
    for name in SIMULATION_REFERENCE_FIELDS:
        value = robot.get(name)
        if value:
            source = (backend_template.parent / str(value)).resolve()
            resolved[name] = simulation_path(source, project)
    return resolved


__all__ = [
    "SIMULATION_REFERENCE_FIELDS",
    "backend_config_for_simulation",
    "simulation_path",
    "validate_backend_references",
]
