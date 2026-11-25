"""Container mounts must resolve real recipes without redirecting test fixtures."""

from pathlib import Path

import pytest
from robot_contracts.project_paths import project_path, scenario_root, scenario_segment


def test_flattened_learning_mounts_resolve_recipe_inputs(monkeypatch):
    monkeypatch.setenv("PROJECT_CONTAINER", "1")
    monkeypatch.setenv("PROJECT_ROOT", "/app")
    monkeypatch.setenv("PROJECT_SOURCE_PREFIX", "learning/lerobot")
    monkeypatch.setenv("SHARED_ROOT", "/shared")
    monkeypatch.setenv("BUILD_ROOT", "/build")
    monkeypatch.setenv("DATA_ROOT", "/data")
    monkeypatch.setenv("THIRD_PARTY_ROOT", "/third_party")
    assert project_path("output/marvin/mujoco/stack/models/run", Path("/app")) == Path(
        "/data/marvin/mujoco/stack/models/run"
    )
    assert project_path("output/build/ros-dev/contracts.json", Path("/app")) == Path(
        "/build/ros-dev/contracts.json"
    )
    assert project_path("shared/scenarios/marvin/mujoco/stack/binding.yaml", Path("/app")) == Path(
        "/shared/scenarios/marvin/mujoco/stack/binding.yaml"
    )
    assert project_path("learning/lerobot/docker/learning-source.json", Path("/app")) == Path(
        "/app/docker/learning-source.json"
    )
    assert project_path("third_party/weights/backbone.pth", Path("/app")) == Path(
        "/third_party/weights/backbone.pth"
    )
    monkeypatch.setenv("PROJECT_SOURCE_PREFIX", "simulation/mujoco")
    assert project_path("simulation/mujoco/configs/container.env", Path("/app")) == Path(
        "/app/configs/container.env"
    )


def test_container_environment_does_not_redirect_explicit_fixture_roots(monkeypatch, tmp_path):
    monkeypatch.setenv("PROJECT_CONTAINER", "1")
    monkeypatch.setenv("PROJECT_ROOT", "/app")
    assert (
        project_path("output/marvin/mujoco/stack/models/run", tmp_path)
        == tmp_path / "output/marvin/mujoco/stack/models/run"
    )
    assert (
        project_path("output/build/ros-dev/contracts.json", tmp_path)
        == tmp_path / "output/build/ros-dev/contracts.json"
    )


def test_absolute_inputs_and_host_project_roots_remain_explicit(monkeypatch, tmp_path):
    monkeypatch.delenv("PROJECT_CONTAINER", raising=False)
    assert (
        project_path("output/marvin/mujoco/stack/models/run", tmp_path)
        == tmp_path / "output/marvin/mujoco/stack/models/run"
    )
    assert project_path("/data/marvin/mujoco/stack/models/run", tmp_path) == Path(
        "/data/marvin/mujoco/stack/models/run"
    )


def test_scenario_segment_supports_three_tiers_and_reserves_build():
    assert scenario_segment("marvin", "mujoco", "stack") == "marvin/mujoco/stack"
    assert scenario_segment(None, None, None) == "_direct"
    assert scenario_root(Path("/data"), "marvin", "mujoco", "stack") == Path(
        "/data/marvin/mujoco/stack"
    )
    with pytest.raises(ValueError, match="reserved"):
        scenario_segment("build", "mujoco", "stack")
    with pytest.raises(ValueError):
        scenario_segment("marvin", "mujoco", None)
