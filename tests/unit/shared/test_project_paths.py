"""Container mounts must resolve real recipes without redirecting test fixtures."""

from pathlib import Path

from robot_contracts.project_paths import project_path


def test_flattened_learning_mounts_resolve_recipe_inputs(monkeypatch):
    monkeypatch.setenv("PROJECT_CONTAINER", "1")
    monkeypatch.setenv("PROJECT_ROOT", "/app")
    monkeypatch.setenv("PROJECT_SOURCE_PREFIX", "learning/lerobot")
    monkeypatch.setenv("SHARED_ROOT", "/shared")
    monkeypatch.setenv("BUILD_ROOT", "/build")
    monkeypatch.setenv("DATA_ROOT", "/data")
    monkeypatch.setenv("THIRD_PARTY_ROOT", "/third_party")
    assert project_path("var/task/models/run", Path("/app")) == Path("/data/task/models/run")
    assert project_path("shared/scenarios/task/binding.yaml", Path("/app")) == Path(
        "/shared/scenarios/task/binding.yaml"
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
    assert project_path("var/task/models/run", tmp_path) == tmp_path / "var/task/models/run"


def test_absolute_inputs_and_host_project_roots_remain_explicit(monkeypatch, tmp_path):
    monkeypatch.delenv("PROJECT_CONTAINER", raising=False)
    assert project_path("var/task/models/run", tmp_path) == tmp_path / "var/task/models/run"
    assert project_path("/data/task/models/run", tmp_path) == Path("/data/task/models/run")
