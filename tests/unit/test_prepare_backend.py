"""Backend preparation must retain the paths visible in its own container."""

import importlib.util
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location(
    "prepare_backend", ROOT / "simulation/mujoco/scripts/prepare_backend.py"
)
assert spec is not None and spec.loader is not None
prepare_backend = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prepare_backend)


def test_prepare_keeps_project_and_external_dependency_paths(tmp_path):
    project = tmp_path / "app"
    config = project / "configs"
    config.mkdir(parents=True)
    shared_profile = tmp_path / "shared/scenarios/panda_stack/profiles/profile.json"
    shared_profile.parent.mkdir(parents=True)
    shared_profile.write_text("{}")
    scene = config / "scene.yaml"
    scene.write_text("{}")
    template = config / "backend.yaml"
    template.write_text(yaml.safe_dump({
        "robot": {
            "profile": "../../shared/scenarios/panda_stack/profiles/profile.json",
            "scene": "scene.yaml",
        },
        "run": {"seed": 0},
    }))
    output = tmp_path / "data/prepared/run-1"
    manifest = prepare_backend.prepare_backend(
        template=template, seed=42, run_id="run-1", output=output
    )
    result = yaml.safe_load((output / "backend.yaml").read_text())
    assert result["robot"]["scene"] == str(scene)
    assert result["robot"]["profile"] == str(shared_profile)
    assert result["run"]["seed"] == 42
    assert manifest["kind"] == "simulation_prepare"
    before = (output / "backend.yaml").read_bytes()
    with pytest.raises(prepare_backend.PrepareError, match="refusing to overwrite"):
        prepare_backend.prepare_backend(
            template=template, seed=43, run_id="run-1", output=output
        )
    assert (output / "backend.yaml").read_bytes() == before
