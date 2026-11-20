from __future__ import annotations

import json
import re
import sys
from pathlib import Path
from types import ModuleType

import pytest

ROOT = Path(__file__).resolve().parents[2]
SIM_SRC = ROOT / "simulation/mujoco/src"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))

from robot_backend_sim.tools import probe_environment as probe  # noqa: E402


def test_requirements_lock_is_the_only_pinned_simulation_source():
    docker = ROOT / "simulation/mujoco/docker"
    lock_path = docker / "requirements.lock"
    lock = lock_path.read_text()
    pins = re.findall(
        r"^([A-Za-z0-9_.-]+)==([^\s]+)\s+--hash=sha256:([0-9a-f]{64})$",
        lock,
        re.MULTILINE,
    )
    assert len(pins) == len({name for name, _version, _hash in pins})
    assert len(pins) == sum(bool(line.strip()) and not line.startswith(("#", "--"))
                            for line in lock.splitlines())
    locked = {name: version for name, version, _digest in pins}
    assert {
        "mujoco",
        "mink",
        "pillow",
        "numpy",
        "scipy",
        "qpsolvers",
        "daqp",
        "PyOpenGL",
    } <= set(locked)
    assert "ruckig" not in locked
    assert "ruckig" not in probe._PACKAGE_DISTRIBUTIONS
    assert not (docker / "sim-requirements.lock").exists()
    assert not (docker / "sim-packages.json").exists()
    assert not (docker / "sim-tools-requirements.txt").exists()
    assert not (docker / "ruckig-requirements.lock").exists()


def test_probe_check_dependencies_uses_current_packages_and_modules_without_reports(
    tmp_path, monkeypatch
):
    monkeypatch.setattr(probe, "_version", lambda _name: "1.0")
    imported = []

    def import_module(name: str) -> ModuleType:
        imported.append(name)
        module = ModuleType(name)
        module.__file__ = str(tmp_path / f"{name.replace('.', '_')}.py")
        return module

    monkeypatch.setattr(probe.importlib, "import_module", import_module)
    info = probe.check_dependencies()
    assert set(info["packages"]) == set(probe._PACKAGE_DISTRIBUTIONS)
    assert set(info["packages"].values()) == {"1.0"}
    assert imported == list(probe._IMPORT_MODULES)
    assert set(info["imports"]) == set(probe._IMPORT_MODULES)
    assert info["sim_package_root"] == str(Path(probe.__file__).resolve().parents[1])
    assert "requirements" not in info
    assert "requirements_sha256" not in info
    assert not (tmp_path / "versions.json").exists()


def test_probe_check_dependencies_reports_missing_imports(monkeypatch):
    monkeypatch.setattr(probe, "_version", lambda _name: "1.0")

    def import_module(name: str) -> ModuleType:
        if name == "mink":
            raise ModuleNotFoundError("mink is not installed")
        module = ModuleType(name)
        module.__file__ = __file__
        return module

    monkeypatch.setattr(probe.importlib, "import_module", import_module)
    with pytest.raises(probe.ProbeFailure) as caught:
        probe.check_dependencies()
    assert caught.value.reason == "dependency_import_failed"
    assert "mink" in caught.value.message


def test_probe_rejects_unsafe_run_identity_and_report_root():
    with pytest.raises(probe.ProbeUsageError):
        probe._validate_run_id("..")
    with pytest.raises(probe.ProbeUsageError):
        probe._validate_run_id("bad/path")
    with pytest.raises(probe.ProbeUsageError):
        probe._validate_report_root(Path("/"))
    with pytest.raises(probe.ProbeUsageError):
        probe._validate_report_root(Path.home())
    assert probe._validate_run_id("sim-001_OK.1") == "sim-001_OK.1"


def test_probe_log_directory_uses_its_actual_instance_identity(tmp_path, monkeypatch):
    arguments = probe.ProbeArguments(tmp_path, "identity")
    monkeypatch.setattr(
        probe,
        "check_dependencies",
        lambda: {"python": {"version": "3.12"}, "packages": {}, "imports": {}},
    )
    monkeypatch.setattr(probe, "_verify_mounted_source", lambda *_args: None)
    monkeypatch.setattr(
        probe, "_check_model", lambda: ({"stats": {"nq": 9, "nv": 9, "nu": 8}}, object())
    )
    monkeypatch.setattr(probe, "_check_qp", lambda: {"max_abs_error": 0.0})
    monkeypatch.setattr(
        probe,
        "_render_preview",
        lambda *_args: {
            "artifact": "preview.png",
            "vendor": "fixture",
            "renderer": "fixture",
            "unique_colors": 32,
        },
    )

    assert probe.run(arguments) == 0
    report = json.loads((tmp_path / "identity/report.json").read_text())
    assert "image_versions" not in report
    assert report["source"] == {
        "model_directory": str(probe.assets.robot_model_path("marvin").parent)
    }
    assert Path(report["log_directory"]).name == report["instance_id"]
    events = list((tmp_path / "identity" / report["log_directory"]).glob("events*.jsonl"))
    assert events
    assert all(
        json.loads(line)["instance_id"] == report["instance_id"]
        for path in events
        for line in path.read_text().splitlines()
    )


@pytest.mark.parametrize("shadowed", [False, True])
def test_mounted_source_check_uses_declared_module_root(tmp_path, monkeypatch, shadowed):
    source_root = tmp_path / "app/src"
    monkeypatch.setenv("PROJECT_SOURCE_ROOT", str(source_root))
    actual = tmp_path / "installed/robot_backend_sim" if shadowed else (
        source_root / "robot_backend_sim"
    )
    info = {"sim_package_root": str(actual)}
    if shadowed:
        with pytest.raises(probe.ProbeFailure, match="expected the mounted source"):
            probe._verify_mounted_source(info)
    else:
        probe._verify_mounted_source(info)
