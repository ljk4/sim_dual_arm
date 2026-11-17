"""Read-only CLI behaviour for scenario.py."""

from __future__ import annotations

import importlib.util
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest
from scenario_fixtures import SHARED, ScenarioTree, build_scenario

ROOT = Path(__file__).resolve().parents[2]
TOOL = ROOT / "shared/common/tools/scenario.py"
BANNED_MODULES = {
    "rclpy",
    "mujoco",
    "torch",
    "numpy",
    "scipy",
    "lerobot",
    "dual_arm_runtime",
    "dual_arm_learning",
    "robot_backend_sim",
    "subprocess",
    "multiprocessing",
}
BANNED_PREFIXES = ("rclpy.", "mujoco.", "torch.", "lerobot.", "dual_arm_")


@pytest.fixture
def scenario(tmp_path: Path) -> ScenarioTree:
    return build_scenario(tmp_path / "scenario")


def _tool_module():
    spec = importlib.util.spec_from_file_location("scenario_tool_for_tests", TOOL)
    module = importlib.util.module_from_spec(spec)
    assert module is not None and spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _run(module, *arguments: str) -> int:
    return module.main(list(arguments))


def _tree_state(root: Path) -> dict[str, tuple[int, int]]:
    return {
        str(path.relative_to(root)): (path.stat().st_size, path.stat().st_mtime_ns)
        for path in sorted(root.rglob("*"))
    }


def test_validate_and_show_succeed(scenario, capsys):
    module = _tool_module()
    binding = str(scenario.binding_path)
    code = _run(
        module, "validate", "--binding", binding, "--stage", "collection",
        "--shared-root", str(SHARED),
    )
    assert code == 0
    result = json.loads(capsys.readouterr().out)
    assert result["ok"] is True
    assert result["stage"] == "collection"
    assert result["scenario_id"] == "synthetic_sim/stack"
    assert result["models"]["source"] == "none"
    assert result["models"]["digest"] is None

    code = _run(module, "show", "--binding", binding, "--shared-root", str(SHARED))
    assert code == 0
    output = capsys.readouterr().out
    assert output.lstrip().startswith("{")
    shown = json.loads(output)
    assert shown["binding"]["scenario_id"] == "synthetic_sim/stack"
    assert shown["stages"]["inference"]["satisfied"] is True
    assert "eval" not in output.splitlines()[0]


def test_show_is_identical_from_another_cwd(scenario, tmp_path, monkeypatch, capsys):
    module = _tool_module()
    binding = str(scenario.binding_path)
    assert _run(module, "show", "--binding", binding, "--shared-root", str(SHARED)) == 0
    first = capsys.readouterr().out
    elsewhere = tmp_path / "elsewhere"
    elsewhere.mkdir()
    monkeypatch.chdir(elsewhere)
    assert _run(module, "show", "--binding", binding, "--shared-root", str(SHARED)) == 0
    assert capsys.readouterr().out == first


def test_validate_reports_an_unsatisfied_stage(scenario, capsys):
    document = scenario.copy()
    document["interfaces"]["policy_api"] = None
    scenario.write(document)
    module = _tool_module()

    code = _run(
        module, "validate", "--binding", str(scenario.binding_path),
        "--stage", "inference", "--shared-root", str(SHARED),
    )
    assert code == 2
    captured = capsys.readouterr()
    assert captured.out == ""
    assert "BINDING_STAGE" in captured.err
    assert "interfaces.policy_api" in captured.err


def test_bundle_override_is_resolved_from_the_caller_cwd(scenario, monkeypatch, capsys):
    (scenario.root / "override_bundle").mkdir()
    monkeypatch.chdir(scenario.root)
    module = _tool_module()
    code = _run(
        module, "validate", "--binding", str(scenario.binding_path),
        "--stage", "inference", "--bundle", "override_bundle",
        "--shared-root", str(SHARED),
    )
    assert code == 0
    result = json.loads(capsys.readouterr().out)
    assert result["models"]["source"] == "override"
    assert result["models"]["effective"] == str(
        (scenario.root / "override_bundle").resolve()
    )


def test_tool_does_not_write_to_the_scenario_or_cwd(scenario, tmp_path, monkeypatch, capsys):
    before = _tree_state(scenario.root)
    empty = tmp_path / "empty"
    empty.mkdir()
    monkeypatch.chdir(empty)
    module = _tool_module()
    assert _run(
        module, "validate", "--binding", str(scenario.binding_path),
        "--stage", "learning", "--shared-root", str(SHARED),
    ) == 0
    assert _run(module, "show", "--binding", str(scenario.binding_path),
                "--shared-root", str(SHARED)) == 0
    capsys.readouterr()
    assert _tree_state(scenario.root) == before
    assert list(empty.iterdir()) == []


def test_tool_process_imports_no_runtime_and_starts_nothing(scenario, tmp_path):
    empty = tmp_path / "empty"
    empty.mkdir()
    script = f"""
import contextlib
import importlib.util
import io
import json
import sys

spec = importlib.util.spec_from_file_location("scenario_tool_subprocess", {json.dumps(str(TOOL))})
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
buffer = io.StringIO()
with contextlib.redirect_stdout(buffer), contextlib.redirect_stderr(buffer):
    code = module.main(["validate", "--binding", {json.dumps(str(scenario.binding_path))},
                        "--stage", "collection", "--shared-root", {json.dumps(str(SHARED))}])
banned = sorted(name for name in sys.modules
                if name in {sorted(BANNED_MODULES)!r}
                or any(name.startswith(prefix) for prefix in {BANNED_PREFIXES!r}))
print(json.dumps({{
    "code": code,
    "banned": banned,
    "output": buffer.getvalue(),
    "cwd_files": sorted(path.name for path in __import__("pathlib").Path.cwd().iterdir()),
}}))
"""
    environment = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
    environment.pop("SHARED_ROOT", None)
    completed = subprocess.run(
        [sys.executable, "-I", "-c", script],
        cwd=empty,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr
    result = json.loads(completed.stdout)
    assert result["code"] == 0
    assert result["banned"] == []
    assert json.loads(result["output"])["ok"] is True
    assert result["cwd_files"] == []


def test_shared_root_defaults_to_the_tool_layout(scenario, monkeypatch, capsys):
    monkeypatch.delenv("SHARED_ROOT", raising=False)
    module = _tool_module()
    assert _run(module, "show", "--binding", str(scenario.binding_path)) == 0
    assert json.loads(capsys.readouterr().out)["binding"]["scenario_id"] == "synthetic_sim/stack"


def test_missing_schema_root_is_a_clean_error(scenario, tmp_path, capsys):
    module = _tool_module()
    code = _run(
        module, "show", "--binding", str(scenario.binding_path),
        "--shared-root", str(tmp_path / "missing-shared"),
    )
    assert code == 2
    captured = capsys.readouterr()
    assert captured.out == ""
    assert "schema directory does not exist" in captured.err


def test_unknown_stage_is_rejected_by_argparse(scenario):
    module = _tool_module()
    with pytest.raises(SystemExit) as caught:
        _run(module, "validate", "--binding", str(scenario.binding_path), "--stage", "training")
    assert caught.value.code == 2
