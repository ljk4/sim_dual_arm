from __future__ import annotations

import json
import os
import shutil
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "shared/common/tools/scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))


def load_tool(name: str, relative: str):
    import importlib.util

    path = ROOT / relative
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


dev_workspace = load_tool("dsh011_dev_workspace", "shared/common/tools/scripts/dev_workspace.py")


def write_package(
    directory: Path,
    name: str,
    *,
    body: str = "int32 value\n",
    service: bool = False,
    dependencies: tuple[str, ...] = (),
    readme: str = "readme\n",
) -> Path:
    (directory / "msg").mkdir(parents=True, exist_ok=True)
    (directory / "msg/Example.msg").write_text(body)
    if service:
        (directory / "srv").mkdir(parents=True, exist_ok=True)
        (directory / "srv/Example.srv").write_text(f"---\n{body}")
    (directory / "package.xml").write_text(
        "<package format='3'><name>"
        + name
        + "</name>"
        + "".join(f"<depend>{item}</depend>" for item in dependencies)
        + "</package>"
    )
    (directory / "CMakeLists.txt").write_text("rosidl_generate_interfaces(pkg)\n")
    (directory / "README.md").write_text(readme)
    return directory


def project_tree(tmp_path: Path) -> tuple[Path, Path]:
    project = tmp_path / "repo"
    shared = project / "shared"
    write_package(shared / "common/interfaces/ros/robot_api_msgs", "robot_api_msgs")
    return project, shared


def add_common(shared: Path, name: str, **kwargs) -> Path:
    return write_package(shared / "common/interfaces/ros" / name, name, **kwargs)


def add_extension(shared: Path, name: str, **kwargs) -> Path:
    return write_package(shared / "scenarios/demo/sim/extensions/ros" / name, name, **kwargs)


def create_generated_install(build_root: Path, packages) -> None:
    install = build_root / "install"
    for package in packages:
        module = install / f"{package.name}/lib/python3.12/site-packages/{package.name}"
        directories = [module]
        for submodule in ("msg", "srv", "action"):
            if (package.directory / submodule).is_dir():
                directories.append(module / submodule)
        for directory in directories:
            directory.mkdir(parents=True, exist_ok=True)
            (directory / "__init__.py").write_text("# generated, test_only\n")
    install.mkdir(parents=True, exist_ok=True)
    (install / "setup.bash").write_text("# generated setup\n")


def successful_runner(project: Path, build_root: Path, packages) -> int:
    create_generated_install(build_root, packages)
    return 0


def failed_runner(project: Path, build_root: Path, packages) -> int:
    return 7


def build(project: Path, shared: Path, build_root: Path):
    return dev_workspace.build_workspace(
        project, build_root, shared_root=shared, runner=successful_runner
    )


def verify(project: Path, shared: Path, build_root: Path):
    return dev_workspace.verify_workspace(project, build_root, shared_root=shared)


def test_package_digest_is_stable_and_ignores_readme_changes(tmp_path):
    project, shared = project_tree(tmp_path)
    packages = dev_workspace.discover_packages(shared)
    first = dev_workspace._package_records(packages)
    second = dev_workspace._package_records(packages)
    assert first == second

    readme = shared / "common/interfaces/ros/robot_api_msgs/README.md"
    readme.write_text("changed documentation only\n")
    assert dev_workspace._package_records(packages) == first

    idl = shared / "common/interfaces/ros/robot_api_msgs/msg/Example.msg"
    idl.write_text("int32 value\n# changed\n")
    assert dev_workspace._package_records(packages) != first


def test_build_metadata_and_relative_paths_change_the_digest(tmp_path):
    project, shared = project_tree(tmp_path)
    packages = dev_workspace.discover_packages(shared)
    first = dev_workspace._package_records(packages)

    cmake = shared / "common/interfaces/ros/robot_api_msgs/CMakeLists.txt"
    cmake.write_text("rosidl_generate_interfaces(pkg msg/Example.msg)\n")
    changed = dev_workspace._package_records(packages)
    assert changed != first

    msg = shared / "common/interfaces/ros/robot_api_msgs/msg/Example.msg"
    msg.rename(msg.with_name("Renamed.msg"))
    assert dev_workspace._package_records(packages) != changed


def test_successful_core_build_writes_marker_and_verify_accepts_it(tmp_path):
    project, shared = project_tree(tmp_path)
    document = build(project, shared, project / "output")

    marker = project / "output/contracts.json"
    assert marker.is_file()
    assert json.loads(marker.read_text()) == document
    assert list(document["packages"]) == ["robot_api_msgs"]
    assert verify(project, shared, project / "output") == document


def test_full_build_covers_common_and_extension_packages(tmp_path):
    project, shared = project_tree(tmp_path)
    add_common(shared, "sim_api_msgs")
    add_extension(shared, "demo_api_msgs", service=True)
    document = build(project, shared, project / "output")

    assert set(document["packages"]) == {"robot_api_msgs", "sim_api_msgs", "demo_api_msgs"}
    assert document["packages"]["sim_api_msgs"]["role"] == "common"
    assert document["packages"]["demo_api_msgs"]["role"] == "extension"
    assert verify(project, shared, project / "output") == document


def test_check_rejects_a_new_package_added_after_build(tmp_path):
    project, shared = project_tree(tmp_path)
    build(project, shared, project / "output")
    add_extension(shared, "late_api_msgs", service=True)

    with pytest.raises(dev_workspace.ContractCheckError) as caught:
        verify(project, shared, project / "output")
    assert "stale" in str(caught.value)


def test_readme_change_does_not_invalidate_but_idl_change_does(tmp_path):
    project, shared = project_tree(tmp_path)
    build(project, shared, project / "output")

    (shared / "common/interfaces/ros/robot_api_msgs/README.md").write_text("docs\n")
    verify(project, shared, project / "output")

    (shared / "common/interfaces/ros/robot_api_msgs/msg/Example.msg").write_text(
        "int32 changed\n"
    )
    with pytest.raises(dev_workspace.ContractCheckError) as caught:
        verify(project, shared, project / "output")
    assert "stale" in str(caught.value)
    assert dev_workspace.BUILD_COMMAND in str(caught.value)


def test_verify_rejects_missing_or_damaged_generated_modules(tmp_path):
    project, shared = project_tree(tmp_path)
    build_root = project / "output"
    build(project, shared, build_root)
    module = build_root / "install/robot_api_msgs/lib/python3.12/site-packages/robot_api_msgs"
    (module / "msg/__init__.py").unlink()

    with pytest.raises(dev_workspace.ContractCheckError) as caught:
        verify(project, shared, build_root)
    assert "robot_api_msgs.msg" in str(caught.value)

    (module / "msg/__init__.py").write_text("# generated\n")
    (build_root / "contracts.json").write_text("{not json")
    with pytest.raises(dev_workspace.ContractCheckError) as caught:
        verify(project, shared, build_root)
    assert "unreadable" in str(caught.value)


def test_duplicate_packages_and_duplicate_modules_are_rejected(tmp_path):
    project, shared = project_tree(tmp_path)
    build_root = project / "output"
    build(project, shared, build_root)
    duplicate = (
        build_root
        / "install/robot_api_msgs/lib/python3.11/site-packages/robot_api_msgs"
    )
    duplicate.mkdir(parents=True)
    (duplicate / "__init__.py").write_text("# stale generated copy\n")
    (duplicate / "msg").mkdir()
    (duplicate / "msg/__init__.py").write_text("# stale generated copy\n")

    with pytest.raises(dev_workspace.ContractCheckError) as caught:
        verify(project, shared, build_root)
    assert "expected one generated robot_api_msgs module" in str(caught.value)

    first = add_extension(shared, "demo_api_msgs", service=True)
    second = write_package(
        shared / "scenarios/other/sim/extensions/ros/demo_api_msgs", "demo_api_msgs"
    )
    assert first.is_dir() and second.is_dir()
    with pytest.raises(dev_workspace.DevWorkspaceError) as caught:
        dev_workspace.discover_packages(shared)
    assert "duplicate package" in str(caught.value)


def test_failed_build_removes_previous_marker(tmp_path):
    project, shared = project_tree(tmp_path)
    build_root = project / "output"
    dev_workspace.build_workspace(
        project, build_root, shared_root=shared, runner=successful_runner
    )
    assert (build_root / "contracts.json").is_file()

    with pytest.raises(dev_workspace.DevWorkspaceError) as caught:
        dev_workspace.build_workspace(
            project, build_root, shared_root=shared, runner=failed_runner
        )
    assert "exit code 7" in str(caught.value)
    assert not (build_root / "contracts.json").exists()

    build(project, shared, build_root)
    assert (build_root / "contracts.json").is_file()


def test_rebuild_keeps_unknown_top_level_files_and_replaces_outputs(tmp_path):
    project, shared = project_tree(tmp_path)
    build_root = project / "output"
    build_root.mkdir()

    build(project, shared, build_root)
    kept = build_root / "operator-notes.txt"
    kept.write_text("top-level file\n")

    build(project, shared, build_root)
    assert (build_root / "contracts.json").is_file()
    assert kept.read_text() == "top-level file\n"
    assert not (build_root / "build").exists()
    assert not (build_root / "log").exists()
    assert not (build_root / "colcon-home").exists()
    assert not (build_root / "workspace.json").exists()


def test_build_rejects_concurrent_builder_before_cleaning(tmp_path):
    project, shared = project_tree(tmp_path)
    build_root = project / "output"
    build_root.mkdir()
    (build_root / "install").mkdir()
    calls = []

    def runner(project_root, root, packages):
        calls.append((project_root, root, packages))
        return 1

    with dev_workspace.build_lock(build_root):
        with pytest.raises(dev_workspace.BuildBusyError) as caught:
            dev_workspace.build_workspace(
                project, build_root, shared_root=shared, runner=runner
            )
    assert "another builder" in str(caught.value)
    assert calls == []
    assert (build_root / "install").is_dir()


@pytest.mark.parametrize("value", ["/", "."])
def test_build_root_rejects_dangerous_locations(tmp_path, value):
    project, shared = project_tree(tmp_path)
    candidate = Path(value) if value == "/" else project
    with pytest.raises(dev_workspace.DevWorkspaceError):
        dev_workspace.validate_build_root(project, candidate, shared)


def test_build_rejects_a_file_at_the_output_root(tmp_path):
    project, shared = project_tree(tmp_path)
    build_root = project / "output"
    build_root.write_text("not a directory\n")

    with pytest.raises(dev_workspace.DevWorkspaceError) as caught:
        build(project, shared, build_root)
    assert "not a directory" in str(caught.value)
    assert build_root.read_text() == "not a directory\n"


def test_build_removes_symlinked_outputs_without_following(tmp_path):
    project, shared = project_tree(tmp_path)
    build_root = project / "output"
    outside = tmp_path / "outside"
    outside.mkdir()
    (outside / "keep.txt").write_text("keep\n")

    build(project, shared, build_root)
    shutil.rmtree(build_root / "install")
    (build_root / "install").symlink_to(outside, target_is_directory=True)

    build(project, shared, build_root)
    assert (outside / "keep.txt").read_text() == "keep\n"
    assert (build_root / "install").is_dir()
    assert not (build_root / "install").is_symlink()


def test_run_colcon_uses_temporary_intermediates_and_cleans_them(tmp_path, monkeypatch):
    captured = {}

    def fake_run(command, *, cwd, env, check):
        captured["command"] = command
        captured["env"] = env
        return type("Completed", (), {"returncode": 0})()

    monkeypatch.setattr(dev_workspace.subprocess, "run", fake_run)
    project, shared = project_tree(tmp_path)
    build_root = project / "output"
    packages = dev_workspace.discover_packages(shared)
    assert dev_workspace.run_colcon(project, build_root, packages) == 0

    environment = captured["env"]
    assert environment.get("HOME") == os.environ.get("HOME")
    assert environment["ROS_LOG_DIR"] == dev_workspace.ROS_LOG_DIRECTORY
    command = captured["command"]
    install_base = command[command.index("--install-base") + 1]
    build_base = command[command.index("--build-base") + 1]
    log_base = command[command.index("--log-base") + 1]
    assert install_base == str(build_root.resolve() / "install")
    assert str(build_root.resolve() / "build") not in command
    for value in (build_base, log_base, environment["COLCON_HOME"]):
        assert Path(value).parent.name.startswith("sim-dual-arm-ros-")
        assert not Path(value).exists()
    base_index = command.index("--base-paths")
    assert command[base_index + 1] == str(
        (shared / "common/interfaces/ros/robot_api_msgs").resolve()
    )
